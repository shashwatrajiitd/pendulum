# Programming Guide

This document describes the software architecture, libraries, build system, and code organization of the pendulum physics engine.

## 1. Tech Stack

| Layer | Technology | Purpose |
|-------|-----------|---------|
| Physics core | C++20 | High-precision numerical integration |
| Linear algebra | Eigen 3.4 | Fixed-size matrix operations (up to 6x6) |
| Testing | Catch2 v3.5 | Unit and validation test framework |
| Build system | CMake 3.20+ / Ninja | Cross-platform builds |
| WASM bindings | Emscripten (em++) | Compile C++ to WebAssembly |
| Frontend | React 18 + TypeScript | UI controls and visualization |
| Bundler | Vite 8 | Dev server and production build |
| Rendering | Canvas2D | Pendulum animation, plots |

## 2. Project Structure

```
pendulum/
├── CMakeLists.txt                 # Root build — native + WASM
├── .gitignore
├── pendulum_physics_engine_plan.md
│
├── src/
│   ├── core/
│   │   ├── types.hpp              # LinkParams, SystemConfig, State<N>
│   │   ├── pendulum_n1.hpp        # N=1 specialized coefficients (legacy)
│   │   └── eom.hpp                # General N-pendulum EOM assembler
│   ├── integrators/
│   │   └── dopri5.hpp             # Dormand-Prince 5(4) adaptive integrator
│   ├── engine.hpp                 # Engine<N> — top-level simulation driver
│   └── api/
│       └── wasm.cpp               # Emscripten C API bindings
│
├── tests/
│   ├── CMakeLists.txt
│   ├── test_smoke.cpp             # Eigen build verification
│   ├── test_types.cpp             # SystemConfig validation
│   ├── test_pendulum_n1.cpp       # N=1 coefficient and energy tests
│   ├── test_dopri5.cpp            # Integrator tests (harmonic osc, exp growth)
│   ├── test_engine_n1.cpp         # Engine<1> basic simulation tests
│   ├── test_engine_n2.cpp         # Engine<2> energy + time-reversal tests
│   ├── test_engine_n3.cpp         # Engine<3> energy + time-reversal tests
│   └── test_validation_n1.cpp     # Elliptic integral period, 60s energy, reversal
│
├── web/
│   ├── package.json
│   ├── vite.config.ts
│   ├── index.html
│   ├── public/wasm/
│   │   ├── pendulum.js            # Emscripten JS loader
│   │   └── pendulum.wasm          # Compiled physics engine (~306 KB)
│   └── src/
│       ├── App.tsx                 # Main app — multi-system engine mgmt + imperative canvas drawing
│       ├── main.tsx                # React entry point
│       ├── engine/
│       │   └── wasm-bridge.ts     # TypeScript WASM bridge (pre-allocated buffers)
│       └── components/
│           ├── Controls.tsx       # Multi-system controls with add/remove, speed slider
│           └── InfoPanel.tsx      # Per-system live state readout
│
└── docs/
    ├── physics.md                 # Mathematical formulation
    ├── programming.md             # This file
    └── screenshots/
```

## 3. C++ Core Design

### 3.1 Header-Only Library

The physics core is entirely header-only. There is no `.cpp` file to compile for the library itself — all code lives in `.hpp` headers. This simplifies the build and enables full inlining of the hot path.

The `pendulum_core` CMake target is an `INTERFACE` library that provides include paths and links Eigen.

### 3.2 Template Architecture

The engine is templated on `N` (pendulum count):

```cpp
template <int N>
class Engine { ... };
```

This gives us:
- **Stack-allocated fixed-size Eigen matrices** (no heap allocation in the hot path)
- **Full inlining** of the Cholesky solve and RHS evaluation
- **No virtual dispatch** in the integration loop
- Compile-time specialization for N=1, 2, 3

The integrator state vector has dimension `Dim = 2*N` (N angles + N angular velocities).

### 3.3 Key Types

```cpp
struct LinkParams {
    double L;       // rod length (m), must be > 0
    double m_rod;   // rod mass (kg), must be >= 0
    double m_bob;   // bob mass (kg), must be > 0
    double r_bob;   // bob radius (m), must be in [0, L/2]
};

struct SystemConfig {
    int N;                              // 1, 2, or 3
    std::array<LinkParams, 3> links;    // first N used
    double g = 9.80665;                 // gravitational acceleration
    void validate() const;              // throws on invariant violation
};

template <int N>
struct State {
    Eigen::Matrix<double, N, 1> theta;      // generalized positions
    Eigen::Matrix<double, N, 1> theta_dot;  // generalized velocities
    double t = 0.0;                         // simulation time
};
```

### 3.4 EOM Assembler (`eom.hpp`)

The `compute_coefficients<N>()` function precomputes the configuration-independent constants (alpha, beta, gamma) from the physical parameters. These are stored in a `Coefficients<N>` struct and reused at every RHS evaluation.

The `compute_acceleration<N>()` function:
1. Builds the mass matrix M(theta) from beta (diagonal) and alpha * cos(theta_i - theta_j) (off-diagonal)
2. Builds the RHS vector h = C(theta, theta_dot) theta_dot + G(theta)
3. Solves M theta_ddot = -h via Eigen's LLT Cholesky decomposition

### 3.5 Integrator (`dopri5.hpp`)

The `Dopri5<Dim>` class is a generic ODE integrator templated on state dimension. It knows nothing about pendulum physics — it takes a callable `RhsFn` and steps any ODE system.

Key methods:
- `step(t, y, h)` — attempt one step of size h, returns result with error estimate
- `suggest_h(h, err, prev_err)` — PI controller recommends next step size

The Engine constructor accepts configurable tolerances (`atol`, `rtol`, default 1e-10). The WASM build uses 1e-8 for better frame rate while native tests retain 1e-10 for maximum precision.

The Engine's `advance(dt)` method drives the integrator in a loop, accepting or rejecting steps until the target time is reached.

## 4. WASM Bindings

### 4.1 C API Layer (`wasm.cpp`)

Since Emscripten's `extern "C"` functions can't use templates or C++ types directly, the WASM layer uses a virtual dispatch pattern:

```cpp
struct EngineBase {
    virtual void advance(double dt) = 0;
    virtual double energy() const = 0;
    // ...
};

template <int N>
struct EngineWrapper : EngineBase {
    Engine<N> engine;
    // delegates to engine
};
```

The `engine_create()` function selects the right specialization based on N and returns an opaque pointer.

### 4.2 Exported Functions

| Function | Signature | Description |
|----------|----------|-------------|
| `engine_create` | `(N, params*, theta0*, thetaDot0*) -> handle` | Create engine instance |
| `engine_destroy` | `(handle) -> void` | Free engine |
| `engine_advance` | `(handle, dt) -> void` | Step simulation forward |
| `engine_time` | `(handle) -> double` | Get current time |
| `engine_energy` | `(handle) -> double` | Get total energy |
| `engine_energy_drift` | `(handle) -> double` | Get \|dE/E0\| |
| `engine_get_state` | `(handle, theta*, thetaDot*) -> void` | Read angles/velocities |
| `engine_get_positions` | `(handle, pos*, lengths*) -> void` | Get bob XY positions |

### 4.3 Build Configuration

The WASM build uses these Emscripten flags:
- `MODULARIZE=1` + `EXPORT_NAME='PendulumModule'` — creates a factory function, not a global
- `ALLOW_MEMORY_GROWTH=1` — dynamic memory
- `ENVIRONMENT='web'` — browser-only target
- `-fwasm-exceptions` — native WASM exception support (near-zero overhead on non-throwing path)
- `-O3` — full optimization
- Exported runtime methods: `setValue`, `getValue` for safe memory access

## 5. TypeScript WASM Bridge

The `PendulumEngine` class in `wasm-bridge.ts` wraps the C API:

- Loads the WASM module once (cached via static property — no re-downloads on reset)
- Marshals parameters to/from WASM memory using `setValue`/`getValue` (not raw HEAPF64, for Emscripten compatibility)
- Pre-allocates reusable WASM memory buffers at engine creation (no malloc/free per frame)
- Provides a clean TypeScript API: `advance(dt)`, `getState()`, `destroy()`

## 6. React Frontend

### 6.1 Multi-System Architecture

The app manages up to 3 independent pendulum systems (`PendulumSystem[]`), each with its own WASM engine instance, trail buffer, energy history, and phase-space data.

```
App (multi-system orchestrator)
├── Controls          # Per-system config (add/remove, N, links, ICs), speed slider, play/pause
├── PendulumCanvas    # Canvas2D: all systems drawn from shared pivot (imperative)
├── EnergyPlot        # Canvas2D: overlaid E(t) - E0 per system (imperative)
├── PhaseSpacePlot    # Canvas2D: theta_i vs theta_dot_i for selected system (imperative)
└── InfoPanel         # Per-system readout: t, E, |dE/E0|, FPS
```

### 6.2 Imperative Rendering Loop

Canvas drawing bypasses React's render cycle for 60 FPS performance:

1. Each frame calls `engine.advance(speed/60)` for every system — the integrator handles substepping
2. `engine.getState()` reads state via pre-allocated WASM memory buffers (no malloc/free per frame)
3. Drawing functions write directly to canvas refs — no React state updates for visualization
4. Energy/phase plots only redraw every 3rd frame; React InfoPanel state updates every 6th frame
5. Trail history (last 800 points), energy history (last 600), and phase-space history (last 600) per system are maintained in refs

### 6.3 Parameter Changes

Changing any system's parameters, gravity, or the number of systems triggers a full reset:
- The `resetEngines` callback depends on `[systems, g]`
- A `useEffect` watching `resetEngines` re-creates all WASM engines whenever parameters change
- This destroys old engines, clears all per-system data, and starts fresh

## 7. Build Instructions

### Native (tests)

```bash
cmake -B build -G Ninja
cmake --build build
cd build && ctest --output-on-failure
```

Requires: C++20 compiler, CMake 3.20+, Ninja (optional — omit `-G Ninja` to use Make).

Eigen and Catch2 are fetched automatically via CMake FetchContent.

### WASM

```bash
emcmake cmake -B build-wasm -G Ninja
cmake --build build-wasm
```

Produces `build-wasm/pendulum.js` and `build-wasm/pendulum.wasm`.

Copy to the web app: `cp build-wasm/pendulum.{js,wasm} web/public/wasm/`

### Frontend

```bash
cd web
npm install
npm run dev        # development server at localhost:5173
npm run build      # production build to web/dist/
```

## 8. Compiler Flags

| Flag | Purpose |
|------|---------|
| `-std=c++20` | Language standard |
| `-fno-fast-math` | Prevent FP reordering that breaks determinism |
| `-ffp-contract=off` | Prevent fused multiply-add that changes rounding |
| `-fwasm-exceptions` | Native WASM exception support (WASM build only) |
| `-O3` | Full optimization (WASM build) |

These flags ensure IEEE-754 compliant arithmetic, which is critical for reproducible energy conservation results. Native WASM exceptions provide near-zero overhead on the non-throwing path, unlike the JavaScript-based exception mechanism (`-sDISABLE_EXCEPTION_CATCHING=0`) which wraps every function call in try/catch.

## 9. Dependencies

| Dependency | Version | How Acquired |
|-----------|---------|--------------|
| Eigen | 3.4.0 | CMake FetchContent (auto-downloaded) |
| Catch2 | 3.5.2 | CMake FetchContent (auto-downloaded) |
| Emscripten | 5.0+ | System install (`brew install emscripten`) |
| Node.js | 18+ | System install |
| React | 18 | npm (via Vite template) |
| TypeScript | 5+ | npm |
| Vite | 8 | npm |
