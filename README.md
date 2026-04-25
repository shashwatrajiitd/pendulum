# Pendulum Physics Engine

A high-precision physics engine for N-pendulum systems (N = 1, 2, 3) with real-time browser visualization. Run up to 3 independent pendulum systems simultaneously from the same pivot, each with separate parameters and initial conditions. The engine achieves energy drift below 1e-9 over 60 seconds of simulation — four orders of magnitude better than the 1e-5 design target.

## Screenshots

### Single Pendulum (N=1)

![Single Pendulum](docs/screenshots/single_pendulum.png)

A single compound pendulum with configurable rod mass, bob mass, and bob radius. The phase-space plot shows the characteristic elliptical orbit of a conservative oscillator.

### Double Pendulum (N=2)

![Double Pendulum](docs/screenshots/double_pendulum.png)

A chaotic double pendulum released from large angles. The motion trail shows the sensitivity to initial conditions, while the energy plot confirms drift stays below 1e-9 over the simulation window.

### Triple Pendulum (N=3)

![Triple Pendulum](docs/screenshots/triple_pendulum.png)

A triple pendulum with three phase-space plots showing the fully chaotic dynamics. All three links exhibit aperiodic, space-filling trajectories.

### Multi-System Mode (up to 3 systems)

![Multi-System Pendulums](docs/screenshots/multi_pendulum.png)

Three independent pendulum systems (two double pendulums and one double pendulum with different ICs) running simultaneously from the same pivot at 61 FPS. Each system has its own color-coded trail, energy line, and per-system info panel. Phase-space plots show the selected system.

## Features

- **N = 1, 2, 3 pendulums** — serial chain with configurable rod lengths, masses, bob radii
- **Multi-system mode** — up to 3 independent pendulum systems sharing one pivot, each with separate parameters
- **Lagrangian mechanics** — manipulator-form EOM: M(theta) theta_ddot + C(theta, theta_dot) theta_dot + G(theta) = 0
- **Adaptive integration** — Dormand-Prince 5(4) with PI step-size control (atol = rtol = 1e-8 for WASM, 1e-10 for tests)
- **Energy conservation** — |dE/E_0| < 1e-9 over 60 seconds for chaotic double/triple pendulums
- **Speed control** — 1/8x to 8x simulation speed with real-time slider
- **47 validated tests** — period vs elliptic integral, energy conservation, time-reversal symmetry, zero-gravity invariants
- **WASM powered** — C++20 core compiled to WebAssembly (~306 KB), runs at 60 FPS in the browser
- **Real-time visualization** — Canvas2D pendulum renderer with color-coded motion trails, overlaid energy drift plot, and phase-space diagrams

## Quick Start

### Run the web app

```bash
cd web
npm install
npm run dev
```

Open http://localhost:5173 in your browser. Configure systems (up to 3), set N, initial angles, and click Play. Use the speed slider to slow down or speed up the simulation.

### Run the test suite

```bash
cmake -B build -G Ninja
cmake --build build
cd build && ctest --output-on-failure
```

All 47 tests should pass in under 4 seconds.

### Rebuild WASM (after modifying C++ code)

```bash
emcmake cmake -B build-wasm -G Ninja
cmake --build build-wasm
cp build-wasm/pendulum.{js,wasm} web/public/wasm/
```

## Architecture

```
C++ Physics Core (header-only, templated on N)
    |
    |-- Engine<N>: simulation driver
    |-- Dopri5<Dim>: generic adaptive ODE integrator
    |-- eom.hpp: M(theta), C(theta, theta_dot), G(theta) assembly + Cholesky solve
    |-- types.hpp: SystemConfig, State<N>, LinkParams
    |
    v
Emscripten WASM API (extern "C" functions)
    |
    v
TypeScript Bridge (PendulumEngine class)
    |
    v
React + Canvas2D Frontend (multi-system)
    |-- PendulumCanvas: all systems rendered from shared pivot
    |-- EnergyPlot: overlaid E(t) - E_0 per system
    |-- PhaseSpacePlot: theta_i vs theta_dot_i (selected system)
    |-- Controls: add/remove systems, per-system N/params/ICs, speed
    |-- InfoPanel: per-system t, E, |dE/E_0|, FPS
```

## Precision

| Metric | Target | Achieved |
|--------|--------|----------|
| Energy drift \|dE/E_0\| over 60 s (native, tol=1e-10) | < 1e-5 | ~1e-10 |
| Energy drift \|dE/E_0\| over 60 s (WASM, tol=1e-8) | < 1e-5 | ~1e-9 |
| Period accuracy vs elliptic integral (N=1) | < 1e-5 | < 1e-8 |
| Time-reversal error over 10 s round-trip | < 1e-5 | < 1e-8 |
| Zero-gravity velocity drift | < 1e-12 | < 1e-12 |
| Browser frame rate (3 systems, N=2 each) | 60 FPS | 60 FPS |

## Documentation

- **[Physics and Numerical Methods](docs/physics.md)** — Lagrangian formulation, EOM derivation, DOPRI5(4) integrator, validation strategy
- **[Programming Guide](docs/programming.md)** — code architecture, libraries, build system, WASM bindings, React frontend

## Tech Stack

| Component | Technology |
|-----------|-----------|
| Physics core | C++20, Eigen 3.4 |
| Integration | Dormand-Prince 5(4) with PI step control |
| Testing | Catch2 v3.5 |
| Build | CMake + Ninja |
| WASM | Emscripten |
| Frontend | React 18, TypeScript, Vite, Canvas2D |

## Requirements

- C++20 compiler (GCC 10+, Clang 13+, Apple Clang 14+)
- CMake 3.20+
- Node.js 18+
- Emscripten 3.0+ (for WASM builds)

Eigen and Catch2 are downloaded automatically by CMake.

## License

MIT
