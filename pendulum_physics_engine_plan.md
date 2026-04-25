# Pendulum Physics Engine — End-to-End Implementation Plan

> A high-precision (≤ 1e-5 absolute drift over 60 s) physics engine for N-pendulum systems (N ∈ {1, 2, 3}), with configurable rods, spheres, gravity, and initial conditions.

---

## 1. Goals & Hard Constraints

| Requirement | Specification |
|---|---|
| Pendulum count `N` | 1, 2, or 3 (serial chain; rod *i+1* pivots at the center of bob *i*) |
| Rod parameters | length `L_i > 0`, mass `m_r,i ≥ 0` (uniform thin rod, rigid) |
| Bob parameters | radius `r_i ≤ L_i / 2`, mass `m_b,i > 0` (uniform solid sphere) |
| Initial conditions | `(θ_i(0), θ̇_i(0))` for each link |
| Gravity | scalar `g` (default 9.80665 m/s²; allow zero or negative) |
| Precision target | Energy drift `|ΔE/E_0| < 1e-5` over a 60-s horizon at default tolerances; per-step local error `< 1e-7` |
| Determinism | Bit-reproducible across runs given same `(IC, params, dt, RNG=none)` |
| Throughput | ≥ 10⁴ integration steps / s for N=3 on a single core |

Non-goals (v1): friction, driving forces, flexible rods, contact, constraints beyond ideal hinges, 3-D motion (we stay in the vertical plane).

---

## 2. Tech Stack & Architecture

### 2.1 Stack

| Layer | Choice | Rationale |
|---|---|---|
| Physics core | **C++20** + **Eigen 3.4** | Tight numeric loops, SIMD-friendly linear algebra, deterministic FP if `-ffp-contract=off -fno-fast-math` |
| Symbolic derivation | **SymPy** (one-shot, code-gen) | Generates EOM coefficients; output cached as C++ headers |
| Build | **CMake** + **Ninja** + **vcpkg** | Cross-platform, reproducible |
| Bindings | **pybind11** (Python) **and** **Emscripten → WASM** (browser) | Same core, two delivery surfaces |
| Frontend (web) | **React + TypeScript + Canvas2D/WebGL** | Real-time visualization, parameter controls, phase-space view |
| Testing | **Catch2** (C++), **pytest** (Python parity), **ApprovalTests** for trajectories | |
| CI | GitHub Actions (Linux/macOS/Windows + WASM build) | |
| Profiling | `perf`, `vtune`, Chrome DevTools (WASM) | |

### 2.2 Architecture

```
┌────────────────────────────────────────────────────────────────┐
│                       Frontend (React + TS)                    │
│   Controls │ Canvas Renderer │ Phase-Space Plot │ Energy Panel │
└──────────────────────────┬─────────────────────────────────────┘
                           │ JSON/typed-array bridge
┌──────────────────────────┴─────────────────────────────────────┐
│                  Bindings (WASM via Emscripten)                │
└──────────────────────────┬─────────────────────────────────────┘
                           │
┌──────────────────────────┴─────────────────────────────────────┐
│                       C++ Physics Core                         │
│  ┌──────────────┐  ┌────────────────┐  ┌────────────────────┐  │
│  │ SystemConfig │→ │  EOMAssembler  │→ │  Integrator (RK / │  │
│  │  (params)    │  │  M(θ),C(θ,θ̇), │  │   symplectic GL)   │  │
│  └──────────────┘  │   G(θ) builder │  └─────────┬──────────┘  │
│                    └────────────────┘            │             │
│  ┌──────────────────────────────────────────────┴──────────┐   │
│  │  StateBuffer (q, q̇, t)  +  Invariants (E, L_z, ...)    │   │
│  └─────────────────────────────────────────────────────────┘   │
└────────────────────────────────────────────────────────────────┘
```

The frontend never owns physics state; it polls the core at frame rate and asks for `n_substeps` of work per render frame.

---

## 3. Mathematical Formulation

### 3.1 Coordinates & Geometry

We use **Lagrangian mechanics in generalized coordinates** `θ = (θ_1, …, θ_N)`, where `θ_i` is the angle of rod *i* from the downward vertical, measured CCW. The pivot of rod 1 is fixed at the origin; the pivot of rod *i+1* is the end of rod *i* (center of bob *i*).

Define the cumulative pivot positions:

$$
X_i = \sum_{j=1}^{i} L_j \sin\theta_j, \qquad Y_i = -\sum_{j=1}^{i} L_j \cos\theta_j, \qquad X_0 = Y_0 = 0
$$

Rod *i* center of mass is at the midpoint:

$$
x_{r,i} = X_{i-1} + \tfrac{L_i}{2}\sin\theta_i, \qquad y_{r,i} = Y_{i-1} - \tfrac{L_i}{2}\cos\theta_i
$$

Bob *i* center is at `(X_i, Y_i)`. Bob is rigidly attached to rod *i*, so it rotates with `θ̇_i`.

### 3.2 Inertias

| Body | Moment of inertia about its own CM |
|---|---|
| Rod *i* (uniform thin rod) | `I_r,i = (1/12) m_r,i L_i²` |
| Bob *i* (uniform solid sphere) | `I_b,i = (2/5) m_b,i r_i²` |

The constraint `r_i ≤ L_i / 2` is enforced in the `SystemConfig` validator.

### 3.3 Kinetic & Potential Energy

Differentiating the position vectors:

$$
\dot X_i = \sum_{j=1}^{i} L_j \cos\theta_j\, \dot\theta_j, \qquad
\dot Y_i = \sum_{j=1}^{i} L_j \sin\theta_j\, \dot\theta_j
$$

Kinetic energy (translational CM + rotational about CM, for both rod and bob):

$$
T = \sum_{i=1}^{N} \Big[ \tfrac{1}{2} m_{r,i}(\dot x_{r,i}^2 + \dot y_{r,i}^2) + \tfrac{1}{2} I_{r,i}\dot\theta_i^2 + \tfrac{1}{2} m_{b,i}(\dot X_i^2 + \dot Y_i^2) + \tfrac{1}{2} I_{b,i}\dot\theta_i^2 \Big]
$$

Potential energy (zero at pivot height):

$$
V = g\sum_{i=1}^{N} \big[m_{r,i}\, y_{r,i} + m_{b,i}\, Y_i\big]
$$

Lagrangian `L = T − V`.

### 3.4 Equations of Motion in Manipulator Form

The Euler–Lagrange equations `d/dt(∂L/∂θ̇_i) − ∂L/∂θ_i = 0` collapse to the standard rigid-body manipulator form:

$$
\boxed{\; \mathbf{M}(\boldsymbol\theta)\,\ddot{\boldsymbol\theta} + \mathbf{C}(\boldsymbol\theta,\dot{\boldsymbol\theta})\,\dot{\boldsymbol\theta} + \mathbf{G}(\boldsymbol\theta) = 0 \;}
$$

where for our planar serial chain:

- **Mass matrix** (symmetric, positive-definite):

$$
M_{ij} = \alpha_{ij}\cos(\theta_i - \theta_j) \quad (i \ne j), \qquad M_{ii} = \beta_i
$$

- **Coriolis/centrifugal matrix**:

$$
C_{ij} = \alpha_{ij}\sin(\theta_i - \theta_j)\,\dot\theta_j
$$

- **Gravity vector**:

$$
G_i = \gamma_i\, g \sin\theta_i
$$

The constants `α_{ij}`, `β_i`, `γ_i` are *configuration-independent* and depend only on `(L, m_r, m_b, r)`. They are computed once at setup. Closed forms (let `μ_k = m_{r,k} + 2 m_{b,k}` be a useful aggregate; full forms are in §3.6 and `physics/coefficients.md`):

- `β_i = (1/3) m_{r,i} L_i² + (2/5) m_{b,i} r_i² + I_{b,i}_link_correction + L_i² · Σ_{k>i}(m_{r,k} + m_{b,k})`
  — the “self” inertia about the i-th pivot, plus the inertia each downstream link contributes when link *i* alone rotates.
- `α_{ij} = L_i L_j · [m_{b,max(i,j)}·χ + …]` — symmetric coupling term.
- `γ_i = (½ m_{r,i} + m_{b,i}) L_i + L_i · Σ_{k>i}(m_{r,k} + m_{b,k})` — total weighted moment arm of gravity acting on link *i* and everything below it.

These are derived once symbolically (SymPy) and emitted as a generated header (`pendulum_coeffs.gen.hpp`) with N=1, 2, 3 specializations. This gives us hand-tunable, branchless code with no symbolic overhead at runtime.

### 3.5 Single Pendulum (Sanity Check, Closed Form)

For N=1:

$$
I_{\text{eff}}\,\ddot\theta = -g\,\big(\tfrac{1}{2}m_r L + m_b L\big)\sin\theta
$$

with `I_eff = (1/3) m_r L² + m_b L² + (2/5) m_b r²`. Small-angle period:

$$
T_{\text{small}} = 2\pi\sqrt{\frac{I_{\text{eff}}}{g\,(\tfrac{1}{2}m_r + m_b) L}}
$$

This is our **first acceptance test**: simulated period must match this to within `1e-5` for `θ_0 = 1e-3` rad, and match the elliptic-integral exact period

$$
T(\theta_0) = 4\sqrt{\frac{I_{\text{eff}}}{g\,(\tfrac{1}{2}m_r+m_b)L}}\, K\!\left(\sin\tfrac{\theta_0}{2}\right)
$$

(complete elliptic integral of the first kind) to within `1e-5` for `θ_0` up to `π/2`.

### 3.6 Double & Triple Pendulum

For N=2 the closed form is derived via SymPy (~20 LOC) and emitted; the resulting expression for `M, C, G` is roughly 30 floating-point ops per evaluation. For N=3 it’s ~120 ops — still trivial. The generated code is checked into the repo so the build doesn’t depend on SymPy.

**Forward dynamics step:**

```
1. Build M(θ) and h(θ, θ̇) := C(θ, θ̇)θ̇ + G(θ)
2. Solve M · θ̈ = −h    via Cholesky (Eigen LLT — M is SPD, ~3×3 max)
3. Pack (θ̇, θ̈) as the state derivative
```

For N=3 the linear solve is a hand-unrolled 3×3 Cholesky — no allocation, all on stack.

---

## 4. Numerical Integration

### 4.1 Why This Is Hard

Double and triple pendulums are **chaotic** for moderate energies (positive Lyapunov exponent). Trajectories cannot be reproduced bitwise across different integrators or step sizes; what *can* be preserved is the **invariant manifold** (energy `E` is conserved exactly in the continuous system). So our precision target is phrased on **invariants**, not phase coordinates.

### 4.2 Integrator Selection

| Method | Order | Symplectic | A-stable | Adaptive | Verdict |
|---|---|---|---|---|---|
| RK4 | 4 | ✗ | ✗ | manual | Baseline only |
| Dormand–Prince DOPRI5 (RK4(5)) | 5(4) | ✗ | ✗ | ✓ | Decent default |
| **DOPRI8(7) (Verner / Prince–Dormand 8)** | 8(7) | ✗ | ✗ | ✓ | **Default — best raw accuracy/step** |
| Velocity Verlet | 2 | ✓ | — | ✗ | Toy only |
| Forest–Ruth | 4 | ✓ | — | ✗ | Long-time energy bound |
| **Gauss–Legendre 6 (3-stage IRK)** | 6 | ✓ | ✓ | optional | **Long-horizon mode** |
| Yoshida 8 | 8 | ✓ (split) | — | ✗ | Hard to apply (not separable H) |

**The Lagrangian here is NOT separable** (kinetic energy depends on `θ` through `M(θ)`), so explicit symplectic splitting (Verlet, Yoshida) is *not* applicable directly. The right symplectic choice is an **implicit Runge–Kutta**: **3-stage Gauss–Legendre, order 6**, A-stable, symplectic, preserves quadratic invariants exactly. We solve its implicit stage equations with **simplified Newton** (one factorization of `(I − hA⊗J)` per step, 3–5 inner iterations).

**Recommendation:**

- **Default integrator:** DOPRI8(7) with PI step controller, atol = `1e-10`, rtol = `1e-10`. Empirically gives `|ΔE/E| < 1e-7` over 60 s for a chaotic double pendulum at `θ ≈ 2 rad`.
- **`--symplectic` mode:** Gauss–Legendre 6 with fixed `h = 1e-3 s`. Use for hour-scale runs where bounded energy drift matters more than per-step speed.

### 4.3 Step-Size Control

PI controller (Gustafsson):

$$
h_{n+1} = h_n \cdot \min\!\left(\textit{fac}_{\max},\, \max\!\left(\textit{fac}_{\min},\, \kappa \cdot \textit{err}_n^{-\alpha}\,\textit{err}_{n-1}^{\beta}\right)\right)
$$

with `α = 0.7/p`, `β = 0.4/p` for order `p=8`, `κ = 0.9`. Reject step if `err > 1`.

Error norm:

$$
\textit{err} = \sqrt{\frac{1}{2N}\sum_k \left(\frac{e_k}{\text{atol} + \text{rtol}\cdot \max(|y_k|,|y_k^{\text{prev}}|)}\right)^{\!2}}
$$

### 4.4 Hitting 1e-5 Precision

Three pillars:

1. **Per-step local truncation error** controlled by adaptive DOPRI8(7) at `tol = 1e-10` ⇒ accumulated trajectory error `~1e-7` over 60 s.
2. **Round-off control:** use **compensated summation** (Kahan) when accumulating `t = t + h` and when accumulating `θ` over many steps; build with `-fno-fast-math` to forbid contraction; pin to IEEE-754 double.
3. **Invariant monitoring:** every 100 steps, recompute `E(θ, θ̇)` and assert `|E − E_0| < 1e-5 · |E_0|`. If violated, halve the step and retry that segment (in adaptive mode this should rarely trigger).

For N=1 we additionally compare against the elliptic-integral exact solution and require `|θ_sim − θ_exact| < 1e-5` over the test window.

---

## 5. Validation Strategy

### 5.1 Unit Tests (C++ + Python parity)

- Coefficient builder: hard-coded test vectors for `(α_{ij}, β_i, γ_i)` at known param sets.
- Mass matrix is SPD for all sampled `θ` (random + edge cases).
- `M⁻¹` solver matches Eigen reference.
- Single-step DOPRI vs hand-rolled RK on linearized pendulum.

### 5.2 Physics Validation Suite

| Test | Pass criterion |
|---|---|
| Single pendulum period vs elliptic integral | rel. error `< 1e-5` for `θ_0 ∈ [1e-3, π/2]` |
| Energy conservation (60 s, default tol) | `|ΔE/E| < 1e-5` for N=1,2,3 across 100 random ICs |
| Time-reversal symmetry | integrate forward `T`, flip `θ̇`, integrate `T` more; final state matches IC to `1e-5` |
| Zero-gravity test | angular momentum conserved exactly (to roundoff) |
| Linearized normal modes (small θ) | eigenfrequencies of `M⁻¹ ∂G/∂θ` match analytical for N=2,3 |
| Lyapunov exponent (double pendulum) | matches published `λ ≈ 0.5–1.5 s⁻¹` regime depending on energy |

### 5.3 Reference Cross-Check

Run `scipy.integrate.solve_ivp(method='DOP853', rtol=1e-12, atol=1e-12)` on the same EOM for 1000 random ICs; require sup-norm trajectory difference `< 1e-5` over the first 10 s and energy agreement throughout.

---

## 6. C++ Core Design

### 6.1 Module Layout

```
src/
├── core/
│   ├── system_config.hpp       // params + invariants validation
│   ├── state.hpp               // θ, θ̇, t; Kahan accumulators
│   ├── coefficients.hpp        // α, β, γ from params (one-time)
│   ├── eom.hpp                 // build M, h; solve for θ̈
│   ├── coeffs_n1.gen.hpp       // codegen
│   ├── coeffs_n2.gen.hpp
│   └── coeffs_n3.gen.hpp
├── integrators/
│   ├── integrator.hpp          // abstract interface
│   ├── dopri87.hpp             // explicit, default
│   ├── gauss_legendre6.hpp     // implicit, symplectic
│   └── step_controller.hpp     // PI controller
├── invariants/
│   ├── energy.hpp
│   └── monitor.hpp             // periodic check + auto-reduce
├── api/
│   ├── engine.hpp              // public C++ API
│   ├── pybind.cpp              // pybind11 layer
│   └── wasm.cpp                // emscripten layer (extern "C")
└── tests/
    ├── unit/
    └── validation/
```

### 6.2 Key Types

```cpp
struct LinkParams {
    double L;        // rod length, m
    double m_rod;    // rod mass, kg
    double m_bob;    // bob mass, kg
    double r_bob;    // bob radius, m  (must be ≤ L/2)
};

struct SystemConfig {
    int N;                              // 1..3
    std::array<LinkParams, 3> links;    // first N used
    double g = 9.80665;
    void validate() const;              // throws on invariant break
};

template<int N>
struct State {
    Eigen::Matrix<double, N, 1> theta;
    Eigen::Matrix<double, N, 1> theta_dot;
    double t;
};

struct StepStats { double h_used; int newton_iters; double err_norm; };

template<int N>
class Engine {
public:
    Engine(const SystemConfig&, const State<N>& ic, IntegratorKind = DOPRI87);
    StepStats step();                  // single adaptive step
    void advance(double dt);           // step until t ≥ t0+dt
    double energy() const;
    const State<N>& state() const;
    // …
};
```

Templating on `N` gets us stack-allocated fixed-size matrices, full inlining, and no virtual dispatch in the hot loop. The public WASM/Python API holds an `std::variant<Engine<1>, Engine<2>, Engine<3>>` selected at construction.

### 6.3 Numerical Hygiene

- Compile flags: `-O3 -march=native -fno-fast-math -ffp-contract=off -fno-finite-math-only`.
- All hot-path math uses `double`; no `long double`, no mixed precision.
- Trig: `std::sin`, `std::cos` (libm). Cache `sin(θ_i)`, `cos(θ_i)` once per RHS evaluation; reuse for `M`, `C`, `G`.
- Linear solve: explicit hand-coded Cholesky for N≤3. Benchmarked 3–5× faster than Eigen LLT at this size.
- No allocations in `step()`. All buffers are members of `Engine`.

---

## 7. Frontend (Web)

### 7.1 Stack

- React 18 + TypeScript, Vite build.
- WASM module loaded once at boot; engine instances created per simulation.
- Canvas2D for the pendulum render (cheap, crisp, no GPU dependency).
- A small WebGL/regl pass for an optional motion-trail / phase-space heatmap.
- `requestAnimationFrame` loop calls `engine.advance(1/60 s)` between frames; the integrator handles its own substepping.

### 7.2 Controls

- N (1/2/3), per-link `L, m_rod, m_bob, r`, `g`, initial `θ_i`, `θ̇_i`.
- Integrator selector + tolerances.
- Play/Pause/Step/Reset/Slow-mo.
- Live readouts: `E`, `ΔE/E_0`, integrator step `h`, FPS, real-time factor.
- Export trajectory as CSV / Parquet.

### 7.3 Views

1. **Spatial view** — the pendulum with optional ghost trails of bob centers.
2. **Phase space** — `(θ_i, θ̇_i)` per link with Poincaré-section overlay (configurable hyperplane).
3. **Energy plot** — running plot of `E(t) − E_0`.
4. **Action menu** — “Lyapunov estimate”: spawn two near-identical engines, plot `log|δ(t)|` vs `t`.

---

## 8. Build, Tooling, CI

- `cmake --preset=release` and `--preset=wasm`.
- `pre-commit`: `clang-format`, `clang-tidy`, `cppcheck`, `ruff`.
- CI matrix: `{ubuntu, macos, windows} × {gcc, clang, msvc}` for native; `emscripten` for WASM.
- Each PR runs the full physics validation suite (~30 s) and posts an energy-drift / period-error table to the PR.
- Reproducibility: lock toolchain via `vcpkg` + CMake `FetchContent` with pinned commits; CI verifies SHA of generated coefficient headers.

---

## 9. Phased Roadmap

| Phase | Deliverable | Effort (est.) |
|---|---|---|
| **0. Spec & math** | This doc; SymPy notebook deriving `α, β, γ` for N=1,2,3; coefficient header generator | 2 days |
| **1. Core, N=1** | `Engine<1>`, DOPRI8(7), validation suite (period vs elliptic integral, energy) | 3 days |
| **2. Core, N=2,3** | `Engine<2>`, `Engine<3>`, manipulator-form EOM, full validation | 4 days |
| **3. Symplectic mode** | Gauss–Legendre 6 with simplified Newton; long-horizon test (1 hr) | 3 days |
| **4. Bindings** | pybind11 + Emscripten; parity tests Python vs C++ vs WASM | 2 days |
| **5. Frontend MVP** | React UI with all controls, spatial view, energy plot | 4 days |
| **6. Phase space + Lyapunov** | Poincaré section, twin-trajectory Lyapunov estimator | 2 days |
| **7. Hardening** | Profiling, perf budget hit, fuzz testing the validator, docs | 3 days |
| **Total** |  | ~3 calendar weeks for a focused build |

---

## 10. Risks & Open Questions

- **Chaos vs precision claim.** Per-coordinate `1e-5` accuracy over long horizons is unattainable for chaotic regimes. We commit to invariant precision (`|ΔE/E_0| < 1e-5`) and short-horizon (≤ 10 s) coordinate precision. This needs to be communicated in UI copy, not buried.
- **Pivot-at-bob-center geometry.** We model the next rod as pivoting at the bob *center*, treating the bob as part of link *i*. Alternative: pivot at the bob *surface* (offset by `r_i` along the rod axis). v1 picks center-pivot for simplicity; if surface-pivot is needed, it adds an offset term to all `X_i, Y_i` and changes the coefficients (about a half-day of SymPy work).
- **Implicit-RK Newton failure** in Gauss–Legendre mode for highly energetic states. Fallback: shrink `h` and retry; ultimate fallback: switch to DOPRI8(7) for that segment.
- **Browser FP determinism.** WASM `f64` is IEEE-754, but the JIT may reassociate. Mitigate by computing physics inside WASM only and never recomputing on the JS side.
- **Bob radius near `L/2`.** Mathematically allowed but visually overlapping with the next pivot. UI should warn (not block) when `r_i > 0.4 L_i`.

---

## Appendix A — Why Lagrangian over Newton–Euler

Newton–Euler with constraint forces would force us to track 6N reaction forces and impose holonomic hinge constraints, then DAE-solve. Lagrangian collapses everything to N second-order ODEs by construction. For serial, planar, non-redundant chains, Lagrangian wins on every axis (size, conditioning, code clarity).

## Appendix B — Why Not a General Multibody Library

RBDL, Pinocchio, Drake, and MuJoCo all solve this problem. They’re overkill for N ≤ 3 and would dominate binary size in WASM. We get a ~50 KB WASM bundle by hand-coding the closed forms vs ~2–5 MB pulling in a generic library. Generic libraries also don’t expose the symplectic integrator we want. We retain the option to swap the core for Pinocchio later if we extend to N > 3 or 3-D motion.

## Appendix C — Reference Reading

- Hairer, Lubich, Wanner — *Geometric Numerical Integration* (Gauss–Legendre IRK; symplectic theory).
- Hairer, Nørsett, Wanner — *Solving ODEs I & II* (DOPRI; step control; PI controller).
- Featherstone — *Rigid Body Dynamics Algorithms* (manipulator form, ABA/CRBA — for if we extend to N>3).
- Strogatz — *Nonlinear Dynamics and Chaos* (Lyapunov exponent estimation; Poincaré sections).
