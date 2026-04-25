# Pendulum Physics Engine — Phase 0+1: N=1 Core

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a high-precision single-pendulum engine with adaptive DOPRI5(4) integration, validated against elliptic-integral exact periods and energy conservation (|ΔE/E₀| < 1e-5 over 60 s).

**Architecture:** Header-only C++20 library templated on N (pendulum count). Phase 1 implements N=1 only. The integrator is generic (works on any ODE), the physics layer computes θ̈ from the Lagrangian EOM, and Engine<1> wires them together. All hot-path math uses `double` with `-fno-fast-math`.

**Tech Stack:** C++20, Eigen 3.4 (FetchContent), Catch2 v3 (FetchContent), CMake + Ninja

---

## File Structure

```
pendulum/
├── CMakeLists.txt                  # Root: project, fetch deps, flags
├── .gitignore
├── src/
│   ├── core/
│   │   ├── types.hpp               # LinkParams, SystemConfig, State<N>
│   │   └── pendulum_n1.hpp         # N=1 coefficients, acceleration, energy
│   ├── integrators/
│   │   └── dopri5.hpp              # DOPRI5(4) with PI step controller
│   └── engine.hpp                  # Engine<N> public API (N=1 specialization)
├── tests/
│   ├── CMakeLists.txt
│   ├── test_types.cpp
│   ├── test_pendulum_n1.cpp
│   ├── test_dopri5.cpp
│   ├── test_engine_n1.cpp
│   └── test_validation_n1.cpp      # Elliptic integral, energy, time-reversal
└── docs/
    └── superpowers/plans/
```

---

### Task 1: Project Skeleton & Build System

**Files:**
- Create: `CMakeLists.txt`
- Create: `.gitignore`
- Create: `tests/CMakeLists.txt`
- Create: `tests/test_smoke.cpp`

- [ ] **Step 1: Create `.gitignore`**

```
build/
build-*/
.cache/
compile_commands.json
*.o
*.a
*.so
*.dylib
.DS_Store
```

- [ ] **Step 2: Create root `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.20)
project(pendulum LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Numerical precision: forbid FP reordering
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    add_compile_options(-fno-fast-math -ffp-contract=off)
endif()

include(FetchContent)

FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    GIT_SHALLOW TRUE
)

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.5.2
    GIT_SHALLOW TRUE
)

FetchContent_MakeAvailable(eigen Catch2)

add_library(pendulum_core INTERFACE)
target_include_directories(pendulum_core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(pendulum_core INTERFACE Eigen3::Eigen)

enable_testing()
add_subdirectory(tests)
```

- [ ] **Step 3: Create `tests/CMakeLists.txt` with smoke test**

```cmake
include(CTest)
include(Catch)

add_executable(test_smoke test_smoke.cpp)
target_link_libraries(test_smoke PRIVATE pendulum_core Catch2::Catch2WithMain)
catch_discover_tests(test_smoke)
```

- [ ] **Step 4: Create `tests/test_smoke.cpp`**

```cpp
#include <catch2/catch_test_macros.hpp>
#include <Eigen/Dense>

TEST_CASE("Eigen smoke test") {
    Eigen::Vector2d v(1.0, 2.0);
    REQUIRE(v.norm() == Catch::Approx(std::sqrt(5.0)));
}
```

- [ ] **Step 5: Build and verify**

```bash
cmake -B build -G Ninja
cmake --build build
cd build && ctest --output-on-failure
```

- [ ] **Step 6: Commit**

```bash
git add CMakeLists.txt .gitignore tests/ src/
git commit -m "build: project skeleton with Eigen + Catch2 via FetchContent"
```

---

### Task 2: Core Types — SystemConfig & State

**Files:**
- Create: `src/core/types.hpp`
- Create: `tests/test_types.cpp`
- Modify: `tests/CMakeLists.txt`

- [ ] **Step 1: Write the test file `tests/test_types.cpp`**

```cpp
#include <catch2/catch_test_macros.hpp>
#include "core/types.hpp"

using namespace pendulum;

TEST_CASE("Valid N=1 config passes validation") {
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = {1.0, 0.5, 1.0, 0.05};
    REQUIRE_NOTHROW(cfg.validate());
}

TEST_CASE("N=0 throws") {
    SystemConfig cfg;
    cfg.N = 0;
    REQUIRE_THROWS_AS(cfg.validate(), std::invalid_argument);
}

TEST_CASE("N=4 throws") {
    SystemConfig cfg;
    cfg.N = 4;
    REQUIRE_THROWS_AS(cfg.validate(), std::invalid_argument);
}

TEST_CASE("Negative rod mass throws") {
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = {1.0, -0.1, 1.0, 0.05};
    REQUIRE_THROWS_AS(cfg.validate(), std::invalid_argument);
}

TEST_CASE("Zero bob mass throws") {
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = {1.0, 0.5, 0.0, 0.05};
    REQUIRE_THROWS_AS(cfg.validate(), std::invalid_argument);
}

TEST_CASE("Bob radius > L/2 throws") {
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = {1.0, 0.5, 1.0, 0.6};
    REQUIRE_THROWS_AS(cfg.validate(), std::invalid_argument);
}

TEST_CASE("Zero rod length throws") {
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = {0.0, 0.5, 1.0, 0.0};
    REQUIRE_THROWS_AS(cfg.validate(), std::invalid_argument);
}

TEST_CASE("State<1> default is zero") {
    State<1> s;
    REQUIRE(s.theta[0] == 0.0);
    REQUIRE(s.theta_dot[0] == 0.0);
    REQUIRE(s.t == 0.0);
}

TEST_CASE("Zero gravity is allowed") {
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = {1.0, 0.0, 1.0, 0.05};
    cfg.g = 0.0;
    REQUIRE_NOTHROW(cfg.validate());
}

TEST_CASE("Negative gravity is allowed") {
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = {1.0, 0.0, 1.0, 0.05};
    cfg.g = -9.81;
    REQUIRE_NOTHROW(cfg.validate());
}
```

- [ ] **Step 2: Run tests — expect FAIL (types.hpp doesn't exist yet)**

```bash
cmake --build build 2>&1 | grep -i error
```

- [ ] **Step 3: Create `src/core/types.hpp`**

```cpp
#pragma once
#include <array>
#include <stdexcept>
#include <string>
#include <Eigen/Dense>

namespace pendulum {

struct LinkParams {
    double L = 1.0;
    double m_rod = 0.0;
    double m_bob = 1.0;
    double r_bob = 0.05;
};

struct SystemConfig {
    int N = 1;
    std::array<LinkParams, 3> links = {};
    double g = 9.80665;

    void validate() const {
        if (N < 1 || N > 3)
            throw std::invalid_argument("N must be 1, 2, or 3");
        for (int i = 0; i < N; ++i) {
            const auto& lk = links[i];
            if (lk.L <= 0.0)
                throw std::invalid_argument("Link " + std::to_string(i) + ": L must be > 0");
            if (lk.m_bob <= 0.0)
                throw std::invalid_argument("Link " + std::to_string(i) + ": m_bob must be > 0");
            if (lk.m_rod < 0.0)
                throw std::invalid_argument("Link " + std::to_string(i) + ": m_rod must be >= 0");
            if (lk.r_bob < 0.0 || lk.r_bob > lk.L / 2.0)
                throw std::invalid_argument("Link " + std::to_string(i) + ": r_bob must be in [0, L/2]");
        }
    }
};

template <int N>
struct State {
    Eigen::Matrix<double, N, 1> theta = Eigen::Matrix<double, N, 1>::Zero();
    Eigen::Matrix<double, N, 1> theta_dot = Eigen::Matrix<double, N, 1>::Zero();
    double t = 0.0;
};

} // namespace pendulum
```

- [ ] **Step 4: Add test_types to `tests/CMakeLists.txt` and run**

Add:
```cmake
add_executable(test_types test_types.cpp)
target_link_libraries(test_types PRIVATE pendulum_core Catch2::Catch2WithMain)
catch_discover_tests(test_types)
```

```bash
cmake --build build && cd build && ctest -R test_types --output-on-failure
```

- [ ] **Step 5: Commit**

```bash
git add src/core/types.hpp tests/test_types.cpp tests/CMakeLists.txt
git commit -m "feat: add SystemConfig, LinkParams, State<N> with validation"
```

---

### Task 3: N=1 Physics — Coefficients, Acceleration, Energy

**Files:**
- Create: `src/core/pendulum_n1.hpp`
- Create: `tests/test_pendulum_n1.cpp`
- Modify: `tests/CMakeLists.txt`

**Math recap (N=1):**
- `β₁ = (1/3)m_r L² + m_b L² + (2/5)m_b r²`  (effective moment of inertia)
- `γ₁ = (m_r/2 + m_b) L`  (gravity moment arm)
- `θ̈ = -(γ₁ g sin θ) / β₁`
- `T = (1/2) β₁ θ̇²`
- `V = -γ₁ g cos θ`
- `E = T + V`

- [ ] **Step 1: Write `tests/test_pendulum_n1.cpp`**

```cpp
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "core/pendulum_n1.hpp"
#include <cmath>

using namespace pendulum;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

TEST_CASE("Coefficients for point-mass pendulum (m_rod=0, r=0)") {
    LinkParams lk{1.0, 0.0, 1.0, 0.0};
    auto c = compute_coeffs_n1(lk);
    REQUIRE_THAT(c.beta, WithinAbs(1.0, 1e-15));   // m_b * L^2
    REQUIRE_THAT(c.gamma, WithinAbs(1.0, 1e-15));   // m_b * L
}

TEST_CASE("Coefficients for rod-only pendulum (m_bob very small)") {
    LinkParams lk{2.0, 3.0, 1e-10, 0.0};
    auto c = compute_coeffs_n1(lk);
    REQUIRE_THAT(c.beta, WithinRel(3.0 * 4.0 / 3.0, 1e-9));   // m_r*L^2/3 = 4.0
    REQUIRE_THAT(c.gamma, WithinRel(3.0 * 2.0 / 2.0, 1e-9));   // m_r*L/2 = 3.0
}

TEST_CASE("Acceleration at theta=0 is zero") {
    LinkParams lk{1.0, 0.0, 1.0, 0.0};
    auto c = compute_coeffs_n1(lk);
    REQUIRE_THAT(acceleration_n1(c, 9.80665, 0.0), WithinAbs(0.0, 1e-15));
}

TEST_CASE("Acceleration at theta=pi/2 matches -g/L for point mass") {
    LinkParams lk{1.0, 0.0, 1.0, 0.0};
    auto c = compute_coeffs_n1(lk);
    double acc = acceleration_n1(c, 9.80665, M_PI / 2.0);
    REQUIRE_THAT(acc, WithinRel(-9.80665, 1e-12));
}

TEST_CASE("Energy is conserved symbolically") {
    LinkParams lk{1.5, 0.5, 2.0, 0.1};
    auto c = compute_coeffs_n1(lk);
    double g = 9.80665;
    double theta = 1.0, theta_dot = 0.5;
    double T = kinetic_energy_n1(c, theta_dot);
    double V = potential_energy_n1(c, g, theta);
    REQUIRE_THAT(total_energy_n1(c, g, theta, theta_dot), WithinAbs(T + V, 1e-15));
}

TEST_CASE("Energy at rest hanging down is minimum") {
    LinkParams lk{1.0, 0.0, 1.0, 0.0};
    auto c = compute_coeffs_n1(lk);
    double E_bottom = total_energy_n1(c, 9.80665, 0.0, 0.0);
    double E_side = total_energy_n1(c, 9.80665, M_PI / 2.0, 0.0);
    REQUIRE(E_bottom < E_side);
}
```

- [ ] **Step 2: Run tests — expect FAIL**

- [ ] **Step 3: Create `src/core/pendulum_n1.hpp`**

```cpp
#pragma once
#include "types.hpp"
#include <cmath>

namespace pendulum {

struct CoeffsN1 {
    double beta;
    double gamma;
};

inline CoeffsN1 compute_coeffs_n1(const LinkParams& lk) {
    double L = lk.L, mr = lk.m_rod, mb = lk.m_bob, r = lk.r_bob;
    return {
        mr * L * L / 3.0 + mb * L * L + 0.4 * mb * r * r,
        (0.5 * mr + mb) * L
    };
}

inline double acceleration_n1(const CoeffsN1& c, double g, double theta) {
    return -(c.gamma * g * std::sin(theta)) / c.beta;
}

inline double kinetic_energy_n1(const CoeffsN1& c, double theta_dot) {
    return 0.5 * c.beta * theta_dot * theta_dot;
}

inline double potential_energy_n1(const CoeffsN1& c, double g, double theta) {
    return -c.gamma * g * std::cos(theta);
}

inline double total_energy_n1(const CoeffsN1& c, double g, double theta, double theta_dot) {
    return kinetic_energy_n1(c, theta_dot) + potential_energy_n1(c, g, theta);
}

} // namespace pendulum
```

- [ ] **Step 4: Add to CMakeLists, build, run**

- [ ] **Step 5: Commit**

---

### Task 4: DOPRI5(4) Integrator with PI Step Controller

**Files:**
- Create: `src/integrators/dopri5.hpp`
- Create: `tests/test_dopri5.cpp`
- Modify: `tests/CMakeLists.txt`

**Key details:**
- Dormand–Prince 5(4) with 7 stages (FSAL)
- PI step controller: `h' = h · clamp(κ · err^{-α} · prev_err^{β}, fac_min, fac_max)` where `α=0.7/5, β=0.4/5, κ=0.9`
- Error norm: `√(1/Dim · Σ(e_i / (atol + rtol·max(|y_i|,|y_new_i|)))²)`
- Test on harmonic oscillator y'' = -y with exact solution y(t) = cos(t)

- [ ] **Step 1: Write `tests/test_dopri5.cpp`**

```cpp
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "integrators/dopri5.hpp"
#include <cmath>

using namespace pendulum;
using Catch::Matchers::WithinAbs;

TEST_CASE("DOPRI5 solves harmonic oscillator y''=-y") {
    // State: [y, y'] = [cos(t), -sin(t)]
    auto rhs = [](double, const Eigen::Vector2d& y) -> Eigen::Vector2d {
        return {y[1], -y[0]};
    };

    Dopri5<2> solver(rhs, 1e-12, 1e-12);
    Eigen::Vector2d y(1.0, 0.0);
    double t = 0.0;
    double h = 0.1;
    double prev_err = 0.0;

    while (t < 2 * M_PI) {
        double h_try = std::min(h, 2 * M_PI - t);
        auto res = solver.step(t, y, h_try);
        if (res.accepted) {
            y = res.y;
            t = res.t;
            h = solver.suggest_h(h_try, res.err_norm, prev_err);
            prev_err = res.err_norm;
        } else {
            h = solver.suggest_h(h_try, res.err_norm, prev_err);
        }
    }

    REQUIRE_THAT(y[0], WithinAbs(1.0, 1e-8));
    REQUIRE_THAT(y[1], WithinAbs(0.0, 1e-8));
}

TEST_CASE("DOPRI5 step rejects large error") {
    auto rhs = [](double, const Eigen::Vector2d& y) -> Eigen::Vector2d {
        return {y[1], -y[0]};
    };

    Dopri5<2> solver(rhs, 1e-14, 1e-14);
    Eigen::Vector2d y(1.0, 0.0);
    auto res = solver.step(0.0, y, 10.0);  // huge step
    REQUIRE_FALSE(res.accepted);
}
```

- [ ] **Step 2: Run — expect FAIL**

- [ ] **Step 3: Create `src/integrators/dopri5.hpp`**

Full Dormand–Prince 5(4) implementation with the standard Butcher tableau coefficients:
- `b  = [35/384, 0, 500/1113, 125/192, -2187/6784, 11/84, 0]`
- `b* = [5179/57600, 0, 7571/16695, 393/640, -92097/339200, 187/2100, 1/40]`

- [ ] **Step 4: Build, run, verify tests pass**

- [ ] **Step 5: Commit**

---

### Task 5: Engine<1> Assembly

**Files:**
- Create: `src/engine.hpp`
- Create: `tests/test_engine_n1.cpp`
- Modify: `tests/CMakeLists.txt`

- [ ] **Step 1: Write `tests/test_engine_n1.cpp`**

```cpp
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "engine.hpp"
#include <cmath>

using namespace pendulum;
using Catch::Matchers::WithinAbs;

TEST_CASE("Engine<1> runs without crashing") {
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = {1.0, 0.0, 1.0, 0.0};
    State<1> ic;
    ic.theta[0] = 0.1;
    Engine<1> engine(cfg, ic);
    engine.advance(1.0);
    REQUIRE(engine.state().t == Catch::Approx(1.0));
}

TEST_CASE("Engine<1> energy drift < 1e-10 over 10 s") {
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = {1.0, 0.5, 1.0, 0.05};
    State<1> ic;
    ic.theta[0] = 1.0;
    Engine<1> engine(cfg, ic);
    engine.advance(10.0);
    REQUIRE(engine.energy_drift() < 1e-10);
}

TEST_CASE("Engine<1> at rest stays at rest") {
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = {1.0, 0.0, 1.0, 0.0};
    State<1> ic;  // theta=0, theta_dot=0
    Engine<1> engine(cfg, ic);
    engine.advance(5.0);
    REQUIRE_THAT(engine.state().theta[0], WithinAbs(0.0, 1e-14));
    REQUIRE_THAT(engine.state().theta_dot[0], WithinAbs(0.0, 1e-14));
}
```

- [ ] **Step 2: Run — expect FAIL**
- [ ] **Step 3: Create `src/engine.hpp`**

Engine<1> holds SystemConfig, State, CoeffsN1, Dopri5<2>, initial energy E0, step size h, and prev_err. Provides `step()`, `advance(dt)`, `energy()`, `energy_drift()`.

- [ ] **Step 4: Build, run, verify**
- [ ] **Step 5: Commit**

---

### Task 6: Validation Suite

**Files:**
- Create: `tests/test_validation_n1.cpp`
- Modify: `tests/CMakeLists.txt`

**Validation criteria from spec:**
1. Period vs elliptic integral: rel error < 1e-5 for θ₀ ∈ [1e-3, π/2]
2. Energy conservation: |ΔE/E₀| < 1e-5 over 60 s
3. Time-reversal symmetry: forward T, flip θ̇, forward T → matches IC to 1e-5

**Elliptic integral K(k) via AGM:**
```cpp
inline double complete_elliptic_K(double k) {
    double a = 1.0, b = std::sqrt(1.0 - k * k);
    while (std::abs(a - b) > 1e-15) {
        double an = (a + b) / 2.0;
        b = std::sqrt(a * b);
        a = an;
    }
    return M_PI / (2.0 * a);
}
```

Exact period: `T(θ₀) = 4√(β/(γg)) · K(sin(θ₀/2))`

- [ ] **Step 1: Write `tests/test_validation_n1.cpp`** — all three validation tests
- [ ] **Step 2: Run — expect PASS (engine already works)**
- [ ] **Step 3: Commit**
- [ ] **Step 4: Run full test suite, verify all green**

---

## Upgrade Path (Phase 2)

After Phase 1 lands:
- **Task 7:** SymPy codegen for N=2,3 coefficient headers
- **Task 8:** General EOM assembler (M, C, G matrices)
- **Task 9:** Engine<2>, Engine<3>
- **Task 10:** DOP853 integrator (upgrade from DOPRI5)
- **Task 11:** Gauss–Legendre 6 (symplectic mode)
- **Task 12:** Validation for N=2,3
