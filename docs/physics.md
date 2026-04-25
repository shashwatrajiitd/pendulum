# Physics and Numerical Methods

This document describes the mathematical models, equations, and numerical integration schemes used in the pendulum physics engine.

## 1. System Description

The engine simulates a serial chain of N rigid pendulums (N = 1, 2, or 3) swinging in a vertical plane under gravity. Each link consists of:

- A **uniform thin rod** of length L_i and mass m_rod,i
- A **uniform solid sphere (bob)** of radius r_i and mass m_bob,i, rigidly attached at the rod's tip

Rod i+1 pivots at the center of bob i. The first rod pivots at a fixed point (the origin). All motion is confined to the 2D vertical plane.

## 2. Generalized Coordinates

We use **Lagrangian mechanics** with generalized coordinates theta = (theta_1, ..., theta_N), where theta_i is the angle of rod i measured from the downward vertical, positive counterclockwise.

### Cumulative pivot positions

The position of each bob center in Cartesian coordinates:

```
X_i = sum_{j=1}^{i} L_j sin(theta_j)
Y_i = -sum_{j=1}^{i} L_j cos(theta_j)
```

The center of mass of rod i:

```
x_rod,i = X_{i-1} + (L_i / 2) sin(theta_i)
y_rod,i = Y_{i-1} - (L_i / 2) cos(theta_i)
```

## 3. Moments of Inertia

Each body has a moment of inertia about its own center of mass:

| Body | Formula |
|------|---------|
| Rod i (uniform thin rod) | I_rod,i = (1/12) m_rod,i L_i^2 |
| Bob i (uniform solid sphere) | I_bob,i = (2/5) m_bob,i r_i^2 |

## 4. Kinetic and Potential Energy

### Kinetic energy

The total kinetic energy includes translational (center-of-mass) and rotational contributions for each rod and bob:

```
T = sum_{i=1}^{N} [ (1/2) m_rod,i (x_dot_rod,i^2 + y_dot_rod,i^2)
                   + (1/2) I_rod,i theta_dot_i^2
                   + (1/2) m_bob,i (X_dot_i^2 + Y_dot_i^2)
                   + (1/2) I_bob,i theta_dot_i^2 ]
```

### Potential energy

With zero potential at the fixed pivot height:

```
V = g sum_{i=1}^{N} [ m_rod,i y_rod,i + m_bob,i Y_i ]
```

The Lagrangian is L = T - V.

## 5. Equations of Motion (Manipulator Form)

The Euler-Lagrange equations reduce to the standard rigid-body manipulator form:

```
M(theta) theta_ddot + C(theta, theta_dot) theta_dot + G(theta) = 0
```

where:

### Mass matrix M (symmetric, positive-definite)

```
M_ii = beta_i
M_ij = alpha_ij cos(theta_i - theta_j)   for i != j
```

### Coriolis/centrifugal matrix C

```
C_ij = alpha_ij sin(theta_i - theta_j) theta_dot_j
```

### Gravity vector G

```
G_i = gamma_i g sin(theta_i)
```

### Configuration-independent coefficients

These constants are computed once from physical parameters and reused at every timestep:

**beta_i** (self-inertia about pivot i):

```
beta_i = (1/3) m_rod,i L_i^2
       + m_bob,i L_i^2
       + (2/5) m_bob,i r_i^2
       + L_i^2 sum_{k>i} (m_rod,k + m_bob,k)
```

**alpha_ij** (coupling between links i and j, symmetric):

```
alpha_ij = L_i L_j [ m_bob,j + (1/2) m_rod,j + sum_{k>j} (m_rod,k + m_bob,k) ]
```

**gamma_i** (gravity moment arm):

```
gamma_i = (m_rod,i / 2 + m_bob,i) L_i + L_i sum_{k>i} (m_rod,k + m_bob,k)
```

### Forward dynamics step

At each timestep:

1. Build M(theta) and h(theta, theta_dot) = C(theta, theta_dot) theta_dot + G(theta)
2. Solve M theta_ddot = -h via Cholesky decomposition (M is symmetric positive-definite)
3. Pack (theta_dot, theta_ddot) as the state derivative

For N <= 3 the Cholesky solve operates on at most a 3x3 matrix — no heap allocation, all on stack.

## 6. Single Pendulum (N=1) Closed Form

For N=1 the EOM simplifies to:

```
I_eff theta_ddot = -g ((1/2) m_rod + m_bob) L sin(theta)
```

where I_eff = beta_1 = (1/3) m_rod L^2 + m_bob L^2 + (2/5) m_bob r^2.

The exact period for release from angle theta_0 with zero velocity:

```
T(theta_0) = 4 sqrt(I_eff / (gamma_1 g)) K(sin(theta_0 / 2))
```

where K is the complete elliptic integral of the first kind, computed via the arithmetic-geometric mean (AGM) iteration:

```
a_0 = 1,  b_0 = sqrt(1 - k^2)
a_{n+1} = (a_n + b_n) / 2,  b_{n+1} = sqrt(a_n b_n)
K(k) = pi / (2 a_inf)
```

This converges to machine precision (~1e-15) in about 25 iterations and is used as the ground-truth reference in validation tests.

## 7. Energy Conservation

The total energy E = T + V is an invariant of the continuous system (no friction, no driving). For N=1:

```
E = (1/2) beta_1 theta_dot^2 - gamma_1 g cos(theta)
```

For general N:

```
E = (1/2) theta_dot^T M(theta) theta_dot + sum_i (-gamma_i g cos(theta_i))
```

The engine monitors |Delta E / E_0| as the primary precision metric.

## 8. Numerical Integration

### 8.1 Integrator: Dormand-Prince 5(4) (DOPRI5)

The engine uses the Dormand-Prince embedded Runge-Kutta pair of orders 5 and 4. This is a 7-stage explicit method (with FSAL property) that provides:

- A 5th-order accurate solution for time-stepping
- A 4th-order embedded solution for error estimation

The Butcher tableau coefficients are the standard ones from Dormand and Prince (1980):

- b (5th order): [35/384, 0, 500/1113, 125/192, -2187/6784, 11/84, 0]
- b* (4th order): [5179/57600, 0, 7571/16695, 393/640, -92097/339200, 187/2100, 1/40]

### 8.2 Adaptive Step-Size Control

A PI (proportional-integral) controller adjusts the step size based on the local error estimate:

```
h_{n+1} = h_n * clamp(kappa * err_n^{-alpha} * err_{n-1}^{beta}, fac_min, fac_max)
```

with parameters:
- alpha = 0.7 / p, beta = 0.4 / p where p = 5 (method order)
- kappa = 0.9 (safety factor)
- fac_min = 0.2, fac_max = 10.0

The error norm uses mixed absolute/relative tolerances:

```
err = sqrt( (1/dim) sum_k (e_k / (atol + rtol * max(|y_k|, |y_new_k|)))^2 )
```

Tolerances are configurable per engine instance:
- **Native tests**: atol = rtol = 1e-10 (maximum precision, energy drift ~1e-10)
- **WASM browser**: atol = rtol = 1e-8 (optimized for 60 FPS, energy drift ~1e-9)

Steps are rejected when err > 1 and retried with the reduced step size.

### 8.3 Why This Approach

Double and triple pendulums are **chaotic** for moderate energies (positive Lyapunov exponent). Trajectories diverge exponentially even with exact arithmetic. What *can* be preserved is the **energy invariant** E = T + V. The precision target |Delta E / E_0| < 1e-5 over 60 seconds is achievable with DOPRI5(4) at tight tolerances. Native builds use 1e-10 (yielding drift ~1e-10), while the WASM browser build uses 1e-8 (yielding drift ~1e-9) to maintain 60 FPS with up to 3 simultaneous pendulum systems.

### 8.4 Numerical Hygiene

- All computation uses IEEE-754 `double` precision
- Compiled with `-fno-fast-math -ffp-contract=off` to prevent FP reordering
- Trigonometric values sin(theta_i), cos(theta_i) computed once per RHS evaluation
- No heap allocation in the integration hot path (all Eigen matrices are stack-allocated with fixed sizes)
- Cholesky factorization via Eigen's LLT on at most 3x3 matrices

## 9. Validation Strategy

The engine is validated against multiple independent criteria:

| Test | Pass Criterion |
|------|---------------|
| Single pendulum period vs elliptic integral | Trajectory error < 1e-5 for theta_0 in [1e-3, pi/2] |
| Energy conservation (60 s) | \|Delta E / E_0\| < 1e-5 for N=1,2,3 |
| Time-reversal symmetry | Forward T, flip velocities, forward T; matches IC to 1e-5 |
| Zero-gravity test | Angular velocity constant to machine precision (~1e-12) |
| Equilibrium stability | theta=0, theta_dot=0 stays at zero to ~1e-14 |

All 47 tests pass across N=1, N=2, and N=3 configurations.

## 10. References

- Hairer, Norsett, Wanner — *Solving Ordinary Differential Equations I* (DOPRI, step control, PI controller)
- Hairer, Lubich, Wanner — *Geometric Numerical Integration* (symplectic theory)
- Dormand, Prince (1980) — "A family of embedded Runge-Kutta formulae"
- Goldstein, Poole, Safko — *Classical Mechanics* (Lagrangian formulation)
- Strogatz — *Nonlinear Dynamics and Chaos* (Lyapunov exponents, chaos in pendulums)
