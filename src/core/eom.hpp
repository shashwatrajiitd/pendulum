#pragma once
#include "types.hpp"
#include <cmath>
#include <Eigen/Dense>

namespace pendulum {

// Configuration-independent coefficients computed once from physical params.
//
// For an N-link serial pendulum chain:
//   β_i  = diagonal of mass matrix (self-inertia of link i about its pivot)
//   α_ij = off-diagonal coupling (symmetric, i≠j)
//   γ_i  = gravity coefficient
//
// The EOM in manipulator form:
//   M(θ)θ̈ + C(θ,θ̇)θ̇ + G(θ) = 0
// where:
//   M_ii = β_i
//   M_ij = α_ij cos(θ_i - θ_j)   (i ≠ j)
//   C_ij = α_ij sin(θ_i - θ_j) θ̇_j   (Coriolis/centrifugal)
//   G_i  = γ_i g sin(θ_i)

template <int N>
struct Coefficients {
    Eigen::Matrix<double, N, 1> beta;
    Eigen::Matrix<double, N, N> alpha;
    Eigen::Matrix<double, N, 1> gamma_coeff;
};

template <int N>
Coefficients<N> compute_coefficients(const SystemConfig& cfg) {
    Coefficients<N> c;
    c.alpha.setZero();

    for (int i = 0; i < N; ++i) {
        const auto& li = cfg.links[i];
        double Li = li.L, mri = li.m_rod, mbi = li.m_bob, ri = li.r_bob;

        // β_i: effective inertia of link i
        // = (1/3)m_r,i L_i² + (2/5)m_b,i r_i²   (own rotation)
        //   + m_b,i L_i²                           (bob at end of rod)
        //   + L_i² Σ_{k>i}(m_r,k + m_b,k)         (downstream load)
        double downstream_mass = 0.0;
        for (int k = i + 1; k < N; ++k)
            downstream_mass += cfg.links[k].m_rod + cfg.links[k].m_bob;

        c.beta[i] = mri * Li * Li / 3.0
                     + mbi * Li * Li
                     + 0.4 * mbi * ri * ri
                     + Li * Li * downstream_mass;

        // γ_i: gravity moment arm
        // = (m_r,i/2 + m_b,i) L_i + L_i Σ_{k>i}(m_r,k + m_b,k)
        c.gamma_coeff[i] = (0.5 * mri + mbi) * Li
                            + Li * downstream_mass;

        // α_ij: coupling between links i and j (symmetric)
        for (int j = i + 1; j < N; ++j) {
            const auto& lj = cfg.links[j];
            double Lj = lj.L, mrj = lj.m_rod, mbj = lj.m_bob;

            // α_ij = L_i L_j [m_b,j + m_r,j/2 + Σ_{k>j}(m_r,k + m_b,k)]
            double downstream_j = 0.0;
            for (int k = j + 1; k < N; ++k)
                downstream_j += cfg.links[k].m_rod + cfg.links[k].m_bob;

            c.alpha(i, j) = Li * Lj * (mbj + 0.5 * mrj + downstream_j);
            c.alpha(j, i) = c.alpha(i, j);
        }
    }
    return c;
}

template <int N>
using VecN = Eigen::Matrix<double, N, 1>;

template <int N>
using MatN = Eigen::Matrix<double, N, N>;

template <int N>
VecN<N> compute_acceleration(const Coefficients<N>& c, double g,
                              const VecN<N>& theta, const VecN<N>& theta_dot) {
    // Build mass matrix M
    MatN<N> M;
    for (int i = 0; i < N; ++i) {
        M(i, i) = c.beta[i];
        for (int j = i + 1; j < N; ++j) {
            double cos_diff = std::cos(theta[i] - theta[j]);
            M(i, j) = c.alpha(i, j) * cos_diff;
            M(j, i) = M(i, j);
        }
    }

    // Build h = C(θ,θ̇)θ̇ + G(θ)
    VecN<N> h;
    for (int i = 0; i < N; ++i) {
        double coriolis = 0.0;
        for (int j = 0; j < N; ++j) {
            if (j != i) {
                double sin_diff = std::sin(theta[i] - theta[j]);
                coriolis += c.alpha(i, j) * sin_diff
                            * theta_dot[j] * theta_dot[j];
            }
        }
        h[i] = coriolis + c.gamma_coeff[i] * g * std::sin(theta[i]);
    }

    // Solve M θ̈ = -h via Cholesky (M is SPD)
    return M.llt().solve(-h);
}

// Energy computation for general N-pendulum
template <int N>
double compute_kinetic_energy(const Coefficients<N>& c,
                               const VecN<N>& theta,
                               const VecN<N>& theta_dot) {
    // T = (1/2) θ̇ᵀ M(θ) θ̇
    MatN<N> M;
    for (int i = 0; i < N; ++i) {
        M(i, i) = c.beta[i];
        for (int j = i + 1; j < N; ++j) {
            double cos_diff = std::cos(theta[i] - theta[j]);
            M(i, j) = c.alpha(i, j) * cos_diff;
            M(j, i) = M(i, j);
        }
    }
    return 0.5 * (theta_dot.transpose() * M * theta_dot)(0, 0);
}

template <int N>
double compute_potential_energy(const Coefficients<N>& c, double g,
                                 const VecN<N>& theta) {
    double V = 0.0;
    for (int i = 0; i < N; ++i)
        V += -c.gamma_coeff[i] * g * std::cos(theta[i]);
    return V;
}

template <int N>
double compute_total_energy(const Coefficients<N>& c, double g,
                             const VecN<N>& theta,
                             const VecN<N>& theta_dot) {
    return compute_kinetic_energy(c, theta, theta_dot)
           + compute_potential_energy(c, g, theta);
}

} // namespace pendulum
