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
                throw std::invalid_argument(
                    "Link " + std::to_string(i) + ": L must be > 0");
            if (lk.m_bob <= 0.0)
                throw std::invalid_argument(
                    "Link " + std::to_string(i) + ": m_bob must be > 0");
            if (lk.m_rod < 0.0)
                throw std::invalid_argument(
                    "Link " + std::to_string(i) + ": m_rod must be >= 0");
            if (lk.r_bob < 0.0 || lk.r_bob > lk.L / 2.0)
                throw std::invalid_argument(
                    "Link " + std::to_string(i) + ": r_bob must be in [0, L/2]");
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
