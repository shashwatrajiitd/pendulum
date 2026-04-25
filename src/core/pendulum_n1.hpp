#pragma once
#include "types.hpp"
#include <cmath>

namespace pendulum {

struct CoeffsN1 {
    double beta;   // effective moment of inertia
    double gamma;  // gravity moment arm coefficient
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

inline double total_energy_n1(const CoeffsN1& c, double g,
                               double theta, double theta_dot) {
    return kinetic_energy_n1(c, theta_dot) + potential_energy_n1(c, g, theta);
}

} // namespace pendulum
