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
    REQUIRE_THAT(c.beta, WithinAbs(1.0, 1e-15));
    REQUIRE_THAT(c.gamma, WithinAbs(1.0, 1e-15));
}

TEST_CASE("Coefficients for rod-only pendulum") {
    LinkParams lk{2.0, 3.0, 1e-10, 0.0};
    auto c = compute_coeffs_n1(lk);
    REQUIRE_THAT(c.beta, WithinRel(3.0 * 4.0 / 3.0, 1e-9));
    REQUIRE_THAT(c.gamma, WithinRel(3.0 * 2.0 / 2.0, 1e-9));
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

TEST_CASE("Energy T + V identity") {
    LinkParams lk{1.5, 0.5, 2.0, 0.1};
    auto c = compute_coeffs_n1(lk);
    double g = 9.80665;
    double theta = 1.0, theta_dot = 0.5;
    double T = kinetic_energy_n1(c, theta_dot);
    double V = potential_energy_n1(c, g, theta);
    REQUIRE_THAT(total_energy_n1(c, g, theta, theta_dot),
                 WithinAbs(T + V, 1e-15));
}

TEST_CASE("Energy at rest hanging down is minimum") {
    LinkParams lk{1.0, 0.0, 1.0, 0.0};
    auto c = compute_coeffs_n1(lk);
    double E_bottom = total_energy_n1(c, 9.80665, 0.0, 0.0);
    double E_side = total_energy_n1(c, 9.80665, M_PI / 2.0, 0.0);
    REQUIRE(E_bottom < E_side);
}

TEST_CASE("Coefficients with nonzero bob radius") {
    LinkParams lk{1.0, 0.0, 1.0, 0.1};
    auto c = compute_coeffs_n1(lk);
    double expected_beta = 1.0 + 0.4 * 0.01;
    REQUIRE_THAT(c.beta, WithinAbs(expected_beta, 1e-15));
    REQUIRE_THAT(c.gamma, WithinAbs(1.0, 1e-15));
}
