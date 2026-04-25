#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "integrators/dopri5.hpp"
#include <cmath>

using namespace pendulum;
using Catch::Matchers::WithinAbs;

TEST_CASE("DOPRI5 solves harmonic oscillator y''=-y over one period") {
    auto rhs = [](double, const Eigen::Vector2d& y) -> Eigen::Vector2d {
        return {y[1], -y[0]};
    };

    Dopri5<2> solver(rhs, 1e-12, 1e-12);
    Eigen::Vector2d y(1.0, 0.0);
    double t = 0.0;
    double h = 0.1;
    double prev_err = 0.0;
    double t_end = 2.0 * M_PI;

    while (t < t_end - 1e-15) {
        double h_try = std::min(h, t_end - t);
        auto res = solver.step(t, y, h_try);
        double h_new = solver.suggest_h(h_try, res.err_norm, prev_err);

        if (res.accepted) {
            y = res.y;
            t = res.t;
            prev_err = res.err_norm;
            h = h_new;
        } else {
            h = h_new;
        }
    }

    REQUIRE_THAT(y[0], WithinAbs(1.0, 1e-8));
    REQUIRE_THAT(y[1], WithinAbs(0.0, 1e-8));
}

TEST_CASE("DOPRI5 rejects step with huge step size") {
    auto rhs = [](double, const Eigen::Vector2d& y) -> Eigen::Vector2d {
        return {y[1], -y[0]};
    };

    Dopri5<2> solver(rhs, 1e-14, 1e-14);
    Eigen::Vector2d y(1.0, 0.0);
    auto res = solver.step(0.0, y, 10.0);
    REQUIRE_FALSE(res.accepted);
}

TEST_CASE("DOPRI5 exponential growth y'=y") {
    auto rhs = [](double, const Eigen::Matrix<double, 1, 1>& y)
        -> Eigen::Matrix<double, 1, 1> {
        return y;
    };

    Dopri5<1> solver(rhs, 1e-12, 1e-12);
    Eigen::Matrix<double, 1, 1> y;
    y << 1.0;
    double t = 0.0, h = 0.1, prev_err = 0.0;
    double t_end = 1.0;

    while (t < t_end - 1e-15) {
        double h_try = std::min(h, t_end - t);
        auto res = solver.step(t, y, h_try);
        double h_new = solver.suggest_h(h_try, res.err_norm, prev_err);
        if (res.accepted) {
            y = res.y;
            t = res.t;
            prev_err = res.err_norm;
            h = h_new;
        } else {
            h = h_new;
        }
    }

    REQUIRE_THAT(y[0], WithinAbs(std::exp(1.0), 1e-8));
}
