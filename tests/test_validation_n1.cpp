#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/catch_approx.hpp>
#include "engine.hpp"
#include <cmath>
#include <vector>

using namespace pendulum;
using Catch::Matchers::WithinAbs;

// Complete elliptic integral K(k) via arithmetic-geometric mean
static double complete_elliptic_K(double k) {
    if (k >= 1.0)
        return std::numeric_limits<double>::infinity();
    double a = 1.0, b = std::sqrt(1.0 - k * k);
    for (int i = 0; i < 100 && std::abs(a - b) > 1e-15; ++i) {
        double an = (a + b) / 2.0;
        b = std::sqrt(a * b);
        a = an;
    }
    return M_PI / (2.0 * a);
}

static double exact_period_n1(const LinkParams& lk, double g, double theta0) {
    auto c = compute_coeffs_n1(lk);
    double k = std::sin(theta0 / 2.0);
    return 4.0 * std::sqrt(c.beta / (c.gamma * g)) * complete_elliptic_K(k);
}

// --- Validation 1: Period vs elliptic integral ---

TEST_CASE("Period matches elliptic integral for small angle (1e-3 rad)") {
    LinkParams lk{1.0, 0.0, 1.0, 0.0};
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = lk;

    double theta0 = 1e-3;
    double T_exact = exact_period_n1(lk, cfg.g, theta0);

    State<1> ic;
    ic.theta[0] = theta0;
    Engine<1> engine(cfg, ic);
    engine.advance(T_exact);

    double theta_err = std::abs(engine.state().theta[0] - theta0);
    REQUIRE(theta_err < 1e-5);
}

TEST_CASE("Period matches elliptic integral for theta0 = 0.5 rad") {
    LinkParams lk{1.0, 0.0, 1.0, 0.0};
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = lk;

    double theta0 = 0.5;
    double T_exact = exact_period_n1(lk, cfg.g, theta0);

    State<1> ic;
    ic.theta[0] = theta0;
    Engine<1> engine(cfg, ic);
    engine.advance(T_exact);

    double theta_err = std::abs(engine.state().theta[0] - theta0);
    REQUIRE(theta_err < 1e-5);
}

TEST_CASE("Period matches elliptic integral for theta0 = pi/4") {
    LinkParams lk{1.0, 0.0, 1.0, 0.0};
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = lk;

    double theta0 = M_PI / 4.0;
    double T_exact = exact_period_n1(lk, cfg.g, theta0);

    State<1> ic;
    ic.theta[0] = theta0;
    Engine<1> engine(cfg, ic);
    engine.advance(T_exact);

    double theta_err = std::abs(engine.state().theta[0] - theta0);
    REQUIRE(theta_err < 1e-5);
}

TEST_CASE("Period matches elliptic integral for theta0 = pi/2") {
    LinkParams lk{1.0, 0.0, 1.0, 0.0};
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = lk;

    double theta0 = M_PI / 2.0;
    double T_exact = exact_period_n1(lk, cfg.g, theta0);

    State<1> ic;
    ic.theta[0] = theta0;
    Engine<1> engine(cfg, ic);
    engine.advance(T_exact);

    double theta_err = std::abs(engine.state().theta[0] - theta0);
    REQUIRE(theta_err < 1e-5);
}

TEST_CASE("Period with rod mass and bob radius") {
    LinkParams lk{1.5, 0.5, 2.0, 0.1};
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = lk;

    double theta0 = 0.8;
    double T_exact = exact_period_n1(lk, cfg.g, theta0);

    State<1> ic;
    ic.theta[0] = theta0;
    Engine<1> engine(cfg, ic);
    engine.advance(T_exact);

    double theta_err = std::abs(engine.state().theta[0] - theta0);
    REQUIRE(theta_err < 1e-5);
}

// --- Validation 2: Energy conservation over 60 s ---

TEST_CASE("Energy conservation |dE/E0| < 1e-5 over 60s (point mass, theta0=1)") {
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = {1.0, 0.0, 1.0, 0.0};
    State<1> ic;
    ic.theta[0] = 1.0;
    Engine<1> engine(cfg, ic);
    engine.advance(60.0);
    REQUIRE(engine.energy_drift() < 1e-5);
}

TEST_CASE("Energy conservation |dE/E0| < 1e-5 over 60s (rod+bob, theta0=2)") {
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = {1.5, 0.5, 2.0, 0.1};
    State<1> ic;
    ic.theta[0] = 2.0;
    Engine<1> engine(cfg, ic);
    engine.advance(60.0);
    REQUIRE(engine.energy_drift() < 1e-5);
}

TEST_CASE("Energy conservation |dE/E0| < 1e-5 over 60s (near-inverted, theta0=3)") {
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = {1.0, 0.0, 1.0, 0.0};
    State<1> ic;
    ic.theta[0] = 3.0;
    Engine<1> engine(cfg, ic);
    engine.advance(60.0);
    REQUIRE(engine.energy_drift() < 1e-5);
}

// --- Validation 3: Time-reversal symmetry ---

TEST_CASE("Time reversal: forward T, flip v, forward T -> returns to IC") {
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = {1.0, 0.5, 1.0, 0.1};
    State<1> ic;
    ic.theta[0] = 1.2;
    ic.theta_dot[0] = 0.5;

    Engine<1> engine(cfg, ic);
    engine.advance(5.0);

    State<1> mid = engine.state();
    mid.theta_dot[0] = -mid.theta_dot[0];
    mid.t = 0.0;

    Engine<1> engine2(cfg, mid);
    engine2.advance(5.0);

    REQUIRE_THAT(engine2.state().theta[0],
                 WithinAbs(ic.theta[0], 1e-5));
    REQUIRE_THAT(engine2.state().theta_dot[0],
                 WithinAbs(-ic.theta_dot[0], 1e-5));
}

TEST_CASE("Time reversal with zero initial velocity") {
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = {1.0, 0.0, 1.0, 0.0};
    State<1> ic;
    ic.theta[0] = 0.8;

    Engine<1> engine(cfg, ic);
    engine.advance(10.0);

    State<1> mid = engine.state();
    mid.theta_dot[0] = -mid.theta_dot[0];
    mid.t = 0.0;

    Engine<1> engine2(cfg, mid);
    engine2.advance(10.0);

    REQUIRE_THAT(engine2.state().theta[0],
                 WithinAbs(ic.theta[0], 1e-5));
    REQUIRE_THAT(engine2.state().theta_dot[0],
                 WithinAbs(0.0, 1e-5));
}

// --- Validation 4: Zero-gravity angular momentum conservation ---

TEST_CASE("Zero gravity: angular velocity stays constant") {
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = {1.0, 0.0, 1.0, 0.0};
    cfg.g = 0.0;
    State<1> ic;
    ic.theta[0] = 0.5;
    ic.theta_dot[0] = 2.0;

    Engine<1> engine(cfg, ic);
    engine.advance(10.0);

    REQUIRE_THAT(engine.state().theta_dot[0],
                 WithinAbs(2.0, 1e-12));
    double expected_theta = 0.5 + 2.0 * 10.0;
    REQUIRE_THAT(engine.state().theta[0],
                 WithinAbs(expected_theta, 1e-8));
}
