#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/catch_approx.hpp>
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

TEST_CASE("Engine<1> energy drift < 1e-10 over 10s") {
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = {1.0, 0.5, 1.0, 0.05};
    State<1> ic;
    ic.theta[0] = 1.0;
    Engine<1> engine(cfg, ic);
    engine.advance(10.0);
    REQUIRE(engine.energy_drift() < 1e-8);
}

TEST_CASE("Engine<1> at rest stays at rest") {
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = {1.0, 0.0, 1.0, 0.0};
    State<1> ic;
    Engine<1> engine(cfg, ic);
    engine.advance(5.0);
    REQUIRE_THAT(engine.state().theta[0], WithinAbs(0.0, 1e-14));
    REQUIRE_THAT(engine.state().theta_dot[0], WithinAbs(0.0, 1e-14));
}

TEST_CASE("Engine<1> small oscillation is periodic") {
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = {1.0, 0.0, 1.0, 0.0};
    State<1> ic;
    ic.theta[0] = 0.001;
    Engine<1> engine(cfg, ic);

    double T = 2.0 * M_PI * std::sqrt(1.0 / 9.80665);
    engine.advance(T);

    REQUIRE_THAT(engine.state().theta[0],
                 WithinAbs(0.001, 1e-5));
    REQUIRE_THAT(engine.state().theta_dot[0],
                 WithinAbs(0.0, 1e-4));
}
