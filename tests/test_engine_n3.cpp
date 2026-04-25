#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/catch_approx.hpp>
#include "engine.hpp"
#include <cmath>

using namespace pendulum;
using Catch::Matchers::WithinAbs;

TEST_CASE("Engine<3> runs without crashing") {
    SystemConfig cfg;
    cfg.N = 3;
    cfg.links[0] = {1.0, 0.0, 1.0, 0.0};
    cfg.links[1] = {1.0, 0.0, 1.0, 0.0};
    cfg.links[2] = {1.0, 0.0, 1.0, 0.0};
    State<3> ic;
    ic.theta[0] = 0.3;
    ic.theta[1] = 0.5;
    ic.theta[2] = 0.2;
    Engine<3> engine(cfg, ic);
    engine.advance(1.0);
    REQUIRE(engine.state().t == Catch::Approx(1.0));
}

TEST_CASE("Engine<3> energy drift < 1e-5 over 60s") {
    SystemConfig cfg;
    cfg.N = 3;
    cfg.links[0] = {1.0, 0.0, 1.0, 0.0};
    cfg.links[1] = {1.0, 0.0, 1.0, 0.0};
    cfg.links[2] = {1.0, 0.0, 1.0, 0.0};
    State<3> ic;
    ic.theta[0] = 1.5;
    ic.theta[1] = 1.0;
    ic.theta[2] = 0.5;
    Engine<3> engine(cfg, ic);
    engine.advance(60.0);
    REQUIRE(engine.energy_drift() < 1e-5);
}

TEST_CASE("Engine<3> energy drift < 1e-5 over 60s (mixed params)") {
    SystemConfig cfg;
    cfg.N = 3;
    cfg.links[0] = {1.5, 0.3, 2.0, 0.1};
    cfg.links[1] = {1.0, 0.2, 1.5, 0.08};
    cfg.links[2] = {0.8, 0.1, 1.0, 0.05};
    State<3> ic;
    ic.theta[0] = 1.0;
    ic.theta[1] = 0.8;
    ic.theta[2] = 0.5;
    ic.theta_dot[0] = 0.5;
    Engine<3> engine(cfg, ic);
    engine.advance(60.0);
    REQUIRE(engine.energy_drift() < 1e-5);
}

TEST_CASE("Engine<3> at rest stays at rest") {
    SystemConfig cfg;
    cfg.N = 3;
    cfg.links[0] = {1.0, 0.0, 1.0, 0.0};
    cfg.links[1] = {1.0, 0.0, 1.0, 0.0};
    cfg.links[2] = {1.0, 0.0, 1.0, 0.0};
    State<3> ic;
    Engine<3> engine(cfg, ic);
    engine.advance(5.0);
    for (int i = 0; i < 3; ++i) {
        REQUIRE_THAT(engine.state().theta[i], WithinAbs(0.0, 1e-14));
        REQUIRE_THAT(engine.state().theta_dot[i], WithinAbs(0.0, 1e-14));
    }
}

TEST_CASE("Engine<3> time reversal") {
    SystemConfig cfg;
    cfg.N = 3;
    cfg.links[0] = {1.0, 0.0, 1.0, 0.0};
    cfg.links[1] = {1.0, 0.0, 1.0, 0.0};
    cfg.links[2] = {1.0, 0.0, 1.0, 0.0};
    State<3> ic;
    ic.theta[0] = 0.5;
    ic.theta[1] = 0.3;
    ic.theta[2] = 0.2;
    ic.theta_dot[0] = 0.1;
    ic.theta_dot[1] = -0.2;
    ic.theta_dot[2] = 0.3;

    Engine<3> engine(cfg, ic);
    engine.advance(5.0);

    State<3> mid = engine.state();
    mid.theta_dot = -mid.theta_dot;
    mid.t = 0.0;

    Engine<3> engine2(cfg, mid);
    engine2.advance(5.0);

    for (int i = 0; i < 3; ++i)
        REQUIRE_THAT(engine2.state().theta[i],
                     WithinAbs(ic.theta[i], 1e-5));
}
