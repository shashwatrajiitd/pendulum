#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/catch_approx.hpp>
#include "engine.hpp"
#include <cmath>

using namespace pendulum;
using Catch::Matchers::WithinAbs;

TEST_CASE("Engine<2> runs without crashing") {
    SystemConfig cfg;
    cfg.N = 2;
    cfg.links[0] = {1.0, 0.0, 1.0, 0.0};
    cfg.links[1] = {1.0, 0.0, 1.0, 0.0};
    State<2> ic;
    ic.theta[0] = 0.5;
    ic.theta[1] = 0.5;
    Engine<2> engine(cfg, ic);
    engine.advance(1.0);
    REQUIRE(engine.state().t == Catch::Approx(1.0));
}

TEST_CASE("Engine<2> energy drift < 1e-5 over 60s (chaotic regime)") {
    SystemConfig cfg;
    cfg.N = 2;
    cfg.links[0] = {1.0, 0.0, 1.0, 0.0};
    cfg.links[1] = {1.0, 0.0, 1.0, 0.0};
    State<2> ic;
    ic.theta[0] = 2.0;
    ic.theta[1] = 2.0;
    Engine<2> engine(cfg, ic);
    engine.advance(60.0);
    REQUIRE(engine.energy_drift() < 1e-5);
}

TEST_CASE("Engine<2> energy drift < 1e-5 over 60s (moderate energy)") {
    SystemConfig cfg;
    cfg.N = 2;
    cfg.links[0] = {1.5, 0.5, 2.0, 0.1};
    cfg.links[1] = {1.0, 0.3, 1.5, 0.08};
    State<2> ic;
    ic.theta[0] = 1.0;
    ic.theta[1] = 0.5;
    ic.theta_dot[0] = 0.3;
    Engine<2> engine(cfg, ic);
    engine.advance(60.0);
    REQUIRE(engine.energy_drift() < 1e-5);
}

TEST_CASE("Engine<2> at rest stays at rest") {
    SystemConfig cfg;
    cfg.N = 2;
    cfg.links[0] = {1.0, 0.0, 1.0, 0.0};
    cfg.links[1] = {1.0, 0.0, 1.0, 0.0};
    State<2> ic;
    Engine<2> engine(cfg, ic);
    engine.advance(5.0);
    REQUIRE_THAT(engine.state().theta[0], WithinAbs(0.0, 1e-14));
    REQUIRE_THAT(engine.state().theta[1], WithinAbs(0.0, 1e-14));
}

TEST_CASE("Engine<2> time reversal") {
    SystemConfig cfg;
    cfg.N = 2;
    cfg.links[0] = {1.0, 0.0, 1.0, 0.0};
    cfg.links[1] = {1.0, 0.0, 1.0, 0.0};
    State<2> ic;
    ic.theta[0] = 0.5;
    ic.theta[1] = 0.3;
    ic.theta_dot[0] = 0.1;
    ic.theta_dot[1] = -0.2;

    Engine<2> engine(cfg, ic);
    engine.advance(5.0);

    State<2> mid = engine.state();
    mid.theta_dot[0] = -mid.theta_dot[0];
    mid.theta_dot[1] = -mid.theta_dot[1];
    mid.t = 0.0;

    Engine<2> engine2(cfg, mid);
    engine2.advance(5.0);

    REQUIRE_THAT(engine2.state().theta[0], WithinAbs(ic.theta[0], 1e-5));
    REQUIRE_THAT(engine2.state().theta[1], WithinAbs(ic.theta[1], 1e-5));
}

TEST_CASE("Engine<2> zero gravity: total angular momentum conserved") {
    SystemConfig cfg;
    cfg.N = 2;
    cfg.links[0] = {1.0, 0.0, 1.0, 0.0};
    cfg.links[1] = {1.0, 0.0, 1.0, 0.0};
    cfg.g = 0.0;
    State<2> ic;
    ic.theta[0] = 0.5;
    ic.theta[1] = 1.0;
    ic.theta_dot[0] = 1.0;
    ic.theta_dot[1] = -0.5;

    Engine<2> engine(cfg, ic);
    double E0 = engine.energy();
    engine.advance(10.0);
    double E_final = engine.energy();
    REQUIRE(std::abs(E_final - E0) < 1e-10);
}
