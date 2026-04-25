#include <catch2/catch_test_macros.hpp>
#include "core/types.hpp"

using namespace pendulum;

TEST_CASE("Valid N=1 config passes validation") {
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = {1.0, 0.5, 1.0, 0.05};
    REQUIRE_NOTHROW(cfg.validate());
}

TEST_CASE("N=0 throws") {
    SystemConfig cfg;
    cfg.N = 0;
    REQUIRE_THROWS_AS(cfg.validate(), std::invalid_argument);
}

TEST_CASE("N=4 throws") {
    SystemConfig cfg;
    cfg.N = 4;
    REQUIRE_THROWS_AS(cfg.validate(), std::invalid_argument);
}

TEST_CASE("Negative rod mass throws") {
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = {1.0, -0.1, 1.0, 0.05};
    REQUIRE_THROWS_AS(cfg.validate(), std::invalid_argument);
}

TEST_CASE("Zero bob mass throws") {
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = {1.0, 0.5, 0.0, 0.05};
    REQUIRE_THROWS_AS(cfg.validate(), std::invalid_argument);
}

TEST_CASE("Bob radius > L/2 throws") {
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = {1.0, 0.5, 1.0, 0.6};
    REQUIRE_THROWS_AS(cfg.validate(), std::invalid_argument);
}

TEST_CASE("Zero rod length throws") {
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = {0.0, 0.5, 1.0, 0.0};
    REQUIRE_THROWS_AS(cfg.validate(), std::invalid_argument);
}

TEST_CASE("State<1> default is zero") {
    State<1> s;
    REQUIRE(s.theta[0] == 0.0);
    REQUIRE(s.theta_dot[0] == 0.0);
    REQUIRE(s.t == 0.0);
}

TEST_CASE("Zero gravity is allowed") {
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = {1.0, 0.0, 1.0, 0.05};
    cfg.g = 0.0;
    REQUIRE_NOTHROW(cfg.validate());
}

TEST_CASE("Negative gravity is allowed") {
    SystemConfig cfg;
    cfg.N = 1;
    cfg.links[0] = {1.0, 0.0, 1.0, 0.05};
    cfg.g = -9.81;
    REQUIRE_NOTHROW(cfg.validate());
}
