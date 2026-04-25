#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <Eigen/Dense>
#include <cmath>

TEST_CASE("Eigen smoke test") {
    Eigen::Vector2d v(1.0, 2.0);
    REQUIRE(v.norm() == Catch::Approx(std::sqrt(5.0)));
}
