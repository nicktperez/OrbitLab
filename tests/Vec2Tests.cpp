#include "orbitlab/Vec2.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

using orbitlab::Vec2;

TEST_CASE("Vec2 supports arithmetic and geometric operations", "[math]") {
    const Vec2 first{3.0, 4.0};
    const Vec2 second{-1.0, 2.0};

    REQUIRE((first + second == Vec2{2.0, 6.0}));
    REQUIRE((first - second == Vec2{4.0, 2.0}));
    REQUIRE((first * 2.0 == Vec2{6.0, 8.0}));
    REQUIRE(first.length() == Catch::Approx(5.0));
    REQUIRE(first.dot(second) == Catch::Approx(5.0));
    REQUIRE(first.normalized().length() == Catch::Approx(1.0));
    REQUIRE((Vec2{}.normalized() == Vec2{}));
}

TEST_CASE("Vec2 rejects division by zero", "[math]") {
    REQUIRE_THROWS_AS((Vec2{1.0, 2.0} / 0.0), std::domain_error);
}
