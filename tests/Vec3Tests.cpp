#include "orbitlab/Vec3.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using orbitlab::Vec3;

TEST_CASE("Vec3 supports spatial vector operations", "[math][3d]") {
    const Vec3 first{1.0, 2.0, 3.0};
    const Vec3 second{-2.0, 0.5, 4.0};

    REQUIRE((first + second == Vec3{-1.0, 2.5, 7.0}));
    REQUIRE(first.dot(second) == Catch::Approx(11.0));
    REQUIRE((Vec3{1.0, 0.0, 0.0}.cross({0.0, 1.0, 0.0}) ==
             Vec3{0.0, 0.0, 1.0}));
    REQUIRE(first.normalized().length() == Catch::Approx(1.0));
    REQUIRE(Vec3{}.normalized() == Vec3{});
}

TEST_CASE("Vec3 rejects division by zero", "[math][3d]") {
    REQUIRE_THROWS_AS((Vec3{1.0, 2.0, 3.0} / 0.0), std::domain_error);
}
