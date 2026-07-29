#include "orbitlab/GravitySolver.hpp"

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <random>

using namespace orbitlab;

TEST_CASE("Barnes-Hut closely approximates the direct solver", "[physics][barnes-hut]") {
    std::mt19937 generator(7);
    std::uniform_real_distribution<double> position(-2.0, 2.0);
    std::uniform_real_distribution<double> mass(0.1, 2.0);
    std::vector<Body> bodies;
    for (std::uint64_t index = 1; index <= 80; ++index) {
        bodies.push_back(
             {index, "Body", mass(generator), 0.001, {},
             {position(generator), position(generator), position(generator)}, {}});
    }

    const DirectGravitySolver direct;
    const BarnesHutGravitySolver barnesHut(0.45);
    const auto expected = direct.accelerations(bodies, 1.0, 0.01);
    const auto approximate = barnesHut.accelerations(bodies, 1.0, 0.01);

    double errorSquared = 0.0;
    double referenceSquared = 0.0;
    for (std::size_t index = 0; index < bodies.size(); ++index) {
        errorSquared += (approximate[index] - expected[index]).lengthSquared();
        referenceSquared += expected[index].lengthSquared();
    }
    const double relativeRmsError = std::sqrt(errorSquared / referenceSquared);
    REQUIRE(relativeRmsError < 0.035);
}

TEST_CASE("Barnes-Hut handles coincident positions safely", "[physics][barnes-hut]") {
    const std::vector<Body> bodies{
        {1, "A", 1.0, 0.01, {}, {}, {}},
        {2, "B", 2.0, 0.01, {}, {}, {}},
    };
    const BarnesHutGravitySolver solver;
    const auto acceleration = solver.accelerations(bodies, 1.0, 0.01);
    REQUIRE(acceleration[0].isFinite());
    REQUIRE(acceleration[1].isFinite());
}
