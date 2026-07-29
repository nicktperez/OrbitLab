#include "orbitlab/GravitySolver.hpp"
#include "orbitlab/Simulation.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace orbitlab;

TEST_CASE("Direct solver computes symmetric gravitational acceleration", "[physics]") {
    const std::vector<Body> bodies{
        {1, "Left", 2.0, 0.01, {}, {-1.0, 0.0}, {}},
        {2, "Right", 3.0, 0.01, {}, {1.0, 0.0}, {}},
    };
    const DirectGravitySolver solver;
    const auto acceleration = solver.accelerations(bodies, 1.0, 0.0);

    REQUIRE(acceleration[0].x == Catch::Approx(0.75));
    REQUIRE(acceleration[0].y == Catch::Approx(0.0));
    REQUIRE(acceleration[1].x == Catch::Approx(-0.5));
    REQUIRE(acceleration[1].y == Catch::Approx(0.0));
    REQUIRE((acceleration[0] * bodies[0].mass +
             acceleration[1] * bodies[1].mass)
                .length() == Catch::Approx(0.0).margin(1.0e-12));
}

TEST_CASE("SoA solver strategies agree with pairwise direct gravity", "[physics][performance]") {
    const std::vector<Body> bodies{
        {1, "A", 2.0, 0.01, {}, {-1.0, 0.25, 0.4}, {}},
        {2, "B", 3.0, 0.01, {}, {1.0, -0.5, -0.7}, {}},
        {3, "C", 0.4, 0.01, {}, {0.2, 1.25, 1.1}, {}},
        {4, "D", 1.1, 0.01, {}, {-0.7, -1.4, 0.2}, {}},
    };
    const DirectGravitySolver direct;
    const SoADirectGravitySolver soa;
    const ThreadedSoAGravitySolver threaded{2};
    const auto reference = direct.accelerations(bodies, 1.0, 0.002);
    const auto soaResult = soa.accelerations(bodies, 1.0, 0.002);
    const auto threadedResult = threaded.accelerations(bodies, 1.0, 0.002);

    for (std::size_t index = 0; index < bodies.size(); ++index) {
        REQUIRE(soaResult[index].x == Catch::Approx(reference[index].x).epsilon(1.0e-12));
        REQUIRE(soaResult[index].y == Catch::Approx(reference[index].y).epsilon(1.0e-12));
        REQUIRE(soaResult[index].z == Catch::Approx(reference[index].z).epsilon(1.0e-12));
        REQUIRE(threadedResult[index].x == Catch::Approx(soaResult[index].x));
        REQUIRE(threadedResult[index].y == Catch::Approx(soaResult[index].y));
        REQUIRE(threadedResult[index].z == Catch::Approx(soaResult[index].z));
    }
}

TEST_CASE("Direct gravity accelerates bodies across the Z axis", "[physics][3d]") {
    const std::vector<Body> bodies{
        {1, "Below", 2.0, 0.01, {}, {0.0, 0.0, -1.0}, {}},
        {2, "Above", 3.0, 0.01, {}, {0.0, 0.0, 1.0}, {}},
    };
    const DirectGravitySolver solver;
    const auto acceleration = solver.accelerations(bodies, 1.0, 0.0);

    REQUIRE(acceleration[0].x == Catch::Approx(0.0));
    REQUIRE(acceleration[0].y == Catch::Approx(0.0));
    REQUIRE(acceleration[0].z == Catch::Approx(0.75));
    REQUIRE(acceleration[1].z == Catch::Approx(-0.5));
}

TEST_CASE("Simulation can install and clear an external gravity solver", "[architecture]") {
    class ZeroGravitySolver final : public GravitySolver {
    public:
        [[nodiscard]] std::vector<Vec3> accelerations(
            const std::span<const Body> bodies,
            double,
            double) const override {
            return std::vector<Vec3>(bodies.size());
        }
    };

    Simulation simulation;
    simulation.settings().collisionMode = CollisionMode::None;
    simulation.addBody({0, "Left", 1.0, 0.1, {}, {-1.0, 0.0, 0.0}, {}});
    simulation.addBody({0, "Right", 1.0, 0.1, {}, {1.0, 0.0, 0.0}, {}});
    simulation.setGravitySolverOverride(std::make_unique<ZeroGravitySolver>());
    REQUIRE(simulation.hasGravitySolverOverride());
    simulation.step(0.1);
    CHECK(simulation.bodies().front().position.x == Catch::Approx(-1.0));

    simulation.clearGravitySolverOverride();
    CHECK_FALSE(simulation.hasGravitySolverOverride());
    simulation.step(0.1);
    CHECK(simulation.bodies().front().position.x > -1.0);
}
