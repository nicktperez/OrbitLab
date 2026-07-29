#include "orbitlab/Simulation.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace orbitlab;

TEST_CASE("Overlapping bodies merge while conserving mass and momentum", "[collision]") {
    Simulation simulation;
    simulation.settings().softeningLength = 0.01;
    simulation.addBody({0, "Heavy", 3.0, 0.2, {1, 0, 0, 1}, {-0.05, 0.0}, {1.0, 0.0}});
    simulation.addBody({0, "Light", 1.0, 0.2, {0, 0, 1, 1}, {0.05, 0.0}, {-1.0, 0.0}});

    simulation.step(0.001);

    REQUIRE(simulation.bodies().size() == 1);
    const auto& merged = simulation.bodies().front();
    REQUIRE(merged.mass == Catch::Approx(4.0));
    REQUIRE(merged.velocity.x == Catch::Approx(0.5).margin(1.0e-10));
    REQUIRE(merged.radius == Catch::Approx(std::cbrt(0.016)));
    REQUIRE(merged.name == "Heavy");
    REQUIRE(simulation.collisionEvents().size() == 1);
    REQUIRE(simulation.collisionEvents().front().firstBody == "Heavy");
    REQUIRE(simulation.collisionEvents().front().secondBody == "Light");
    REQUIRE(simulation.collisionEvents().front().combinedMass == Catch::Approx(4.0));
    REQUIRE(simulation.collisionEvents().front().momentumError ==
            Catch::Approx(0.0).margin(1.0e-12));
}

TEST_CASE("Collision handling can be disabled", "[collision]") {
    Simulation simulation;
    simulation.settings().collisionMode = CollisionMode::None;
    simulation.addBody({0, "A", 1.0, 0.2, {}, {-0.05, 0.0}, {}});
    simulation.addBody({0, "B", 1.0, 0.2, {}, {0.05, 0.0}, {}});
    simulation.step(0.001);
    REQUIRE(simulation.bodies().size() == 2);
}

TEST_CASE("Elastic collisions conserve mass and reverse approaching velocities", "[collision]") {
    Simulation simulation;
    simulation.settings().collisionMode = CollisionMode::Elastic;
    simulation.settings().softeningLength = 0.1;
    simulation.addBody({0, "A", 1.0, 0.2, {}, {-0.1, 0.0}, {1.0, 0.0}});
    simulation.addBody({0, "B", 1.0, 0.2, {}, {0.1, 0.0}, {-1.0, 0.0}});
    simulation.step(0.0001);

    REQUIRE(simulation.bodies().size() == 2);
    REQUIRE(simulation.bodies()[0].velocity.x < 0.0);
    REQUIRE(simulation.bodies()[1].velocity.x > 0.0);
    REQUIRE(simulation.collisionEvents().size() == 1);
    REQUIRE(simulation.collisionEvents().front().eventType == "elastic");
    REQUIRE(simulation.collisionEvents().front().momentumError ==
            Catch::Approx(0.0).margin(1.0e-12));
}

TEST_CASE("Absorption preserves the larger body's identity", "[collision]") {
    Simulation simulation;
    simulation.settings().collisionMode = CollisionMode::Absorb;
    simulation.addBody(
        {0, "Large", 5.0, 0.2, {1, 0, 0, 1}, {-0.05, 0.0}, {}});
    simulation.addBody(
        {0, "Small", 1.0, 0.2, {0, 0, 1, 1}, {0.05, 0.0}, {}});
    simulation.step(0.0001);
    REQUIRE(simulation.bodies().size() == 1);
    REQUIRE(simulation.bodies().front().name == "Large");
    REQUIRE(simulation.bodies().front().color.r == 1.0F);
    REQUIRE(simulation.collisionEvents().front().eventType == "absorb");
}

TEST_CASE("High speed collisions fragment while conserving mass and momentum", "[collision]") {
    Simulation simulation;
    simulation.settings().collisionMode = CollisionMode::Fragment;
    simulation.settings().fragmentationSpeedThreshold = 0.5;
    simulation.settings().softeningLength = 0.1;
    simulation.addBody({0, "A", 2.0, 0.2, {}, {-0.1, 0.0}, {1.0, 0.0}});
    simulation.addBody({0, "B", 2.0, 0.2, {}, {0.1, 0.0}, {-1.0, 0.0}});
    simulation.step(0.0001);

    REQUIRE(simulation.bodies().size() == 6);
    double totalMass = 0.0;
    Vec3 momentum{};
    for (const auto& body : simulation.bodies()) {
        totalMass += body.mass;
        momentum += body.velocity * body.mass;
    }
    REQUIRE(totalMass == Catch::Approx(4.0));
    REQUIRE(momentum.length() == Catch::Approx(0.0).margin(1.0e-10));
    REQUIRE(simulation.collisionEvents().front().eventType == "fragment");
}
