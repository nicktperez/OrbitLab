#include "orbitlab/Presets.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <array>
#include <cmath>
#include <numbers>

using namespace orbitlab;

TEST_CASE("Single body follows inertial motion", "[integration]") {
    Simulation simulation;
    simulation.settings().collisionMode = CollisionMode::None;
    simulation.addBody({0, "Probe", 1.0, 0.01, {}, {1.0, 2.0}, {3.0, -4.0}});

    simulation.step(0.25);

    REQUIRE(simulation.bodies().front().position.x == Catch::Approx(1.75));
    REQUIRE(simulation.bodies().front().position.y == Catch::Approx(1.0));
    REQUIRE((simulation.bodies().front().velocity == Vec3{3.0, -4.0, 0.0}));
}

TEST_CASE("Sun Earth preset remains orbitally stable for ten periods", "[integration][orbit]") {
    Simulation simulation;
    loadPreset(simulation, Preset::SunEarth);
    simulation.settings().collisionMode = CollisionMode::None;
    simulation.settings().softeningLength = 0.0;

    constexpr double step = 0.001;
    constexpr double tenPeriods = 20.0 * std::numbers::pi;
    const auto steps = static_cast<int>(tenPeriods / step);
    for (int index = 0; index < steps; ++index) {
        simulation.step(step);
    }

    const auto separation =
        (simulation.bodies()[1].position - simulation.bodies()[0].position).length();
    REQUIRE(separation == Catch::Approx(1.0).margin(2.0e-3));
}

TEST_CASE("Invalid numeric simulation input is rejected", "[integration][validation]") {
    Simulation simulation;
    REQUIRE_THROWS_AS(simulation.step(0.0), std::invalid_argument);
    REQUIRE_THROWS_AS(
        simulation.addBody({0, "Invalid", -1.0, 0.1, {}, {}, {}}),
        std::invalid_argument);
}

TEST_CASE("All integrators preserve a bounded Sun Earth orbit", "[integration][integrators]") {
    constexpr std::array integrators{
        IntegratorType::VelocityVerlet,
        IntegratorType::SymplecticEuler,
        IntegratorType::RungeKutta4,
        IntegratorType::Yoshida4,
    };
    for (const auto integrator : integrators) {
        Simulation simulation;
        loadPreset(simulation, Preset::SunEarth);
        simulation.settings().integratorType = integrator;
        simulation.settings().collisionMode = CollisionMode::None;
        simulation.settings().softeningLength = 0.0;
        constexpr double step = 0.004;
        const auto steps =
            static_cast<int>((2.0 * std::numbers::pi) / step);
        for (int index = 0; index < steps; ++index) {
            simulation.step(step);
        }
        const double separation =
            (simulation.bodies()[1].position - simulation.bodies()[0].position).length();
        INFO("Integrator enum value: " << static_cast<int>(integrator));
        REQUIRE(separation == Catch::Approx(1.0).margin(0.015));
    }
}
