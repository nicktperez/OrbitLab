#include "orbitlab/Presets.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <numbers>

using namespace orbitlab;

TEST_CASE("System diagnostics report conserved quantities", "[diagnostics]") {
    Simulation simulation;
    loadPreset(simulation, Preset::BinaryStars);
    simulation.settings().softeningLength = 0.0;
    const auto diagnostics = simulation.diagnostics();

    REQUIRE(diagnostics.kineticEnergy == Catch::Approx(0.5));
    REQUIRE(diagnostics.potentialEnergy == Catch::Approx(-1.0));
    REQUIRE(diagnostics.totalEnergy == Catch::Approx(-0.5));
    REQUIRE(diagnostics.linearMomentum.length() == Catch::Approx(0.0));
    REQUIRE(diagnostics.angularMomentum == Catch::Approx(std::sqrt(0.5)));
}

TEST_CASE("Orbital elements identify a near-circular Earth orbit", "[diagnostics][orbit]") {
    Simulation simulation;
    loadPreset(simulation, Preset::SunEarth);
    const auto elements = simulation.orbitalElements(simulation.bodies()[1].id);
    if (!elements.has_value()) {
        FAIL("Sun/Earth preset did not produce orbital elements");
        return;
    }
    REQUIRE(elements->primaryBodyId == simulation.bodies()[0].id);
    REQUIRE(elements->bound);
    REQUIRE(elements->eccentricity < 1.0e-4);
    REQUIRE(elements->semiMajorAxis == Catch::Approx(1.0).margin(1.0e-5));
    REQUIRE(elements->period == Catch::Approx(2.0 * std::numbers::pi).margin(1.0e-4));
}
