#include "orbitlab/Persistence.hpp"
#include "orbitlab/Presets.hpp"

#include <catch2/catch_test_macros.hpp>

using namespace orbitlab;

TEST_CASE("Simulation JSON round trip preserves state", "[persistence]") {
    Simulation source;
    loadPreset(source, Preset::BinaryStars);
    source.settings().trailsEnabled = false;
    source.settings().solverType = SolverType::BarnesHut;
    source.settings().barnesHutOpeningAngle = 0.45;
    source.settings().integratorType = IntegratorType::Yoshida4;
    source.settings().collisionMode = CollisionMode::Fragment;
    source.settings().fragmentationSpeedThreshold = 2.5;
    source.step(0.01);

    Simulation restored;
    deserializeSimulation(restored, serializeSimulation(source));

    REQUIRE(restored.bodies().size() == source.bodies().size());
    REQUIRE(restored.elapsedTime() == source.elapsedTime());
    REQUIRE(restored.settings().trailsEnabled == false);
    REQUIRE(restored.settings().solverType == SolverType::BarnesHut);
    REQUIRE(restored.settings().barnesHutOpeningAngle == 0.45);
    REQUIRE(restored.settings().integratorType == IntegratorType::Yoshida4);
    REQUIRE(restored.settings().collisionMode == CollisionMode::Fragment);
    REQUIRE(restored.settings().fragmentationSpeedThreshold == 2.5);
    for (std::size_t index = 0; index < source.bodies().size(); ++index) {
        REQUIRE(restored.bodies()[index].id == source.bodies()[index].id);
        REQUIRE(restored.bodies()[index].name == source.bodies()[index].name);
        REQUIRE(restored.bodies()[index].mass == source.bodies()[index].mass);
        REQUIRE(restored.bodies()[index].position == source.bodies()[index].position);
        REQUIRE(restored.bodies()[index].velocity == source.bodies()[index].velocity);
    }
}

TEST_CASE("Invalid and unsafe files are rejected without replacing state", "[persistence]") {
    Simulation simulation;
    loadPreset(simulation, Preset::SunEarth);
    const auto originalCount = simulation.bodies().size();

    REQUIRE_THROWS(deserializeSimulation(simulation, R"({"format":"other","version":1})"));
    REQUIRE(simulation.bodies().size() == originalCount);
}

TEST_CASE("Performance solver selection survives JSON persistence", "[persistence]") {
    Simulation source;
    loadPreset(source, Preset::InclinedSystem);
    source.settings().solverType = SolverType::ThreadedSoA;

    Simulation restored;
    deserializeSimulation(restored, serializeSimulation(source));

    REQUIRE(restored.settings().solverType == SolverType::ThreadedSoA);
    REQUIRE((restored.bodies()[1].position.z != 0.0 ||
             restored.bodies()[1].velocity.z != 0.0));
    REQUIRE(restored.bodies()[1].position == source.bodies()[1].position);
    REQUIRE(restored.bodies()[1].velocity == source.bodies()[1].velocity);
}

TEST_CASE("OrbitLab method settings survive JSON persistence", "[persistence][adaptive]") {
    Simulation source;
    loadPreset(source, Preset::SunEarth);
    source.settings().integratorType = IntegratorType::RungeKutta4;
    source.settings().adaptiveFidelity = {
        true,
        0.12,
        0.4,
        0.8,
        0.00025,
        0.02,
    };

    Simulation restored;
    deserializeSimulation(restored, serializeSimulation(source));

    CHECK(restored.settings().adaptiveFidelity.enabled);
    CHECK(restored.settings().adaptiveFidelity.safetyFactor == 0.12);
    CHECK(restored.settings().adaptiveFidelity.jerkWeight == 0.4);
    CHECK(restored.settings().adaptiveFidelity.encounterWeight == 0.8);
    CHECK(restored.settings().adaptiveFidelity.minimumTimeStep == 0.00025);
    CHECK(restored.settings().adaptiveFidelity.maximumTimeStep == 0.02);
    CHECK(restored.settings().integratorType == IntegratorType::RungeKutta4);
}

TEST_CASE("Version one planar files migrate into three-dimensional state", "[persistence][3d]") {
    constexpr auto oldFile = R"({
        "format":"OrbitLab",
        "version":1,
        "elapsedTime":0.0,
        "settings":{
            "gravitationalConstant":1.0,
            "softeningLength":0.001,
            "fixedTimeStep":0.01,
            "collisionMode":"none",
            "trailsEnabled":true
        },
        "bodies":[{
            "id":1,
            "name":"Legacy probe",
            "mass":1.0,
            "radius":0.01,
            "color":[1.0,1.0,1.0,1.0],
            "position":[2.0,-3.0],
            "velocity":[0.5,0.25]
        }]
    })";
    Simulation restored;
    deserializeSimulation(restored, oldFile);

    REQUIRE((restored.bodies().front().position == Vec3{2.0, -3.0, 0.0}));
    REQUIRE((restored.bodies().front().velocity == Vec3{0.5, 0.25, 0.0}));
}
