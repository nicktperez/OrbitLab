#include "orbitlab/Persistence.hpp"
#include "orbitlab/Presets.hpp"
#include "orbitlab/Replay.hpp"

#include <catch2/catch_test_macros.hpp>

using namespace orbitlab;

TEST_CASE("Replay round trip deterministically reproduces typed commands", "[replay]") {
    Simulation initial;
    loadPreset(initial, Preset::InclinedSystem);
    const auto editedBody = initial.bodies()[1].id;
    const auto removedBody = initial.bodies().back().id;
    ReplayRecording recording{
        serializeSimulation(initial),
        {
            {ReplayCommandType::SetIntegrator,
             0,
             0,
             0.0,
             {},
             {},
             SolverType::Direct,
             IntegratorType::Yoshida4},
            {ReplayCommandType::SetBodyState,
             editedBody,
             0,
             0.0,
             {0.7, -0.2, 0.3},
             {0.1, 0.8, -0.15}},
            {ReplayCommandType::Step, 0, 120, 0.001},
            {ReplayCommandType::RemoveBody, removedBody},
        },
    };

    const ReplayRecording restored = deserializeReplay(serializeReplay(recording));
    const Simulation first = replaySimulation(recording);
    const Simulation second = replaySimulation(restored);

    REQUIRE(first.bodies().size() == initial.bodies().size() - 1);
    REQUIRE(first.settings().integratorType == IntegratorType::Yoshida4);
    REQUIRE(simulationStateHash(first) == simulationStateHash(second));
}

TEST_CASE("Replay rejects unsafe step counts", "[replay][validation]") {
    Simulation simulation;
    loadPreset(simulation, Preset::SunEarth);
    const std::string replay =
        R"({"format":"OrbitLabReplay","version":1,"initialState":)" +
        serializeSimulation(simulation) +
        R"(,"commands":[{"type":"step","count":100000001,"deltaTime":0.001}]})";
    REQUIRE_THROWS(deserializeReplay(replay));
}
