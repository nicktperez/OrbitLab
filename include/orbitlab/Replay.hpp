#pragma once

#include "orbitlab/Simulation.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace orbitlab {

enum class ReplayCommandType {
    Step,
    RemoveBody,
    SetBodyState,
    SetSolver,
    SetIntegrator
};

struct ReplayCommand {
    ReplayCommandType type{ReplayCommandType::Step};
    std::uint64_t bodyId{0};
    std::uint64_t stepCount{0};
    double deltaTime{0.0};
    Vec3 position{};
    Vec3 velocity{};
    SolverType solverType{SolverType::Direct};
    IntegratorType integratorType{IntegratorType::VelocityVerlet};
};

struct ReplayRecording {
    std::string initialSimulationJson;
    std::vector<ReplayCommand> commands;
};

[[nodiscard]] std::uint64_t simulationStateHash(const Simulation& simulation);
[[nodiscard]] std::string simulationStateHashHex(const Simulation& simulation);
[[nodiscard]] std::string serializeReplay(const ReplayRecording& recording);
[[nodiscard]] ReplayRecording deserializeReplay(const std::string& jsonText);
[[nodiscard]] Simulation replaySimulation(const ReplayRecording& recording);

} // namespace orbitlab
