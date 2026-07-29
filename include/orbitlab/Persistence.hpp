#pragma once

#include "orbitlab/Simulation.hpp"

#include <filesystem>
#include <string>

namespace orbitlab {

struct PersistenceResult {
    bool success{false};
    std::string message;
};

[[nodiscard]] PersistenceResult saveSimulation(
    const Simulation& simulation,
    const std::filesystem::path& path);
[[nodiscard]] PersistenceResult loadSimulation(
    Simulation& simulation,
    const std::filesystem::path& path);
[[nodiscard]] std::string serializeSimulation(const Simulation& simulation);
void deserializeSimulation(Simulation& simulation, const std::string& jsonText);

} // namespace orbitlab
