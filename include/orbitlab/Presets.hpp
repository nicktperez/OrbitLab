#pragma once

#include "orbitlab/Simulation.hpp"

#include <cstdint>
#include <string_view>

namespace orbitlab {

enum class Preset {
    SunEarth,
    EarthMoon,
    BinaryStars,
    AsteroidField,
    InclinedSystem
};

[[nodiscard]] std::string_view presetName(Preset preset) noexcept;
void loadPreset(Simulation& simulation, Preset preset, std::uint32_t randomSeed = 42);

} // namespace orbitlab
