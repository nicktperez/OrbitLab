#pragma once

#include "orbitlab/AdaptiveFidelity.hpp"

#include <cstdint>
#include <string>

namespace orbitlab {

struct MethodRunResult {
    std::uint64_t steps{0};
    double milliseconds{0.0};
    double finalPositionError{0.0};
    double energyDriftPercent{0.0};
    double minimumStepUsed{0.0};
    double maximumStepUsed{0.0};
};

struct AdaptiveMethodReport {
    AdaptiveFidelitySettings settings;
    double duration{0.0};
    MethodRunResult coarseFixed;
    MethodRunResult fineFixed;
    MethodRunResult adaptive;
    bool hypothesisPassed{false};
};

[[nodiscard]] AdaptiveMethodReport runAdaptiveMethodExperiment();
[[nodiscard]] std::string serializeAdaptiveMethodReport(
    const AdaptiveMethodReport& report);

} // namespace orbitlab
