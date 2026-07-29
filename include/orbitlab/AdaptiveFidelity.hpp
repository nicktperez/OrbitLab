#pragma once

#include "orbitlab/Body.hpp"

#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace orbitlab {

// OrbitLab Adaptive Fidelity (OAF) combines three physically dimensioned local
// timescales. It is an experimental controller, not a new law of gravity.
struct AdaptiveFidelitySettings {
    bool enabled{false};
    double safetyFactor{0.15};
    double jerkWeight{0.35};
    double encounterWeight{0.65};
    double minimumTimeStep{0.0000625};
    double maximumTimeStep{0.004};
};

enum class AdaptiveLimitReason { MaximumStep, Acceleration, Jerk, Encounter };

struct AdaptiveStepDecision {
    double timeStep{0.0};
    double rawTimeStep{0.0};
    double accelerationTimescale{0.0};
    double jerkTimescale{0.0};
    double encounterTimescale{0.0};
    std::uint64_t limitingBodyId{0};
    AdaptiveLimitReason reason{AdaptiveLimitReason::MaximumStep};
    std::vector<std::uint64_t> sampledBodyIds;
    std::vector<Vec3> sampledAccelerations;
};

class AdaptiveFidelityController {
public:
    [[nodiscard]] AdaptiveStepDecision propose(
        std::span<const Body> bodies,
        double gravitationalConstant,
        double softeningLength,
        const AdaptiveFidelitySettings& settings) const;

    void commit(const AdaptiveStepDecision& decision);
    void reset() noexcept;

    [[nodiscard]] static std::string validate(
        const AdaptiveFidelitySettings& settings);
    [[nodiscard]] static const char* reasonName(AdaptiveLimitReason reason) noexcept;

private:
    std::vector<std::uint64_t> previousBodyIds_;
    std::vector<Vec3> previousAccelerations_;
    double previousTimeStep_{0.0};
};

} // namespace orbitlab
