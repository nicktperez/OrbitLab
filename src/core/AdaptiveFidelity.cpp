#include "orbitlab/AdaptiveFidelity.hpp"

#include "orbitlab/GravitySolver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace orbitlab {
namespace {

constexpr double infinity = std::numeric_limits<double>::infinity();

double dyadicStepAtOrBelow(
    const double requested,
    const double minimum,
    const double maximum) {
    double step = maximum;
    while (step > requested && step > minimum) {
        step *= 0.5;
    }
    return std::clamp(step, minimum, maximum);
}

} // namespace

std::string AdaptiveFidelityController::validate(
    const AdaptiveFidelitySettings& settings) {
    if (!std::isfinite(settings.safetyFactor) ||
        settings.safetyFactor <= 0.0 || settings.safetyFactor > 1.0) {
        return "Adaptive safety factor must be finite and in the range (0, 1]";
    }
    if (!std::isfinite(settings.jerkWeight) || settings.jerkWeight < 0.0 ||
        settings.jerkWeight > 10.0) {
        return "Adaptive jerk weight must be finite and in the range [0, 10]";
    }
    if (!std::isfinite(settings.encounterWeight) ||
        settings.encounterWeight < 0.0 || settings.encounterWeight > 10.0) {
        return "Adaptive encounter weight must be finite and in the range [0, 10]";
    }
    if (!std::isfinite(settings.minimumTimeStep) ||
        !std::isfinite(settings.maximumTimeStep) ||
        settings.minimumTimeStep <= 0.0 ||
        settings.maximumTimeStep < settings.minimumTimeStep ||
        settings.maximumTimeStep > 1.0) {
        return "Adaptive time-step bounds must be finite, positive, ordered, and at most 1";
    }
    return {};
}

const char* AdaptiveFidelityController::reasonName(
    const AdaptiveLimitReason reason) noexcept {
    switch (reason) {
    case AdaptiveLimitReason::MaximumStep:
        return "maximum step";
    case AdaptiveLimitReason::Acceleration:
        return "acceleration";
    case AdaptiveLimitReason::Jerk:
        return "changing acceleration";
    case AdaptiveLimitReason::Encounter:
        return "close encounter";
    }
    return "unknown";
}

AdaptiveStepDecision AdaptiveFidelityController::propose(
    const std::span<const Body> bodies,
    const double gravitationalConstant,
    const double softeningLength,
    const AdaptiveFidelitySettings& settings) const {
    if (const std::string error = validate(settings); !error.empty()) {
        throw std::invalid_argument(error);
    }
    AdaptiveStepDecision decision;
    decision.timeStep = settings.maximumTimeStep;
    decision.rawTimeStep = settings.maximumTimeStep;
    decision.accelerationTimescale = infinity;
    decision.jerkTimescale = infinity;
    decision.encounterTimescale = infinity;
    decision.sampledBodyIds.reserve(bodies.size());
    for (const Body& body : bodies) {
        decision.sampledBodyIds.push_back(body.id);
    }
    if (bodies.size() < 2) {
        decision.sampledAccelerations.assign(bodies.size(), Vec3{});
        return decision;
    }

    decision.sampledAccelerations = DirectGravitySolver{}.accelerations(
        bodies, gravitationalConstant, softeningLength);
    const bool hasComparableHistory =
        previousTimeStep_ > 0.0 &&
        previousBodyIds_ == decision.sampledBodyIds &&
        previousAccelerations_.size() == bodies.size();

    double minimumCombinedTimescale = infinity;
    for (std::size_t index = 0; index < bodies.size(); ++index) {
        double nearestDistance = infinity;
        double encounterTimescale = infinity;
        for (std::size_t other = 0; other < bodies.size(); ++other) {
            if (other == index) {
                continue;
            }
            const Vec3 separation = bodies[other].position - bodies[index].position;
            const double distance = separation.length();
            nearestDistance = std::min(nearestDistance, distance);
            if (distance <= 0.0) {
                encounterTimescale = 0.0;
                continue;
            }
            const Vec3 relativeVelocity =
                bodies[other].velocity - bodies[index].velocity;
            const double closingSpeed =
                std::max(0.0, -separation.dot(relativeVelocity) / distance);
            if (closingSpeed > 0.0) {
                encounterTimescale =
                    std::min(encounterTimescale, distance / closingSpeed);
            }
        }

        const double effectiveLength =
            std::max(nearestDistance, std::max(softeningLength, 1.0e-15));
        const double accelerationMagnitude =
            decision.sampledAccelerations[index].length();
        const double accelerationTimescale = std::sqrt(
            effectiveLength /
            std::max(accelerationMagnitude, std::numeric_limits<double>::min()));
        double jerkTimescale = infinity;
        if (hasComparableHistory) {
            const double jerkMagnitude =
                (decision.sampledAccelerations[index] -
                 previousAccelerations_[index])
                    .length() /
                previousTimeStep_;
            if (jerkMagnitude > 0.0) {
                jerkTimescale =
                    accelerationMagnitude > 0.0
                        ? accelerationMagnitude / jerkMagnitude
                        : 0.0;
            }
        }

        double inverseTimescaleSquared =
            1.0 / (accelerationTimescale * accelerationTimescale);
        if (std::isfinite(jerkTimescale) && jerkTimescale > 0.0) {
            inverseTimescaleSquared +=
                settings.jerkWeight / (jerkTimescale * jerkTimescale);
        } else if (jerkTimescale == 0.0 && settings.jerkWeight > 0.0) {
            inverseTimescaleSquared = infinity;
        }
        if (std::isfinite(encounterTimescale) && encounterTimescale > 0.0) {
            inverseTimescaleSquared +=
                settings.encounterWeight /
                (encounterTimescale * encounterTimescale);
        } else if (encounterTimescale == 0.0 &&
                   settings.encounterWeight > 0.0) {
            inverseTimescaleSquared = infinity;
        }
        const double combinedTimescale =
            inverseTimescaleSquared > 0.0
                ? 1.0 / std::sqrt(inverseTimescaleSquared)
                : infinity;
        if (combinedTimescale < minimumCombinedTimescale) {
            minimumCombinedTimescale = combinedTimescale;
            decision.limitingBodyId = bodies[index].id;
            decision.accelerationTimescale = accelerationTimescale;
            decision.jerkTimescale = jerkTimescale;
            decision.encounterTimescale = encounterTimescale;

            const double accelerationTerm =
                1.0 / (accelerationTimescale * accelerationTimescale);
            const double jerkTerm =
                std::isfinite(jerkTimescale) && jerkTimescale > 0.0
                    ? settings.jerkWeight / (jerkTimescale * jerkTimescale)
                    : (jerkTimescale == 0.0 ? infinity : 0.0);
            const double encounterTerm =
                std::isfinite(encounterTimescale) && encounterTimescale > 0.0
                    ? settings.encounterWeight /
                          (encounterTimescale * encounterTimescale)
                    : (encounterTimescale == 0.0 ? infinity : 0.0);
            decision.reason = AdaptiveLimitReason::Acceleration;
            if (jerkTerm > accelerationTerm && jerkTerm >= encounterTerm) {
                decision.reason = AdaptiveLimitReason::Jerk;
            } else if (
                encounterTerm > accelerationTerm && encounterTerm > jerkTerm) {
                decision.reason = AdaptiveLimitReason::Encounter;
            }
        }
    }

    decision.rawTimeStep =
        std::clamp(
            settings.safetyFactor * minimumCombinedTimescale,
            settings.minimumTimeStep,
            settings.maximumTimeStep);
    decision.timeStep = dyadicStepAtOrBelow(
        decision.rawTimeStep,
        settings.minimumTimeStep,
        settings.maximumTimeStep);
    if (previousTimeStep_ > 0.0) {
        decision.timeStep = std::clamp(
            decision.timeStep,
            std::max(settings.minimumTimeStep, previousTimeStep_ * 0.25),
            std::min(settings.maximumTimeStep, previousTimeStep_ * 2.0));
    }
    if (decision.timeStep >= settings.maximumTimeStep) {
        decision.reason = AdaptiveLimitReason::MaximumStep;
    }
    return decision;
}

void AdaptiveFidelityController::commit(const AdaptiveStepDecision& decision) {
    if (!std::isfinite(decision.timeStep) || decision.timeStep <= 0.0 ||
        decision.sampledBodyIds.size() != decision.sampledAccelerations.size()) {
        throw std::invalid_argument("Cannot commit an invalid adaptive-step decision");
    }
    previousBodyIds_ = decision.sampledBodyIds;
    previousAccelerations_ = decision.sampledAccelerations;
    previousTimeStep_ = decision.timeStep;
}

void AdaptiveFidelityController::reset() noexcept {
    previousBodyIds_.clear();
    previousAccelerations_.clear();
    previousTimeStep_ = 0.0;
}

} // namespace orbitlab
