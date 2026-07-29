#include "orbitlab/AdaptiveExperiment.hpp"

#include "orbitlab/Simulation.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <nlohmann/json.hpp>
#include <numbers>

namespace orbitlab {
namespace {

Simulation eccentricOrbit() {
    constexpr double primaryMass = 1.0;
    constexpr double secondaryMass = 0.001;
    constexpr double semiMajorAxis = 1.0;
    constexpr double eccentricity = 0.8;
    constexpr double gravitationalConstant = 1.0;
    const double totalMass = primaryMass + secondaryMass;
    const double apoapsis = semiMajorAxis * (1.0 + eccentricity);
    const double relativeSpeed = std::sqrt(
        gravitationalConstant * totalMass *
        (2.0 / apoapsis - 1.0 / semiMajorAxis));

    Simulation simulation;
    simulation.settings().gravitationalConstant = gravitationalConstant;
    simulation.settings().softeningLength = 0.0;
    simulation.settings().collisionMode = CollisionMode::None;
    simulation.settings().integratorType = IntegratorType::RungeKutta4;
    simulation.addBody(
        {0,
         "Primary",
         primaryMass,
         0.01,
         {},
         {-secondaryMass / totalMass * apoapsis, 0.0, 0.0},
         {0.0, -secondaryMass / totalMass * relativeSpeed, 0.0}});
    simulation.addBody(
        {0,
         "Orbiter",
         secondaryMass,
         0.001,
         {},
         {primaryMass / totalMass * apoapsis, 0.0, 0.0},
         {0.0, primaryMass / totalMass * relativeSpeed, 0.0}});
    return simulation;
}

Vec3 relativePosition(const Simulation& simulation) {
    return simulation.bodies()[1].position - simulation.bodies()[0].position;
}

MethodRunResult runFixed(
    const double duration,
    const double requestedStep,
    const Vec3& referencePosition) {
    Simulation simulation = eccentricOrbit();
    const auto steps =
        static_cast<std::uint64_t>(std::ceil(duration / requestedStep));
    const double timeStep = duration / static_cast<double>(steps);
    const double initialEnergy = simulation.diagnostics().totalEnergy;
    const auto start = std::chrono::steady_clock::now();
    for (std::uint64_t step = 0; step < steps; ++step) {
        simulation.step(timeStep);
    }
    const double milliseconds =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start)
            .count();
    const double finalEnergy = simulation.diagnostics().totalEnergy;
    return {
        steps,
        milliseconds,
        (relativePosition(simulation) - referencePosition).length(),
        (finalEnergy - initialEnergy) / std::abs(initialEnergy) * 100.0,
        timeStep,
        timeStep,
    };
}

MethodRunResult runAdaptive(
    const double duration,
    const AdaptiveFidelitySettings& settings,
    const Vec3& referencePosition) {
    Simulation simulation = eccentricOrbit();
    AdaptiveFidelityController controller;
    const double initialEnergy = simulation.diagnostics().totalEnergy;
    double minimumStep = std::numeric_limits<double>::infinity();
    double maximumStep = 0.0;
    std::uint64_t steps = 0;
    const auto start = std::chrono::steady_clock::now();
    while (simulation.elapsedTime() < duration && steps < 1'000'000) {
        AdaptiveStepDecision decision = controller.propose(
            simulation.bodies(),
            simulation.settings().gravitationalConstant,
            simulation.settings().softeningLength,
            settings);
        const double remaining = duration - simulation.elapsedTime();
        decision.timeStep = std::min(decision.timeStep, remaining);
        simulation.step(decision.timeStep);
        controller.commit(decision);
        minimumStep = std::min(minimumStep, decision.timeStep);
        maximumStep = std::max(maximumStep, decision.timeStep);
        ++steps;
    }
    const double milliseconds =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start)
            .count();
    const double finalEnergy = simulation.diagnostics().totalEnergy;
    return {
        steps,
        milliseconds,
        (relativePosition(simulation) - referencePosition).length(),
        (finalEnergy - initialEnergy) / std::abs(initialEnergy) * 100.0,
        minimumStep,
        maximumStep,
    };
}

} // namespace

AdaptiveMethodReport runAdaptiveMethodExperiment() {
    constexpr double primaryPlusSecondaryMass = 1.001;
    const double duration =
        2.0 * std::numbers::pi /
        std::sqrt(primaryPlusSecondaryMass);

    Simulation reference = eccentricOrbit();
    constexpr std::uint64_t referenceSteps = 120'000;
    const double referenceStep = duration / static_cast<double>(referenceSteps);
    for (std::uint64_t step = 0; step < referenceSteps; ++step) {
        reference.step(referenceStep);
    }
    const Vec3 referencePosition = relativePosition(reference);

    AdaptiveMethodReport report;
    report.duration = duration;
    report.settings.enabled = true;
    report.settings.safetyFactor = 0.15;
    report.settings.jerkWeight = 0.35;
    report.settings.encounterWeight = 0.65;
    report.settings.minimumTimeStep = 0.000625;
    report.settings.maximumTimeStep = 0.04;
    report.coarseFixed =
        runFixed(duration, report.settings.maximumTimeStep, referencePosition);
    report.fineFixed =
        runFixed(duration, report.settings.minimumTimeStep, referencePosition);
    report.adaptive =
        runAdaptive(duration, report.settings, referencePosition);
    report.hypothesisPassed =
        report.adaptive.finalPositionError <
            report.coarseFixed.finalPositionError &&
        std::abs(report.adaptive.energyDriftPercent) <
            std::abs(report.coarseFixed.energyDriftPercent) &&
        report.adaptive.steps < report.fineFixed.steps;
    return report;
}

std::string serializeAdaptiveMethodReport(
    const AdaptiveMethodReport& report) {
    const auto runJson = [](const MethodRunResult& run) {
        return nlohmann::json{
            {"steps", run.steps},
            {"milliseconds", run.milliseconds},
            {"finalPositionError", run.finalPositionError},
            {"energyDriftPercent", run.energyDriftPercent},
            {"minimumStepUsed", run.minimumStepUsed},
            {"maximumStepUsed", run.maximumStepUsed},
        };
    };
    return nlohmann::json{
        {"format", "OrbitLabAdaptiveMethodExperiment"},
        {"version", 1},
        {"hypothesis",
         "Adaptive stepping reduces final position and energy error versus the "
         "coarse fixed step while using fewer steps than the fine fixed baseline."},
        {"hypothesisPassed", report.hypothesisPassed},
        {"duration", report.duration},
        {"settings",
         {
             {"safetyFactor", report.settings.safetyFactor},
             {"jerkWeight", report.settings.jerkWeight},
             {"encounterWeight", report.settings.encounterWeight},
             {"minimumTimeStep", report.settings.minimumTimeStep},
             {"maximumTimeStep", report.settings.maximumTimeStep},
         }},
        {"coarseFixed", runJson(report.coarseFixed)},
        {"fineFixed", runJson(report.fineFixed)},
        {"adaptive", runJson(report.adaptive)},
    }
        .dump(2);
}

} // namespace orbitlab
