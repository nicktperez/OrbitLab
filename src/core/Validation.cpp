#include "orbitlab/Validation.hpp"

#include "orbitlab/GravitySolver.hpp"
#include "orbitlab/Presets.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <nlohmann/json.hpp>
#include <numbers>
#include <random>
#include <ranges>

namespace orbitlab {
namespace {

const char* integratorName(const IntegratorType type) {
    switch (type) {
    case IntegratorType::VelocityVerlet:
        return "velocity-verlet";
    case IntegratorType::SymplecticEuler:
        return "symplectic-euler";
    case IntegratorType::RungeKutta4:
        return "rk4";
    case IntegratorType::Yoshida4:
        return "yoshida4";
    }
    return "velocity-verlet";
}

Vec3 centerOfMass(const Simulation& simulation) {
    Vec3 weighted{};
    double totalMass = 0.0;
    for (const auto& body : simulation.bodies()) {
        weighted += body.position * body.mass;
        totalMass += body.mass;
    }
    return totalMass > 0.0 ? weighted / totalMass : Vec3{};
}

Vec3 centerOfMassVelocity(const Simulation& simulation) {
    Vec3 momentum{};
    double totalMass = 0.0;
    for (const auto& body : simulation.bodies()) {
        momentum += body.velocity * body.mass;
        totalMass += body.mass;
    }
    return totalMass > 0.0 ? momentum / totalMass : Vec3{};
}

Vec3 referenceRelativePosition(const double duration) {
    Simulation reference;
    loadPreset(reference, Preset::SunEarth);
    reference.settings().collisionMode = CollisionMode::None;
    reference.settings().softeningLength = 0.0;
    reference.settings().integratorType = IntegratorType::RungeKutta4;
    constexpr std::uint64_t referenceSteps = 100'000;
    const double timeStep = duration / static_cast<double>(referenceSteps);
    for (std::uint64_t step = 0; step < referenceSteps; ++step) {
        reference.step(timeStep);
    }
    return reference.bodies()[1].position - reference.bodies()[0].position;
}

IntegratorValidationSample validateIntegrator(
    const IntegratorType integrator,
    const double requestedTimeStep,
    const double duration,
    const Vec3& referenceRelative) {
    Simulation simulation;
    loadPreset(simulation, Preset::SunEarth);
    simulation.settings().collisionMode = CollisionMode::None;
    simulation.settings().softeningLength = 0.0;
    simulation.settings().integratorType = integrator;
    const double initialEnergy = simulation.diagnostics().totalEnergy;
    const auto steps =
        static_cast<std::uint64_t>(std::ceil(duration / requestedTimeStep));
    const double timeStep = duration / static_cast<double>(steps);
    for (std::uint64_t step = 0; step < steps; ++step) {
        simulation.step(timeStep);
    }
    const Vec3 finalRelative =
        simulation.bodies()[1].position - simulation.bodies()[0].position;
    const double finalEnergy = simulation.diagnostics().totalEnergy;
    return {
        integrator,
        timeStep,
        (finalRelative - referenceRelative).length(),
        std::abs(initialEnergy) > 1.0e-15
            ? (finalEnergy - initialEnergy) / std::abs(initialEnergy) * 100.0
            : 0.0,
    };
}

std::vector<Body> validationCloud() {
    std::mt19937_64 generator{0x4f524249544c4142ULL};
    std::uniform_real_distribution<double> position{-2.0, 2.0};
    std::uniform_real_distribution<double> mass{0.01, 1.0};
    std::vector<Body> bodies;
    bodies.reserve(256);
    for (std::uint64_t index = 1; index <= 256; ++index) {
        bodies.push_back(
            {index,
             "Validation body",
             mass(generator),
             0.001,
             {},
             {position(generator), position(generator), position(generator)},
             {}});
    }
    return bodies;
}

BarnesHutValidationSample validateBarnesHut(
    const std::vector<Body>& bodies,
    const std::vector<Vec3>& reference,
    const double openingAngle) {
    const BarnesHutGravitySolver solver{openingAngle};
    const auto approximate = solver.accelerations(bodies, 1.0, 0.01);
    double errorSquared = 0.0;
    double referenceSquared = 0.0;
    for (std::size_t index = 0; index < bodies.size(); ++index) {
        errorSquared += (approximate[index] - reference[index]).lengthSquared();
        referenceSquared += reference[index].lengthSquared();
    }
    return {
        openingAngle,
        referenceSquared > 0.0 ? std::sqrt(errorSquared / referenceSquared) : 0.0,
    };
}

} // namespace

NumericalValidationReport runNumericalValidation() {
    NumericalValidationReport report;
    constexpr std::array integrators{
        IntegratorType::VelocityVerlet,
        IntegratorType::SymplecticEuler,
        IntegratorType::RungeKutta4,
        IntegratorType::Yoshida4,
    };
    constexpr std::array timeSteps{0.08, 0.04, 0.02, 0.01};
    const double validationDuration = 2.0 * std::numbers::pi;
    const Vec3 referenceRelative =
        referenceRelativePosition(validationDuration);
    for (const auto integrator : integrators) {
        for (const double timeStep : timeSteps) {
            report.integratorSamples.push_back(
                validateIntegrator(
                    integrator,
                    timeStep,
                    validationDuration,
                    referenceRelative));
        }
    }

    const auto cloud = validationCloud();
    const DirectGravitySolver direct;
    const auto reference = direct.accelerations(cloud, 1.0, 0.01);
    for (const double openingAngle : {0.3, 0.5, 0.7, 1.0}) {
        report.barnesHutSamples.push_back(
            validateBarnesHut(cloud, reference, openingAngle));
    }

    Simulation momentumSystem;
    loadPreset(momentumSystem, Preset::InclinedSystem);
    momentumSystem.settings().collisionMode = CollisionMode::None;
    const Vec3 initialCenter = centerOfMass(momentumSystem);
    const Vec3 initialCenterVelocity = centerOfMassVelocity(momentumSystem);
    constexpr int momentumSteps = 20'000;
    constexpr double momentumTimeStep = 0.001;
    for (int step = 0; step < momentumSteps; ++step) {
        momentumSystem.step(momentumTimeStep);
    }
    const Vec3 expectedCenter =
        initialCenter +
        initialCenterVelocity *
            (static_cast<double>(momentumSteps) * momentumTimeStep);
    report.centerOfMassDrift =
        (centerOfMass(momentumSystem) - expectedCenter).length();

    Simulation reversible;
    loadPreset(reversible, Preset::SunEarth);
    reversible.settings().collisionMode = CollisionMode::None;
    reversible.settings().softeningLength = 0.0;
    const auto initialBodies = reversible.bodies();
    constexpr int reversibleSteps = 4'000;
    constexpr double reversibleTimeStep = 0.001;
    for (int step = 0; step < reversibleSteps; ++step) {
        reversible.step(reversibleTimeStep);
    }
    for (auto& body : reversible.bodies()) {
        body.velocity *= -1.0;
    }
    for (int step = 0; step < reversibleSteps; ++step) {
        reversible.step(reversibleTimeStep);
    }
    double squaredError = 0.0;
    for (std::size_t index = 0; index < initialBodies.size(); ++index) {
        squaredError +=
            (reversible.bodies()[index].position - initialBodies[index].position)
                .lengthSquared();
        squaredError +=
            (reversible.bodies()[index].velocity + initialBodies[index].velocity)
                .lengthSquared();
    }
    report.reversibilityError =
        std::sqrt(squaredError / static_cast<double>(initialBodies.size() * 2));

    const bool samplesFinite =
        std::ranges::all_of(report.integratorSamples, [](const auto& sample) {
            return std::isfinite(sample.positionError) &&
                   std::isfinite(sample.energyDriftPercent);
        }) &&
        std::ranges::all_of(report.barnesHutSamples, [](const auto& sample) {
            return std::isfinite(sample.relativeRmsError);
        });
    report.passed = samplesFinite && report.centerOfMassDrift < 1.0e-9 &&
                    report.reversibilityError < 1.0e-8 &&
                    report.barnesHutSamples.front().relativeRmsError < 0.01;
    return report;
}

std::string serializeValidationReport(const NumericalValidationReport& report) {
    nlohmann::json integrators = nlohmann::json::array();
    for (const auto& sample : report.integratorSamples) {
        integrators.push_back(
            {
                {"integrator", integratorName(sample.integrator)},
                {"timeStep", sample.timeStep},
                {"positionError", sample.positionError},
                {"energyDriftPercent", sample.energyDriftPercent},
            });
    }
    nlohmann::json barnesHut = nlohmann::json::array();
    for (const auto& sample : report.barnesHutSamples) {
        barnesHut.push_back(
            {
                {"openingAngle", sample.openingAngle},
                {"relativeRmsError", sample.relativeRmsError},
            });
    }
    return nlohmann::json{
        {"format", "OrbitLabNumericalValidation"},
        {"version", 1},
        {"passed", report.passed},
        {"centerOfMassDrift", report.centerOfMassDrift},
        {"reversibilityError", report.reversibilityError},
        {"integrators", std::move(integrators)},
        {"barnesHut", std::move(barnesHut)},
    }
        .dump(2);
}

} // namespace orbitlab
