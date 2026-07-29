#pragma once

#include "orbitlab/Simulation.hpp"

#include <string>
#include <vector>

namespace orbitlab {

struct IntegratorValidationSample {
    IntegratorType integrator{IntegratorType::VelocityVerlet};
    double timeStep{0.0};
    double positionError{0.0};
    double energyDriftPercent{0.0};
};

struct BarnesHutValidationSample {
    double openingAngle{0.0};
    double relativeRmsError{0.0};
};

struct NumericalValidationReport {
    std::vector<IntegratorValidationSample> integratorSamples;
    std::vector<BarnesHutValidationSample> barnesHutSamples;
    double centerOfMassDrift{0.0};
    double reversibilityError{0.0};
    bool passed{false};
};

[[nodiscard]] NumericalValidationReport runNumericalValidation();
[[nodiscard]] std::string serializeValidationReport(
    const NumericalValidationReport& report);

} // namespace orbitlab
