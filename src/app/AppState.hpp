#pragma once

#include "orbitlab/AdaptiveExperiment.hpp"
#include "orbitlab/Presets.hpp"
#include "orbitlab/Validation.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace orbitlab::app {

enum class ReferenceFrame { Inertial, FollowSelected, Barycenter, CoRotatingSelected };

struct RuntimeStats {
    double framesPerSecond{0.0};
    double stepMilliseconds{0.0};
    std::uint64_t totalSteps{0};
    SystemDiagnostics diagnostics{};
    double energyDriftPercent{0.0};
    double directSolverMilliseconds{0.0};
    double soaSolverMilliseconds{0.0};
    double threadedSolverMilliseconds{0.0};
    double barnesHutMilliseconds{0.0};
    double solverRelativeError{0.0};
    bool solverComparisonAvailable{false};
    bool gpuRendererActive{false};
    double gpuComputeRelativeError{0.0};
    double adaptiveControllerMilliseconds{0.0};
    std::uint64_t adaptiveStepCount{0};
    std::optional<AdaptiveStepDecision> adaptiveDecision;
};

struct DiagnosticSample {
    double simulationTime{0.0};
    float energyDriftPercent{0.0F};
    float momentumMagnitude{0.0F};
    float angularMomentum{0.0F};
    float stepMilliseconds{0.0F};
    float bodyCount{0.0F};
};

struct IntegratorComparison {
    double milliseconds{0.0};
    double energyDriftPercent{0.0};
};

struct AppState {
    bool paused{false};
    bool singleStepRequested{false};
    double speedMultiplier{1.0};
    std::optional<std::uint64_t> selectedBodyId;
    std::optional<std::uint64_t> hoveredBodyId;
    Preset selectedPreset{Preset::InclinedSystem};
    RuntimeStats stats;
    std::string notification;
    bool notificationIsError{false};
    bool showPrediction{true};
    bool solverComparisonRequested{false};
    bool integratorComparisonRequested{false};
    bool integratorComparisonAvailable{false};
    std::array<IntegratorComparison, 4> integratorComparison{};
    bool numericalValidationRequested{false};
    std::optional<NumericalValidationReport> numericalValidation;
    bool adaptiveExperimentRequested{false};
    std::optional<AdaptiveMethodReport> adaptiveExperiment;
    bool gpuComputeAvailable{false};
    bool gpuComputeEnabled{false};
    std::string rendererBackend;
    std::string rendererFallbackReason;
    std::string gpuComputeStatus;
    ReferenceFrame referenceFrame{ReferenceFrame::Inertial};
    bool creatingBody{false};
    Vec3 creationStart{};
    Vec3 creationCurrent{};
    std::uint64_t sceneRevision{0};
    std::array<char, 256> filePath{"orbitlab-simulation.json"};
    std::array<char, 96> bodySearch{};
    std::vector<DiagnosticSample> diagnosticSamples;
};

} // namespace orbitlab::app
