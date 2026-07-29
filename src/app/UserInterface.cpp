#include "app/UserInterface.hpp"

#include "orbitlab/Persistence.hpp"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <cfloat>
#include <cctype>
#include <cmath>
#include <cstring>
#include <numbers>
#include <ranges>
#include <string>

namespace orbitlab::app {
namespace {

bool positiveFiniteInput(const char* label, double& value, const char* format = "%.6g") {
    double candidate = value;
    if (!ImGui::InputDouble(label, &candidate, 0.0, 0.0, format)) {
        return false;
    }
    if (std::isfinite(candidate) && candidate > 0.0) {
        value = candidate;
        return true;
    }
    return false;
}

void metricRow(const char* label, const char* value) {
    ImGui::TextDisabled("%s", label);
    ImGui::SameLine(154.0F);
    ImGui::TextUnformatted(value);
}

template <typename Projection>
void diagnosticPlot(
    const char* label,
    const std::vector<DiagnosticSample>& samples,
    Projection projection) {
    std::vector<float> values;
    values.reserve(samples.size());
    for (const auto& sample : samples) {
        values.push_back(projection(sample));
    }
    ImGui::PlotLines(
        label,
        values.data(),
        static_cast<int>(values.size()),
        0,
        values.empty() ? "Waiting for samples" : nullptr,
        FLT_MAX,
        FLT_MAX,
        {-1.0F, 58.0F});
}

} // namespace

void UserInterface::applyStyle() {
    auto& style = ImGui::GetStyle();
    style.WindowRounding = 0.0F;
    style.ChildRounding = 6.0F;
    style.FrameRounding = 5.0F;
    style.PopupRounding = 6.0F;
    style.GrabRounding = 4.0F;
    style.FramePadding = {9.0F, 6.0F};
    style.ItemSpacing = {8.0F, 8.0F};
    style.WindowPadding = {18.0F, 16.0F};
    style.ScrollbarSize = 12.0F;

    auto* colors = style.Colors;
    colors[ImGuiCol_Text] = {0.91F, 0.93F, 0.92F, 1.0F};
    colors[ImGuiCol_TextDisabled] = {0.55F, 0.61F, 0.59F, 1.0F};
    colors[ImGuiCol_WindowBg] = {0.075F, 0.086F, 0.083F, 1.0F};
    colors[ImGuiCol_ChildBg] = {0.095F, 0.11F, 0.105F, 1.0F};
    colors[ImGuiCol_PopupBg] = {0.09F, 0.105F, 0.10F, 1.0F};
    colors[ImGuiCol_Border] = {0.18F, 0.22F, 0.205F, 1.0F};
    colors[ImGuiCol_FrameBg] = {0.12F, 0.145F, 0.135F, 1.0F};
    colors[ImGuiCol_FrameBgHovered] = {0.16F, 0.20F, 0.18F, 1.0F};
    colors[ImGuiCol_FrameBgActive] = {0.20F, 0.28F, 0.24F, 1.0F};
    colors[ImGuiCol_TitleBg] = colors[ImGuiCol_WindowBg];
    colors[ImGuiCol_TitleBgActive] = colors[ImGuiCol_WindowBg];
    colors[ImGuiCol_Button] = {0.14F, 0.18F, 0.16F, 1.0F};
    colors[ImGuiCol_ButtonHovered] = {0.20F, 0.31F, 0.25F, 1.0F};
    colors[ImGuiCol_ButtonActive] = {0.25F, 0.52F, 0.38F, 1.0F};
    colors[ImGuiCol_Header] = {0.14F, 0.20F, 0.17F, 1.0F};
    colors[ImGuiCol_HeaderHovered] = {0.19F, 0.29F, 0.24F, 1.0F};
    colors[ImGuiCol_HeaderActive] = {0.25F, 0.49F, 0.37F, 1.0F};
    colors[ImGuiCol_CheckMark] = {0.42F, 0.78F, 0.57F, 1.0F};
    colors[ImGuiCol_SliderGrab] = {0.42F, 0.78F, 0.57F, 1.0F};
    colors[ImGuiCol_SliderGrabActive] = {0.53F, 0.90F, 0.68F, 1.0F};
    colors[ImGuiCol_Separator] = {0.16F, 0.20F, 0.19F, 1.0F};
}

void UserInterface::draw(
    Simulation& simulation,
    Camera& camera,
    History& history,
    AppState& state,
    const int windowHeight) {
    ImGui::SetNextWindowPos({0.0F, 0.0F});
    ImGui::SetNextWindowSize({panelWidth, static_cast<float>(windowHeight)});
    constexpr auto flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                           ImGuiWindowFlags_NoSavedSettings;
    ImGui::Begin("OrbitLab instruments", nullptr, flags);

    ImGui::TextColored({0.52F, 0.88F, 0.68F, 1.0F}, "OrbitLab");
    ImGui::SameLine();
    ImGui::TextDisabled("N-body workbench");
    ImGui::Spacing();

    if (!state.notification.empty()) {
        const ImVec4 color = state.notificationIsError
                                 ? ImVec4{0.95F, 0.48F, 0.42F, 1.0F}
                                 : ImVec4{0.52F, 0.88F, 0.68F, 1.0F};
        ImGui::PushTextWrapPos();
        ImGui::TextColored(color, "%s", state.notification.c_str());
        ImGui::PopTextWrapPos();
        ImGui::Separator();
    }

    drawTransport(simulation, history, state);
    drawScenario(simulation, camera, history, state);
    drawBodyList(simulation, camera, state);
    drawInspector(simulation, history, state);
    drawDisplay(simulation, camera, history, state);
    drawDiagnosticPlots(state);
    drawNumericalValidation(state);
    drawAdaptiveMethod(simulation, history, state);
    drawCollisionLog(simulation);
    drawPersistence(simulation, camera, history, state);

    if (ImGui::CollapsingHeader("Controls")) {
        ImGui::TextDisabled("Space  Pause/resume   .  Single step");
        ImGui::TextDisabled("Cmd/Ctrl+Z  Undo   Shift+Cmd/Ctrl+Z  Redo");
        ImGui::TextDisabled("Left-drag  Move body   Middle/Shift-right  Orbit");
        ImGui::TextDisabled("Right-drag  Pan camera");
        ImGui::TextDisabled("Drag empty canvas  Create body + velocity");
        ImGui::TextDisabled("Drag green handle  Change velocity");
        ImGui::TextDisabled("Scroll  Zoom   R  Reset camera");
    }
    ImGui::End();

    ImGui::GetForegroundDrawList()->AddText(
        {panelWidth + 16.0F, 16.0F},
        IM_COL32(145, 158, 153, 205),
        "Drag empty space to create  |  Middle/Shift-right orbit  |  Right-drag pan  |  Scroll zoom");

    if (state.hoveredBodyId) {
        if (const Body* hovered = simulation.findBody(*state.hoveredBodyId)) {
            std::array<char, 192> hoverText{};
            std::snprintf(
                hoverText.data(),
                hoverText.size(),
                "%s   x %.4g   y %.4g   z %.4g   |v| %.4g",
                hovered->name.c_str(),
                hovered->position.x,
                hovered->position.y,
                hovered->position.z,
                hovered->velocity.length());
            ImGui::GetForegroundDrawList()->AddText(
                {panelWidth + 16.0F, static_cast<float>(windowHeight) - 30.0F},
                IM_COL32(170, 188, 181, 230),
                hoverText.data());
        }
    }
}

void UserInterface::drawDiagnosticPlots(const AppState& state) {
    if (!ImGui::CollapsingHeader("Live plots")) {
        return;
    }
    diagnosticPlot(
        "Energy drift (%)",
        state.diagnosticSamples,
        [](const DiagnosticSample& sample) { return sample.energyDriftPercent; });
    diagnosticPlot(
        "|Momentum|",
        state.diagnosticSamples,
        [](const DiagnosticSample& sample) { return sample.momentumMagnitude; });
    diagnosticPlot(
        "Angular momentum",
        state.diagnosticSamples,
        [](const DiagnosticSample& sample) { return sample.angularMomentum; });
    diagnosticPlot(
        "Step time (ms)",
        state.diagnosticSamples,
        [](const DiagnosticSample& sample) { return sample.stepMilliseconds; });
    diagnosticPlot(
        "Body count",
        state.diagnosticSamples,
        [](const DiagnosticSample& sample) { return sample.bodyCount; });
}

void UserInterface::drawAdaptiveMethod(
    Simulation& simulation,
    History& history,
    AppState& state) {
    if (!ImGui::CollapsingHeader("OrbitLab method (experimental)")) {
        return;
    }
    ImGui::TextWrapped(
        "Chooses a timestep from acceleration, changing acceleration, and "
        "approaching encounters. It changes numerical control, not gravity.");

    auto settings = simulation.settings().adaptiveFidelity;
    bool enabled = settings.enabled;
    if (ImGui::Checkbox("Use adaptive fidelity", &enabled)) {
        history.commit(simulation);
        simulation.settings().integratorType = IntegratorType::RungeKutta4;
        simulation.settings().adaptiveFidelity.enabled = enabled;
        history.commit(simulation);
        ++state.sceneRevision;
        setNotification(
            state,
            enabled
                ? "OrbitLab method enabled; integrator set to Runge-Kutta 4"
                : "OrbitLab method disabled; fixed timestep restored",
            false);
    }
    ImGui::TextDisabled(
        "RK4 is required. Exact force auditing is capped at 4,096 bodies.");

    if (enabled) {
        settings = simulation.settings().adaptiveFidelity;
        const auto commitSettings = [&](const AdaptiveFidelitySettings& candidate) {
            if (AdaptiveFidelityController::validate(candidate).empty()) {
                history.commit(simulation);
                simulation.settings().adaptiveFidelity = candidate;
                history.commit(simulation);
                ++state.sceneRevision;
            }
        };

        double safety = settings.safetyFactor;
        constexpr double minimumSafety = 0.02;
        constexpr double maximumSafety = 0.5;
        if (ImGui::SliderScalar(
                "Safety factor",
                ImGuiDataType_Double,
                &safety,
                &minimumSafety,
                &maximumSafety,
                "%.3f")) {
            auto candidate = settings;
            candidate.safetyFactor = safety;
            commitSettings(candidate);
        }

        double minimumStep = settings.minimumTimeStep;
        if (positiveFiniteInput("Minimum step", minimumStep, "%.7g")) {
            auto candidate = settings;
            candidate.minimumTimeStep = minimumStep;
            commitSettings(candidate);
        }
        double maximumStep = settings.maximumTimeStep;
        if (positiveFiniteInput("Maximum step", maximumStep, "%.7g")) {
            auto candidate = settings;
            candidate.maximumTimeStep = maximumStep;
            commitSettings(candidate);
        }

        if (ImGui::TreeNode("Equation weights")) {
            double jerkWeight = settings.jerkWeight;
            if (ImGui::InputDouble(
                    "Changing-acceleration weight",
                    &jerkWeight,
                    0.0,
                    0.0,
                    "%.4g") &&
                std::isfinite(jerkWeight) && jerkWeight >= 0.0) {
                auto candidate = settings;
                candidate.jerkWeight = jerkWeight;
                commitSettings(candidate);
            }
            double encounterWeight = settings.encounterWeight;
            if (ImGui::InputDouble(
                    "Encounter weight",
                    &encounterWeight,
                    0.0,
                    0.0,
                    "%.4g") &&
                std::isfinite(encounterWeight) && encounterWeight >= 0.0) {
                auto candidate = settings;
                candidate.encounterWeight = encounterWeight;
                commitSettings(candidate);
            }
            ImGui::TreePop();
        }

        if (state.stats.adaptiveDecision) {
            const auto& decision = *state.stats.adaptiveDecision;
            std::array<char, 64> step{};
            std::array<char, 64> overhead{};
            std::array<char, 96> limiter{};
            std::snprintf(step.data(), step.size(), "%.7g", decision.timeStep);
            std::snprintf(
                overhead.data(),
                overhead.size(),
                "%.4f ms",
                state.stats.adaptiveControllerMilliseconds);
            const Body* body =
                simulation.findBody(decision.limitingBodyId);
            std::snprintf(
                limiter.data(),
                limiter.size(),
                "%s%s%s",
                AdaptiveFidelityController::reasonName(decision.reason),
                body != nullptr ? " · " : "",
                body != nullptr ? body->name.c_str() : "");
            metricRow("Chosen step", step.data());
            metricRow("Limiter", limiter.data());
            metricRow("Controller cost", overhead.data());
        } else {
            ImGui::TextDisabled("Run or single-step to produce a decision.");
        }
    }

    ImGui::Spacing();
    if (ImGui::Button("Run fixed vs adaptive experiment", {-1.0F, 0.0F})) {
        state.adaptiveExperimentRequested = true;
    }
    ImGui::TextDisabled(
        "Deterministic eccentric orbit · unchanged pass/fail hypothesis");
    if (state.adaptiveExperiment) {
        const auto& report = *state.adaptiveExperiment;
        ImGui::TextColored(
            report.hypothesisPassed
                ? ImVec4{0.52F, 0.88F, 0.68F, 1.0F}
                : ImVec4{0.95F, 0.48F, 0.42F, 1.0F},
            "%s",
            report.hypothesisPassed ? "Hypothesis passed" : "Hypothesis failed");
        std::array<char, 80> coarse{};
        std::array<char, 80> adaptive{};
        std::array<char, 80> fine{};
        std::snprintf(
            coarse.data(),
            coarse.size(),
            "%llu steps · error %.3g",
            static_cast<unsigned long long>(report.coarseFixed.steps),
            report.coarseFixed.finalPositionError);
        std::snprintf(
            adaptive.data(),
            adaptive.size(),
            "%llu steps · error %.3g",
            static_cast<unsigned long long>(report.adaptive.steps),
            report.adaptive.finalPositionError);
        std::snprintf(
            fine.data(),
            fine.size(),
            "%llu steps · error %.3g",
            static_cast<unsigned long long>(report.fineFixed.steps),
            report.fineFixed.finalPositionError);
        metricRow("Coarse fixed", coarse.data());
        metricRow("OrbitLab method", adaptive.data());
        metricRow("Fine fixed", fine.data());
    }
}

void UserInterface::drawNumericalValidation(AppState& state) {
    if (!ImGui::CollapsingHeader("Numerical validation")) {
        return;
    }
    ImGui::TextWrapped(
        "Runs deterministic convergence, conservation, reversibility, and octree-error checks.");
    if (ImGui::Button("Run validation suite", {-1.0F, 0.0F})) {
        state.numericalValidationRequested = true;
    }
    if (!state.numericalValidation) {
        ImGui::TextDisabled("No validation report in this session.");
        return;
    }
    const auto& report = *state.numericalValidation;
    ImGui::TextColored(
        report.passed ? ImVec4{0.52F, 0.88F, 0.68F, 1.0F}
                      : ImVec4{0.95F, 0.48F, 0.42F, 1.0F},
        "%s",
        report.passed ? "All validation thresholds passed" : "Validation threshold failed");

    std::array<char, 64> centerDrift{};
    std::array<char, 64> reversibility{};
    std::snprintf(
        centerDrift.data(),
        centerDrift.size(),
        "%.3g",
        report.centerOfMassDrift);
    std::snprintf(
        reversibility.data(),
        reversibility.size(),
        "%.3g",
        report.reversibilityError);
    metricRow("Center-of-mass drift", centerDrift.data());
    metricRow("Reversibility RMS", reversibility.data());

    ImGui::SeparatorText("Finest-step integrator error");
    constexpr const char* names[] = {
        "Velocity Verlet", "Symplectic Euler", "Runge-Kutta 4", "Yoshida 4"};
    for (std::size_t integrator = 0; integrator < 4; ++integrator) {
        const std::size_t index = integrator * 4 + 3;
        if (index >= report.integratorSamples.size()) {
            break;
        }
        const auto& sample = report.integratorSamples[index];
        std::array<char, 80> value{};
        std::snprintf(
            value.data(),
            value.size(),
            "%.3g pos  |  %+.3g%% energy",
            sample.positionError,
            sample.energyDriftPercent);
        metricRow(names[integrator], value.data());
    }

    ImGui::SeparatorText("Barnes-Hut RMS error");
    for (const auto& sample : report.barnesHutSamples) {
        std::array<char, 48> label{};
        std::array<char, 48> value{};
        std::snprintf(
            label.data(), label.size(), "Theta %.1f", sample.openingAngle);
        std::snprintf(
            value.data(),
            value.size(),
            "%.3g%%",
            sample.relativeRmsError * 100.0);
        metricRow(label.data(), value.data());
    }
}

void UserInterface::drawTransport(
    Simulation& simulation,
    History& history,
    AppState& state) {
    if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button(state.paused ? "Resume" : "Pause", {104.0F, 0.0F})) {
            state.paused = !state.paused;
        }
        ImGui::SameLine();
        ImGui::BeginDisabled(!state.paused);
        if (ImGui::Button("Single step", {104.0F, 0.0F})) {
            state.singleStepRequested = true;
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(!history.canUndo());
        if (ImGui::Button("Undo")) {
            if (history.undo(simulation)) {
                ++state.sceneRevision;
                state.selectedBodyId.reset();
                setNotification(state, "Undid the last scene edit", false);
            }
        }
        ImGui::EndDisabled();

        ImGui::BeginDisabled(!history.canRedo());
        if (ImGui::Button("Redo", {104.0F, 0.0F})) {
            if (history.redo(simulation)) {
                ++state.sceneRevision;
                state.selectedBodyId.reset();
                setNotification(state, "Redid the scene edit", false);
            }
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("Restore initial", {-1.0F, 0.0F})) {
            if (history.restoreInitial(simulation)) {
                ++state.sceneRevision;
                state.selectedBodyId.reset();
                setNotification(state, "Restored the initial scenario state", false);
            }
        }

        ImGui::SetNextItemWidth(-1.0F);
        constexpr double minimumSpeed = 0.1;
        constexpr double maximumSpeed = 10.0;
        ImGui::SliderScalar(
            "##speed",
            ImGuiDataType_Double,
            &state.speedMultiplier,
            &minimumSpeed,
            &maximumSpeed,
            "Speed  %.1fx",
            ImGuiSliderFlags_Logarithmic);

        std::array<char, 48> fps{};
        std::array<char, 48> count{};
        std::array<char, 48> step{};
        std::array<char, 48> elapsed{};
        std::snprintf(fps.data(), fps.size(), "%.1f", state.stats.framesPerSecond);
        std::snprintf(count.data(), count.size(), "%zu", simulation.bodies().size());
        std::snprintf(step.data(), step.size(), "%.4f ms", state.stats.stepMilliseconds);
        std::snprintf(elapsed.data(), elapsed.size(), "%.3f", simulation.elapsedTime());
        metricRow("Frames / second", fps.data());
        metricRow("Bodies", count.data());
        metricRow("Mean step", step.data());
        metricRow("Simulation time", elapsed.data());
    }
}

void UserInterface::drawScenario(
    Simulation& simulation,
    Camera& camera,
    History& history,
    AppState& state) {
    if (!ImGui::CollapsingHeader("Scenario", ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }
    constexpr std::array<Preset, 5> presets{
        Preset::SunEarth,
        Preset::EarthMoon,
        Preset::BinaryStars,
        Preset::AsteroidField,
        Preset::InclinedSystem};
    const char* preview = presetName(state.selectedPreset).data();
    if (ImGui::BeginCombo("##preset", preview)) {
        for (const auto preset : presets) {
            const bool selected = preset == state.selectedPreset;
            if (ImGui::Selectable(presetName(preset).data(), selected)) {
                state.selectedPreset = preset;
            }
            if (selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    if (ImGui::Button("Load", {-1.0F, 0.0F})) {
        loadPreset(simulation, state.selectedPreset);
        history.reset(simulation);
        ++state.sceneRevision;
        state.selectedBodyId =
            simulation.bodies().empty() ? std::nullopt
                                        : std::optional{simulation.bodies().front().id};
        camera.reset();
        setNotification(state, std::string{presetName(state.selectedPreset)} + " loaded", false);
    }

    if (ImGui::Button("Add body", {-1.0F, 0.0F})) {
        history.commit(simulation);
        const auto number = simulation.bodies().size() + 1;
        auto& body = simulation.addBody(
            {0, "Body " + std::to_string(number), 1.0e-4, 0.018,
             {0.53F, 0.84F, 0.67F, 1.0F}, camera.center(), {}});
        state.selectedBodyId = body.id;
        state.paused = true;
        history.commit(simulation);
        ++state.sceneRevision;
        setNotification(state, "Body added at the camera center; simulation paused", false);
    }
}

void UserInterface::drawBodyList(
    Simulation& simulation,
    Camera& camera,
    AppState& state) {
    if (!ImGui::CollapsingHeader("Bodies")) {
        return;
    }
    ImGui::SetNextItemWidth(-1.0F);
    ImGui::InputTextWithHint(
        "##body-search", "Search bodies", state.bodySearch.data(), state.bodySearch.size());
    std::string filter = state.bodySearch.data();
    std::ranges::transform(filter, filter.begin(), [](const unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });

    ImGui::BeginChild("body-list", {0.0F, 142.0F}, ImGuiChildFlags_Borders);
    for (const auto& body : simulation.bodies()) {
        std::string searchableName = body.name;
        std::ranges::transform(
            searchableName, searchableName.begin(), [](const unsigned char character) {
                return static_cast<char>(std::tolower(character));
            });
        if (!filter.empty() && searchableName.find(filter) == std::string::npos) {
            continue;
        }
        const std::string label = body.name + "##" + std::to_string(body.id);
        if (ImGui::Selectable(label.c_str(), state.selectedBodyId == body.id)) {
            state.selectedBodyId = body.id;
            camera.focus(body.position);
        }
    }
    ImGui::EndChild();
}

void UserInterface::drawInspector(
    Simulation& simulation,
    History& history,
    AppState& state) {
    if (!ImGui::CollapsingHeader("Selected body", ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }
    Body* body = state.selectedBodyId ? simulation.findBody(*state.selectedBodyId) : nullptr;
    if (body == nullptr) {
        ImGui::TextDisabled("Select a body on the canvas to edit it.");
        return;
    }

    std::array<char, 128> name{};
    std::strncpy(name.data(), body->name.c_str(), name.size() - 1);
    if (ImGui::InputText("Name", name.data(), name.size())) {
        if (name[0] != '\0') {
            history.commit(simulation);
            body->name = name.data();
            history.commit(simulation);
            ++state.sceneRevision;
        } else {
            setNotification(state, "A body name cannot be empty", true);
        }
    }

    double mass = body->mass;
    if (positiveFiniteInput("Mass", mass, "%.8g")) {
        history.commit(simulation);
        body->mass = mass;
        history.commit(simulation);
        ++state.sceneRevision;
        state.notification.clear();
    } else if (ImGui::IsItemDeactivatedAfterEdit()) {
        setNotification(state, "Mass must be finite and greater than zero", true);
    }
    double radius = body->radius;
    if (positiveFiniteInput("Radius", radius, "%.6g")) {
        history.commit(simulation);
        body->radius = radius;
        history.commit(simulation);
        ++state.sceneRevision;
        state.notification.clear();
    } else if (ImGui::IsItemDeactivatedAfterEdit()) {
        setNotification(state, "Radius must be finite and greater than zero", true);
    }
    Color color = body->color;
    if (ImGui::ColorEdit3("Color", &color.r, ImGuiColorEditFlags_NoInputs)) {
        history.commit(simulation);
        body->color = color;
        history.commit(simulation);
        ++state.sceneRevision;
    }

    double position[3]{body->position.x, body->position.y, body->position.z};
    if (ImGui::InputScalarN("Position", ImGuiDataType_Double, position, 3, nullptr, nullptr,
                            "%.5g") &&
        std::isfinite(position[0]) && std::isfinite(position[1]) &&
        std::isfinite(position[2])) {
        history.commit(simulation);
        body->position = {position[0], position[1], position[2]};
        history.commit(simulation);
        ++state.sceneRevision;
    }
    double velocity[3]{body->velocity.x, body->velocity.y, body->velocity.z};
    if (ImGui::InputScalarN("Velocity", ImGuiDataType_Double, velocity, 3, nullptr, nullptr,
                            "%.5g") &&
        std::isfinite(velocity[0]) && std::isfinite(velocity[1]) &&
        std::isfinite(velocity[2])) {
        history.commit(simulation);
        body->velocity = {velocity[0], velocity[1], velocity[2]};
        history.commit(simulation);
        ++state.sceneRevision;
    }

    ImGui::PushStyleColor(ImGuiCol_Text, {0.95F, 0.48F, 0.42F, 1.0F});
    if (ImGui::Button("Delete body", {-1.0F, 0.0F})) {
        history.commit(simulation);
        simulation.removeBody(body->id);
        history.commit(simulation);
        ++state.sceneRevision;
        state.selectedBodyId.reset();
        setNotification(state, "Body deleted", false);
        ImGui::PopStyleColor();
        return;
    }
    ImGui::PopStyleColor();

    if (const auto elements = simulation.orbitalElements(body->id)) {
        const Body* primary = simulation.findBody(elements->primaryBodyId);
        ImGui::SeparatorText("Orbit relative to reference");
        if (primary != nullptr) {
            metricRow("Reference", primary->name.c_str());
        }
        std::array<char, 64> separation{};
        std::array<char, 64> speed{};
        std::array<char, 64> eccentricity{};
        std::array<char, 64> semiMajor{};
        std::snprintf(separation.data(), separation.size(), "%.7g", elements->separation);
        std::snprintf(speed.data(), speed.size(), "%.7g", elements->relativeSpeed);
        std::snprintf(eccentricity.data(), eccentricity.size(), "%.6g", elements->eccentricity);
        std::snprintf(semiMajor.data(), semiMajor.size(), "%.7g", elements->semiMajorAxis);
        metricRow("Separation", separation.data());
        metricRow("Relative speed", speed.data());
        metricRow("Eccentricity", eccentricity.data());
        metricRow("Semi-major axis", semiMajor.data());
        if (elements->bound) {
            std::array<char, 64> periapsis{};
            std::array<char, 64> apoapsis{};
            std::array<char, 64> period{};
            std::snprintf(periapsis.data(), periapsis.size(), "%.7g", elements->periapsis);
            std::snprintf(apoapsis.data(), apoapsis.size(), "%.7g", elements->apoapsis);
            std::snprintf(period.data(), period.size(), "%.7g", elements->period);
            metricRow("Periapsis", periapsis.data());
            metricRow("Apoapsis", apoapsis.data());
            metricRow("Period", period.data());
        } else {
            ImGui::TextDisabled("Open trajectory");
        }
    }
}

void UserInterface::drawDisplay(
    Simulation& simulation,
    Camera& camera,
    History& history,
    AppState& state) {
    if (!ImGui::CollapsingHeader("View and physics")) {
        return;
    }
    bool trailsEnabled = simulation.settings().trailsEnabled;
    if (ImGui::Checkbox("Trajectory trails", &trailsEnabled)) {
        history.commit(simulation);
        simulation.settings().trailsEnabled = trailsEnabled;
        history.commit(simulation);
        ++state.sceneRevision;
    }
    ImGui::Checkbox("Predicted selected-body path", &state.showPrediction);
    if (simulation.bodies().size() > 512 && state.showPrediction) {
        ImGui::TextDisabled("Prediction pauses above 512 bodies.");
    }

    int referenceFrame = static_cast<int>(state.referenceFrame);
    constexpr const char* frameNames[] = {
        "Inertial",
        "Follow selected",
        "System barycenter",
        "Co-rotating selected pair",
    };
    if (ImGui::Combo("Reference frame", &referenceFrame, frameNames, 4)) {
        state.referenceFrame = static_cast<ReferenceFrame>(referenceFrame);
    }

    ImGui::SeparatorText("3D camera");
    double azimuthDegrees = camera.azimuth() * 180.0 / std::numbers::pi;
    double elevationDegrees = camera.elevation() * 180.0 / std::numbers::pi;
    constexpr double minimumAzimuth = -180.0;
    constexpr double maximumAzimuth = 180.0;
    constexpr double minimumElevation = -89.0;
    constexpr double maximumElevation = 89.0;
    if (ImGui::SliderScalar(
            "Azimuth",
            ImGuiDataType_Double,
            &azimuthDegrees,
            &minimumAzimuth,
            &maximumAzimuth,
            "%.0f deg")) {
        camera.setAzimuth(azimuthDegrees * std::numbers::pi / 180.0);
    }
    if (ImGui::SliderScalar(
            "Elevation",
            ImGuiDataType_Double,
            &elevationDegrees,
            &minimumElevation,
            &maximumElevation,
            "%.0f deg")) {
        camera.setElevation(elevationDegrees * std::numbers::pi / 180.0);
    }
    if (ImGui::Button("Isometric view", {160.0F, 0.0F})) {
        camera.setAzimuth(-std::numbers::pi * 0.5);
        camera.setElevation(0.78);
    }
    ImGui::SameLine();
    if (ImGui::Button("Top view", {-1.0F, 0.0F})) {
        camera.setAzimuth(-std::numbers::pi * 0.5);
        camera.setElevation(1.553);
    }
    int collisionMode = static_cast<int>(simulation.settings().collisionMode);
    constexpr const char* collisionNames[] = {
        "None", "Merge", "Elastic rebound", "Larger body absorbs", "Fragment"};
    if (ImGui::Combo("Collisions", &collisionMode, collisionNames, 5)) {
        history.commit(simulation);
        simulation.settings().collisionMode = static_cast<CollisionMode>(collisionMode);
        history.commit(simulation);
        ++state.sceneRevision;
    }
    if (simulation.settings().collisionMode == CollisionMode::Fragment) {
        double threshold = simulation.settings().fragmentationSpeedThreshold;
        if (positiveFiniteInput("Fragment speed", threshold, "%.5g")) {
            history.commit(simulation);
            simulation.settings().fragmentationSpeedThreshold = threshold;
            history.commit(simulation);
            ++state.sceneRevision;
        }
    }

    int solverIndex = static_cast<int>(simulation.settings().solverType);
    constexpr const char* solverNames[] = {
        "Direct pairwise O(n²)",
        "SoA direct O(n²)",
        "Threaded SoA O(n²)",
        "Barnes–Hut O(n log n)"};
    if (ImGui::Combo("Solver", &solverIndex, solverNames, 4)) {
        history.commit(simulation);
        simulation.settings().solverType = static_cast<SolverType>(solverIndex);
        history.commit(simulation);
        ++state.sceneRevision;
    }
    if (simulation.settings().solverType == SolverType::BarnesHut) {
        constexpr double minimumTheta = 0.2;
        constexpr double maximumTheta = 1.2;
        double theta = simulation.settings().barnesHutOpeningAngle;
        if (ImGui::SliderScalar(
                "Opening angle",
                ImGuiDataType_Double,
                &theta,
                &minimumTheta,
                &maximumTheta,
                "%.2f")) {
            history.commit(simulation);
            simulation.settings().barnesHutOpeningAngle = theta;
            history.commit(simulation);
            ++state.sceneRevision;
        }
    }
    if (simulation.settings().solverType == SolverType::ThreadedSoA) {
        int workerCount =
            static_cast<int>(simulation.settings().threadWorkerCount);
        if (ImGui::InputInt("Worker threads", &workerCount)) {
            workerCount = std::clamp(workerCount, 0, 256);
            history.commit(simulation);
            simulation.settings().threadWorkerCount =
                static_cast<unsigned int>(workerCount);
            history.commit(simulation);
            ++state.sceneRevision;
        }
        ImGui::TextDisabled("0 selects hardware concurrency automatically.");
    }

    int integrator = static_cast<int>(simulation.settings().integratorType);
    constexpr const char* integratorNames[] = {
        "Velocity Verlet", "Symplectic Euler", "Runge–Kutta 4", "Yoshida 4"};
    if (ImGui::Combo("Integrator", &integrator, integratorNames, 4)) {
        history.commit(simulation);
        simulation.settings().integratorType =
            static_cast<IntegratorType>(integrator);
        if (simulation.settings().adaptiveFidelity.enabled &&
            simulation.settings().integratorType !=
                IntegratorType::RungeKutta4) {
            simulation.settings().adaptiveFidelity.enabled = false;
            setNotification(
                state,
                "OrbitLab method disabled because its validated domain currently requires RK4",
                false);
        }
        history.commit(simulation);
        ++state.sceneRevision;
    }

    double fixedStep = simulation.settings().fixedTimeStep;
    if (positiveFiniteInput("Fixed step", fixedStep, "%.6g")) {
        history.commit(simulation);
        simulation.settings().fixedTimeStep = fixedStep;
        history.commit(simulation);
        ++state.sceneRevision;
    }

    ImGui::SeparatorText("Conserved quantities");
    std::array<char, 64> totalEnergy{};
    std::array<char, 64> energyDrift{};
    std::array<char, 64> momentum{};
    std::array<char, 64> angularMomentum{};
    std::snprintf(
        totalEnergy.data(),
        totalEnergy.size(),
        "%.7g",
        state.stats.diagnostics.totalEnergy);
    std::snprintf(
        energyDrift.data(),
        energyDrift.size(),
        "%+.4g%%",
        state.stats.energyDriftPercent);
    std::snprintf(
        momentum.data(),
        momentum.size(),
        "%.5g",
        state.stats.diagnostics.linearMomentum.length());
    std::snprintf(
        angularMomentum.data(),
        angularMomentum.size(),
        "%.7g",
        state.stats.diagnostics.angularMomentum);
    metricRow("Total energy", totalEnergy.data());
    metricRow("Energy drift", energyDrift.data());
    metricRow("|Momentum|", momentum.data());
    metricRow("Angular momentum", angularMomentum.data());

    ImGui::SeparatorText("Performance lab");
    metricRow(
        "Render backend",
        state.rendererBackend.empty() ? "Detecting" : state.rendererBackend.c_str());
    if (!state.rendererFallbackReason.empty()) {
        ImGui::TextWrapped("%s", state.rendererFallbackReason.c_str());
    }
    ImGui::BeginDisabled(!state.gpuComputeAvailable);
    ImGui::Checkbox("Use GPU compute gravity", &state.gpuComputeEnabled);
    ImGui::EndDisabled();
    ImGui::TextWrapped("%s", state.gpuComputeStatus.c_str());
    if (state.gpuComputeAvailable) {
        std::array<char, 64> gpuError{};
        std::snprintf(
            gpuError.data(),
            gpuError.size(),
            "%.3g%%",
            state.stats.gpuComputeRelativeError * 100.0);
        metricRow("Startup GPU error", gpuError.data());
        ImGui::TextDisabled(
            "GPU force evaluation uses float precision; integration and collisions stay on CPU.");
    }
    ImGui::Spacing();
    ImGui::TextDisabled("Local force-evaluation timing; lower is better.");
    if (ImGui::Button("Profile all solvers on current state", {-1.0F, 0.0F})) {
        state.solverComparisonRequested = true;
    }
    if (state.stats.solverComparisonAvailable) {
        std::array<char, 64> directTime{};
        std::array<char, 64> soaTime{};
        std::array<char, 64> threadedTime{};
        std::array<char, 64> barnesHutTime{};
        std::array<char, 64> relativeError{};
        std::snprintf(
            directTime.data(),
            directTime.size(),
            "%.4f ms",
            state.stats.directSolverMilliseconds);
        std::snprintf(
            soaTime.data(),
            soaTime.size(),
            "%.4f ms",
            state.stats.soaSolverMilliseconds);
        std::snprintf(
            threadedTime.data(),
            threadedTime.size(),
            "%.4f ms",
            state.stats.threadedSolverMilliseconds);
        std::snprintf(
            barnesHutTime.data(),
            barnesHutTime.size(),
            "%.4f ms",
            state.stats.barnesHutMilliseconds);
        std::snprintf(
            relativeError.data(),
            relativeError.size(),
            "%.3g%%",
            state.stats.solverRelativeError * 100.0);
        metricRow("Pairwise direct", directTime.data());
        metricRow("SoA direct", soaTime.data());
        metricRow("Threaded SoA", threadedTime.data());
        metricRow("Barnes–Hut", barnesHutTime.data());
        metricRow("RMS accel. error", relativeError.data());
    }
    if (ImGui::Button("Compare integrators from current state", {-1.0F, 0.0F})) {
        state.integratorComparisonRequested = true;
    }
    if (state.integratorComparisonAvailable) {
        constexpr const char* shortNames[] = {
            "Velocity Verlet", "Symplectic Euler", "Runge–Kutta 4", "Yoshida 4"};
        ImGui::TextDisabled("Total time · final energy drift");
        for (std::size_t index = 0; index < state.integratorComparison.size(); ++index) {
            std::array<char, 80> result{};
            std::snprintf(
                result.data(),
                result.size(),
                "%.3f ms  ·  %+.3g%%",
                state.integratorComparison[index].milliseconds,
                state.integratorComparison[index].energyDriftPercent);
            metricRow(shortNames[index], result.data());
        }
    }
    if (ImGui::Button("Reset camera", {-1.0F, 0.0F})) {
        camera.reset();
    }
}

void UserInterface::drawCollisionLog(const Simulation& simulation) {
    if (!ImGui::CollapsingHeader("Collision events")) {
        return;
    }
    if (simulation.collisionEvents().empty()) {
        ImGui::TextDisabled("Merged bodies will appear here.");
        return;
    }
    ImGui::BeginChild("collision-log", {0.0F, 120.0F}, ImGuiChildFlags_Borders);
    const auto& events = simulation.collisionEvents();
    const std::size_t first = events.size() > 12 ? events.size() - 12 : 0;
    for (std::size_t index = first; index < events.size(); ++index) {
        const auto& event = events[index];
        ImGui::Text(
            "t %.3f  %s · %s + %s",
            event.simulationTime,
            event.eventType.c_str(),
            event.firstBody.c_str(),
            event.secondBody.c_str());
        ImGui::TextDisabled(
            "  %s · mass %.6g · Δp %.3g",
            event.mergedBody.c_str(),
            event.combinedMass,
            event.momentumError);
    }
    ImGui::EndChild();
}

void UserInterface::drawPersistence(
    Simulation& simulation,
    Camera& camera,
    History& history,
    AppState& state) {
    if (!ImGui::CollapsingHeader("Save and load")) {
        return;
    }
    ImGui::InputText("##path", state.filePath.data(), state.filePath.size());
    if (ImGui::Button("Save JSON", {150.0F, 0.0F})) {
        const auto result = saveSimulation(simulation, state.filePath.data());
        setNotification(state, result.message, !result.success);
    }
    ImGui::SameLine();
    if (ImGui::Button("Load JSON", {-1.0F, 0.0F})) {
        const auto result = loadSimulation(simulation, state.filePath.data());
        if (result.success) {
            history.reset(simulation);
            ++state.sceneRevision;
            state.selectedBodyId =
                simulation.bodies().empty() ? std::nullopt
                                            : std::optional{simulation.bodies().front().id};
            camera.reset();
        }
        setNotification(state, result.message, !result.success);
    }
}

void UserInterface::setNotification(
    AppState& state,
    std::string message,
    const bool error) {
    state.notification = std::move(message);
    state.notificationIsError = error;
}

} // namespace orbitlab::app
