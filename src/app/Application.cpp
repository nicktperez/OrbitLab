#include "app/Application.hpp"

#include "app/GpuGravitySolver.hpp"
#include "orbitlab/Persistence.hpp"
#include "orbitlab/Presets.hpp"

#include <backends/imgui_impl_sdl3.h>
#include <backends/imgui_impl_sdlrenderer3.h>
#include <imgui.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <numbers>
#include <numeric>
#include <stdexcept>
#include <string>

namespace orbitlab::app {
namespace {

[[noreturn]] void throwSdlError(const char* action) {
    throw std::runtime_error(std::string{action} + ": " + SDL_GetError());
}

} // namespace

Application::Application() {
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        throwSdlError("Could not initialize SDL");
    }
    window_ = SDL_CreateWindow(
            "OrbitLab — 3D N-body gravity workbench",
            1440,
            900,
            SDL_WINDOW_RESIZABLE);
    if (window_ == nullptr) {
        SDL_Quit();
        throwSdlError("Could not create the application window");
    }
    gpuPlatform_ = std::make_unique<GpuPlatform>(window_);
    sdlRenderer_ = gpuPlatform_->renderer();
    state_.stats.gpuRendererActive = gpuPlatform_->gpuRendererActive();
    state_.rendererBackend = gpuPlatform_->backendName();
    state_.rendererFallbackReason = gpuPlatform_->fallbackReason();
    SDL_SetWindowMinimumSize(window_, 980, 640);
    SDL_SetRenderVSync(sdlRenderer_, 1);
    defaultCursor_ = SDL_GetDefaultCursor();
    pointerCursor_ = SDL_CreateSystemCursor(SDL_SYSTEM_CURSOR_POINTER);
    moveCursor_ = SDL_CreateSystemCursor(SDL_SYSTEM_CURSOR_MOVE);
    crosshairCursor_ = SDL_CreateSystemCursor(SDL_SYSTEM_CURSOR_CROSSHAIR);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    auto& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.IniFilename = nullptr;
    const auto fontPath = std::filesystem::path{SDL_GetBasePath()} / "assets/Karla-Regular.ttf";
    if (std::filesystem::exists(fontPath)) {
        io.Fonts->AddFontFromFileTTF(fontPath.string().c_str(), 17.0F);
    } else {
        io.Fonts->AddFontDefault();
    }
    userInterface_.applyStyle();
    if (!ImGui_ImplSDL3_InitForSDLRenderer(window_, sdlRenderer_)) {
        throwSdlError("Could not initialize the ImGui SDL backend");
    }
    if (!ImGui_ImplSDLRenderer3_Init(sdlRenderer_)) {
        throwSdlError("Could not initialize the ImGui renderer backend");
    }

    renderer_ = std::make_unique<Renderer>(sdlRenderer_);
    {
        auto gpuSolver = std::make_unique<GpuGravitySolver>(gpuPlatform_->device());
        state_.gpuComputeAvailable = gpuSolver->available();
        state_.gpuComputeStatus = gpuSolver->status();
        if (gpuSolver->available()) {
            const std::array<Body, 2> probe{
                Body{1, "Probe A", 2.0, 0.1, {}, {-0.7, 0.1, 0.2}, {}},
                Body{2, "Probe B", 3.0, 0.1, {}, {0.8, -0.2, -0.1}, {}}};
            const auto reference =
                DirectGravitySolver{}.accelerations(probe, 1.0, 1.0e-4);
            const auto candidate = gpuSolver->accelerations(probe, 1.0, 1.0e-4);
            double errorSquared = 0.0;
            double referenceSquared = 0.0;
            for (std::size_t index = 0; index < probe.size(); ++index) {
                errorSquared +=
                    (candidate[index] - reference[index]).lengthSquared();
                referenceSquared += reference[index].lengthSquared();
            }
            state_.stats.gpuComputeRelativeError =
                std::sqrt(errorSquared / std::max(referenceSquared, 1.0e-30));
            if (state_.stats.gpuComputeRelativeError > 1.0e-4) {
                state_.gpuComputeAvailable = false;
                state_.gpuComputeStatus =
                    "Disabled: startup correctness probe exceeded tolerance";
            } else {
                state_.gpuComputeStatus =
                    "Ready; startup result agrees with the CPU direct solver";
            }
        }
        SDL_Log(
            "OrbitLab renderer: %s; GPU compute: %s",
            state_.rendererBackend.c_str(),
            state_.gpuComputeStatus.c_str());
    }
    loadPreset(simulation_, Preset::InclinedSystem);
    history_.reset(simulation_);
    state_.selectedBodyId =
        simulation_.bodies().size() > 1 ? simulation_.bodies()[1].id
                                        : simulation_.bodies().front().id;
    state_.stats.diagnostics = simulation_.diagnostics();
    baselineEnergy_ = state_.stats.diagnostics.totalEnergy;
    baselineEnergyValid_ = true;
}

Application::~Application() {
    simulation_.clearGravitySolverOverride();
    renderer_.reset();
    if (ImGui::GetCurrentContext() != nullptr) {
        ImGui_ImplSDLRenderer3_Shutdown();
        ImGui_ImplSDL3_Shutdown();
        ImGui::DestroyContext();
    }
    sdlRenderer_ = nullptr;
    gpuPlatform_.reset();
    if (window_ != nullptr) {
        SDL_DestroyWindow(window_);
    }
    SDL_DestroyCursor(pointerCursor_);
    SDL_DestroyCursor(moveCursor_);
    SDL_DestroyCursor(crosshairCursor_);
    SDL_Quit();
}

int Application::run() {
    auto previousTime = std::chrono::steady_clock::now();
    std::uint64_t renderedFrames = 0;
    bool captureComplete = false;
    while (running_) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL3_ProcessEvent(&event);
            processEvent(event);
        }

        const auto currentTime = std::chrono::steady_clock::now();
        const double frameSeconds = std::min(
            std::chrono::duration<double>(currentTime - previousTime).count(), 0.25);
        previousTime = currentTime;
        update(frameSeconds);

        int width = 0;
        int height = 0;
        SDL_GetWindowSize(window_, &width, &height);

        ImGui_ImplSDLRenderer3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();
        userInterface_.draw(simulation_, camera_, history_, state_, height);

        updatePrediction();
        renderer_->draw(
            simulation_,
            camera_,
            trails_,
            prediction_,
            state_,
            width,
            height,
            UserInterface::panelWidth);
        ImGui::Render();
        ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), sdlRenderer_);

        ++renderedFrames;
        const char* capturePath = std::getenv("ORBITLAB_CAPTURE_PATH");
        if (!captureComplete && capturePath != nullptr && renderedFrames >= 120) {
            SDL_Surface* surface = SDL_RenderReadPixels(sdlRenderer_, nullptr);
            if (surface != nullptr) {
                if (!SDL_SaveBMP(surface, capturePath)) {
                    SDL_Log("Could not save OrbitLab capture: %s", SDL_GetError());
                }
                SDL_DestroySurface(surface);
            } else {
                SDL_Log("Could not read OrbitLab render surface: %s", SDL_GetError());
            }
            captureComplete = true;
            if (std::getenv("ORBITLAB_CAPTURE_EXIT") != nullptr) {
                running_ = false;
            }
        }
        SDL_RenderPresent(sdlRenderer_);
    }
    return 0;
}

void Application::processEvent(const SDL_Event& event) {
    if (event.type == SDL_EVENT_QUIT) {
        running_ = false;
        return;
    }
    if (event.type == SDL_EVENT_KEY_DOWN && !event.key.repeat &&
        !ImGui::GetIO().WantCaptureKeyboard) {
        if (event.key.key == SDLK_SPACE) {
            state_.paused = !state_.paused;
        } else if (event.key.key == SDLK_PERIOD && state_.paused) {
            state_.singleStepRequested = true;
        } else if (event.key.key == SDLK_R) {
            camera_.reset();
        } else if (event.key.key == SDLK_Z &&
                   (event.key.mod & (SDL_KMOD_CTRL | SDL_KMOD_GUI)) != 0) {
            const bool redo = (event.key.mod & SDL_KMOD_SHIFT) != 0;
            if (redo ? history_.redo(simulation_) : history_.undo(simulation_)) {
                markSceneChanged();
            }
        } else if (event.key.key == SDLK_Y &&
                   (event.key.mod & (SDL_KMOD_CTRL | SDL_KMOD_GUI)) != 0) {
            if (history_.redo(simulation_)) {
                markSceneChanged();
            }
        }
    }

    // Once a canvas gesture starts, keep ownership of it until button release even if the
    // pointer crosses the instrument panel. This prevents stuck drags when ImGui begins
    // capturing the mouse midway through the gesture.
    if (state_.creatingBody) {
        if (event.type == SDL_EVENT_MOUSE_MOTION) {
            updateBodyCreation(event.motion.x, event.motion.y);
        } else if (event.type == SDL_EVENT_MOUSE_BUTTON_UP &&
                   event.button.button == SDL_BUTTON_LEFT) {
            endBodyCreation();
        }
        return;
    }
    if (draggingVelocity_) {
        if (event.type == SDL_EVENT_MOUSE_MOTION) {
            updateVelocityDrag(event.motion.x, event.motion.y);
        } else if (event.type == SDL_EVENT_MOUSE_BUTTON_UP &&
                   event.button.button == SDL_BUTTON_LEFT) {
            endVelocityDrag();
        }
        return;
    }
    if (draggedBodyId_) {
        if (event.type == SDL_EVENT_MOUSE_MOTION) {
            updateBodyDrag(event.motion.x, event.motion.y);
        } else if (event.type == SDL_EVENT_MOUSE_BUTTON_UP &&
                   event.button.button == SDL_BUTTON_LEFT) {
            endBodyDrag();
        }
        return;
    }
    if (panning_) {
        if (event.type == SDL_EVENT_MOUSE_MOTION) {
            const Vec2 current{event.motion.x, event.motion.y};
            int width = 0;
            int height = 0;
            SDL_GetWindowSize(window_, &width, &height);
            camera_.panPixels(
                current - previousMouse_,
                static_cast<double>(width) - UserInterface::panelWidth,
                static_cast<double>(height));
            previousMouse_ = current;
        } else if (event.type == SDL_EVENT_MOUSE_BUTTON_UP) {
            panning_ = false;
        }
        return;
    }
    if (orbitingCamera_) {
        if (event.type == SDL_EVENT_MOUSE_MOTION) {
            const Vec2 current{event.motion.x, event.motion.y};
            camera_.orbitPixels(current - previousMouse_);
            previousMouse_ = current;
        } else if (event.type == SDL_EVENT_MOUSE_BUTTON_UP) {
            orbitingCamera_ = false;
        }
        return;
    }
    if (ImGui::GetIO().WantCaptureMouse) {
        return;
    }

    if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN &&
        event.button.x > UserInterface::panelWidth) {
        const bool shifted =
            (SDL_GetModState() & SDL_KMOD_SHIFT) != 0;
        if (event.button.button == SDL_BUTTON_RIGHT && !shifted) {
            panning_ = true;
            previousMouse_ = {event.button.x, event.button.y};
        } else if (event.button.button == SDL_BUTTON_MIDDLE ||
                   (event.button.button == SDL_BUTTON_RIGHT && shifted)) {
            orbitingCamera_ = true;
            previousMouse_ = {event.button.x, event.button.y};
        } else if (event.button.button == SDL_BUTTON_LEFT) {
            if (velocityHandleAt(event.button.x, event.button.y)) {
                beginVelocityDrag();
            } else {
                const auto hit = bodyAt(event.button.x, event.button.y);
                state_.selectedBodyId = hit;
                if (hit) {
                    beginBodyDrag(*hit, event.button.x, event.button.y);
                } else {
                    beginBodyCreation(event.button.x, event.button.y);
                }
            }
        }
    } else if (event.type == SDL_EVENT_MOUSE_MOTION) {
        updateHover(event.motion.x, event.motion.y);
    } else if (event.type == SDL_EVENT_MOUSE_WHEEL) {
        float mouseX = 0.0F;
        float mouseY = 0.0F;
        SDL_GetMouseState(&mouseX, &mouseY);
        if (mouseX > UserInterface::panelWidth) {
            int width = 0;
            int height = 0;
            SDL_GetWindowSize(window_, &width, &height);
            const Vec2 canvasPoint{mouseX - UserInterface::panelWidth, mouseY};
            camera_.zoomAt(
                std::pow(1.12, event.wheel.y),
                canvasPoint,
                static_cast<double>(width) - UserInterface::panelWidth,
                static_cast<double>(height));
        }
    }
}

void Application::update(const double frameSeconds) {
    synchronizeGpuSolver();
    const double instantaneousFps = frameSeconds > 0.0 ? 1.0 / frameSeconds : 0.0;
    state_.stats.framesPerSecond =
        state_.stats.framesPerSecond == 0.0
            ? instantaneousFps
            : state_.stats.framesPerSecond * 0.92 + instantaneousFps * 0.08;

    if (!state_.paused) {
        accumulator_ += frameSeconds * state_.speedMultiplier;
    }
    const bool singleStep = state_.singleStepRequested;
    if (singleStep) {
        const double stepBudget = simulation_.settings().adaptiveFidelity.enabled
                                      ? simulation_.settings()
                                            .adaptiveFidelity.maximumTimeStep
                                      : simulation_.settings().fixedTimeStep;
        accumulator_ = std::max(accumulator_, stepBudget);
        state_.singleStepRequested = false;
    }
    if (adaptiveWasEnabled_ !=
        simulation_.settings().adaptiveFidelity.enabled) {
        adaptiveController_.reset();
        state_.stats.adaptiveDecision.reset();
        adaptiveWasEnabled_ = simulation_.settings().adaptiveFidelity.enabled;
    }

    constexpr int maximumStepsPerFrame = 500;
    const int frameStepLimit = singleStep ? 1 : maximumStepsPerFrame;
    int performedSteps = 0;
    double totalStepMilliseconds = 0.0;
    while (performedSteps < frameStepLimit) {
        double timeStep = simulation_.settings().fixedTimeStep;
        std::optional<AdaptiveStepDecision> adaptiveDecision;
        if (simulation_.settings().adaptiveFidelity.enabled) {
            if (simulation_.bodies().size() > 4'096) {
                simulation_.settings().adaptiveFidelity.enabled = false;
                state_.notification =
                    "OrbitLab method disabled above 4,096 bodies to bound its exact-force audit";
                state_.notificationIsError = true;
                adaptiveController_.reset();
            } else {
                const auto controllerStart = std::chrono::steady_clock::now();
                adaptiveDecision = adaptiveController_.propose(
                    simulation_.bodies(),
                    simulation_.settings().gravitationalConstant,
                    simulation_.settings().softeningLength,
                    simulation_.settings().adaptiveFidelity);
                const double controllerMilliseconds =
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - controllerStart)
                        .count();
                state_.stats.adaptiveControllerMilliseconds =
                    state_.stats.adaptiveControllerMilliseconds == 0.0
                        ? controllerMilliseconds
                        : state_.stats.adaptiveControllerMilliseconds * 0.85 +
                              controllerMilliseconds * 0.15;
                timeStep = adaptiveDecision->timeStep;
            }
        }
        if (accumulator_ < timeStep) {
            break;
        }
        const auto start = std::chrono::steady_clock::now();
        try {
            simulation_.step(timeStep);
        } catch (const std::exception& exception) {
            state_.paused = true;
            state_.notification = std::string{"Simulation paused: "} + exception.what();
            state_.notificationIsError = true;
            accumulator_ = 0.0;
            break;
        }
        if (adaptiveDecision) {
            adaptiveController_.commit(*adaptiveDecision);
            state_.stats.adaptiveDecision = std::move(adaptiveDecision);
            ++state_.stats.adaptiveStepCount;
        }
        totalStepMilliseconds += std::chrono::duration<double, std::milli>(
                                     std::chrono::steady_clock::now() - start)
                                     .count();
        accumulator_ -= timeStep;
        ++performedSteps;
        ++state_.stats.totalSteps;
        recordTrails();
    }
    if (performedSteps > 0) {
        const double mean = totalStepMilliseconds / performedSteps;
        state_.stats.stepMilliseconds =
            state_.stats.stepMilliseconds == 0.0
                ? mean
                : state_.stats.stepMilliseconds * 0.85 + mean * 0.15;
    }
    if (performedSteps == maximumStepsPerFrame) {
        accumulator_ = 0.0;
        state_.notification =
            "Simulation could not keep pace; pending time was dropped to preserve UI response";
        state_.notificationIsError = true;
    }

    std::erase_if(trails_, [&](const auto& entry) {
        return simulation_.findBody(entry.first) == nullptr;
    });
    if (!state_.selectedBodyId ||
        simulation_.findBody(*state_.selectedBodyId) == nullptr) {
        state_.selectedBodyId.reset();
    }
    if (observedSceneRevision_ != state_.sceneRevision) {
        trails_.clear();
        prediction_.clear();
        state_.diagnosticSamples.clear();
        baselineEnergyValid_ = false;
        adaptiveController_.reset();
        state_.stats.adaptiveDecision.reset();
        observedSceneRevision_ = state_.sceneRevision;
    }
    updateDiagnostics(frameSeconds);
    if (state_.solverComparisonRequested) {
        runSolverComparison();
        state_.solverComparisonRequested = false;
    }
    if (state_.integratorComparisonRequested) {
        runIntegratorComparison();
        state_.integratorComparisonRequested = false;
    }
    if (state_.numericalValidationRequested) {
        state_.numericalValidation = runNumericalValidation();
        state_.numericalValidationRequested = false;
        state_.notification = state_.numericalValidation->passed
                                  ? "Numerical validation suite passed"
                                  : "Numerical validation reported a threshold failure";
        state_.notificationIsError = !state_.numericalValidation->passed;
    }
    if (state_.adaptiveExperimentRequested) {
        state_.adaptiveExperiment = runAdaptiveMethodExperiment();
        state_.adaptiveExperimentRequested = false;
        state_.notification =
            state_.adaptiveExperiment->hypothesisPassed
                ? "OrbitLab method experiment passed its stated hypothesis"
                : "OrbitLab method experiment falsified its stated hypothesis";
        state_.notificationIsError =
            !state_.adaptiveExperiment->hypothesisPassed;
    }
    updateReferenceFrame();
}

void Application::synchronizeGpuSolver() {
    if (activeGpuSolver_ != nullptr) {
        state_.gpuComputeStatus = activeGpuSolver_->status();
    }
    if (state_.gpuComputeEnabled == simulation_.hasGravitySolverOverride()) {
        return;
    }
    if (!state_.gpuComputeEnabled) {
        simulation_.clearGravitySolverOverride();
        activeGpuSolver_ = nullptr;
        state_.gpuComputeStatus = "CPU solver selected";
        return;
    }
    auto gpuSolver = std::make_unique<GpuGravitySolver>(gpuPlatform_->device());
    if (!gpuSolver->available()) {
        state_.gpuComputeEnabled = false;
        state_.gpuComputeAvailable = false;
        state_.gpuComputeStatus = gpuSolver->status();
        return;
    }
    state_.gpuComputeStatus =
        "Active; float GPU acceleration with CPU collision/integration stages";
    activeGpuSolver_ = gpuSolver.get();
    simulation_.setGravitySolverOverride(std::move(gpuSolver));
}

void Application::updateReferenceFrame() {
    switch (state_.referenceFrame) {
    case ReferenceFrame::Inertial:
        return;
    case ReferenceFrame::FollowSelected: {
        if (state_.selectedBodyId) {
            if (const Body* body = simulation_.findBody(*state_.selectedBodyId)) {
                camera_.focus(body->position);
            }
        }
        return;
    }
    case ReferenceFrame::Barycenter: {
        Vec3 weightedPosition{};
        double totalMass = 0.0;
        for (const auto& body : simulation_.bodies()) {
            weightedPosition += body.position * body.mass;
            totalMass += body.mass;
        }
        if (totalMass > 0.0) {
            camera_.focus(weightedPosition / totalMass);
        }
        return;
    }
    case ReferenceFrame::CoRotatingSelected: {
        if (!state_.selectedBodyId) {
            return;
        }
        const Body* selected = simulation_.findBody(*state_.selectedBodyId);
        const auto elements = simulation_.orbitalElements(*state_.selectedBodyId);
        if (selected == nullptr || !elements) {
            return;
        }
        const Body* primary = simulation_.findBody(elements->primaryBodyId);
        if (primary == nullptr) {
            return;
        }
        const double totalMass = selected->mass + primary->mass;
        camera_.focus(
            (selected->position * selected->mass + primary->position * primary->mass) /
            totalMass);
        const Vec3 axis = selected->position - primary->position;
        camera_.setAzimuth(std::atan2(axis.y, axis.x) - std::numbers::pi * 0.5);
        return;
    }
    }
}

void Application::updateDiagnostics(const double frameSeconds) {
    diagnosticsAccumulator_ += frameSeconds;
    const double refreshInterval = simulation_.bodies().size() > 1'000 ? 2.0 : 0.25;
    if (diagnosticsAccumulator_ < refreshInterval && baselineEnergyValid_) {
        return;
    }
    diagnosticsAccumulator_ = 0.0;
    state_.stats.diagnostics = simulation_.diagnostics();
    if (!baselineEnergyValid_) {
        baselineEnergy_ = state_.stats.diagnostics.totalEnergy;
        baselineEnergyValid_ = true;
    }
    const double denominator = std::abs(baselineEnergy_);
    state_.stats.energyDriftPercent =
        denominator > 1.0e-15
            ? (state_.stats.diagnostics.totalEnergy - baselineEnergy_) / denominator * 100.0
            : 0.0;
    state_.diagnosticSamples.push_back(
        {
            simulation_.elapsedTime(),
            static_cast<float>(state_.stats.energyDriftPercent),
            static_cast<float>(state_.stats.diagnostics.linearMomentum.length()),
            static_cast<float>(state_.stats.diagnostics.angularMomentum),
            static_cast<float>(state_.stats.stepMilliseconds),
            static_cast<float>(simulation_.bodies().size()),
        });
    constexpr std::size_t maximumSamples = 480;
    if (state_.diagnosticSamples.size() > maximumSamples) {
        state_.diagnosticSamples.erase(state_.diagnosticSamples.begin());
    }
}

void Application::updatePrediction() {
    if (!state_.paused || !state_.showPrediction || !state_.selectedBodyId) {
        prediction_.clear();
        predictedBodyId_.reset();
        return;
    }
    if (simulation_.bodies().size() > 512) {
        prediction_.clear();
        predictedBodyId_.reset();
        return;
    }
    const Body* selected = simulation_.findBody(*state_.selectedBodyId);
    if (selected == nullptr) {
        prediction_.clear();
        return;
    }
    const auto now = std::chrono::steady_clock::now();
    const bool unchanged =
        predictedBodyId_ == selected->id && predictedPosition_ == selected->position &&
        predictedVelocity_ == selected->velocity;
    if (unchanged ||
        now - lastPredictionUpdate_ < std::chrono::milliseconds{100}) {
        return;
    }

    Simulation projected;
    deserializeSimulation(projected, serializeSimulation(simulation_));
    projected.settings().collisionMode = CollisionMode::None;
    prediction_.clear();
    prediction_.reserve(241);
    prediction_.push_back(selected->position);
    const double predictionStep =
        std::min(projected.settings().fixedTimeStep * 8.0, 0.02);
    for (int index = 0; index < 240; ++index) {
        projected.step(predictionStep);
        const Body* projectedBody = projected.findBody(selected->id);
        if (projectedBody == nullptr) {
            break;
        }
        prediction_.push_back(projectedBody->position);
    }
    predictedBodyId_ = selected->id;
    predictedPosition_ = selected->position;
    predictedVelocity_ = selected->velocity;
    lastPredictionUpdate_ = now;
}

void Application::runSolverComparison() {
    const auto& bodies = simulation_.bodies();
    if (bodies.size() < 2) {
        state_.notification = "Add at least two bodies before comparing solvers";
        state_.notificationIsError = true;
        return;
    }
    if (bodies.size() > 4'096) {
        state_.notification =
            "Solver comparison is limited to 4,096 bodies to keep the interface responsive";
        state_.notificationIsError = true;
        return;
    }
    DirectGravitySolver direct;
    SoADirectGravitySolver soa;
    ThreadedSoAGravitySolver threaded;
    BarnesHutGravitySolver barnesHut(simulation_.settings().barnesHutOpeningAngle);
    const int repetitions = bodies.size() > 1'024 ? 1 : (bodies.size() > 256 ? 3 : 8);

    const auto measure = [&](const GravitySolver& solver) {
        std::vector<Vec3> result;
        const auto start = std::chrono::steady_clock::now();
        for (int index = 0; index < repetitions; ++index) {
            result = solver.accelerations(
                bodies,
                simulation_.settings().gravitationalConstant,
                simulation_.settings().softeningLength);
        }
        const double milliseconds =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start)
                .count() /
            repetitions;
        return std::pair{milliseconds, std::move(result)};
    };
    auto directMeasurement = measure(direct);
    const auto soaMeasurement = measure(soa);
    const auto threadedMeasurement = measure(threaded);
    auto barnesHutMeasurement = measure(barnesHut);
    state_.stats.directSolverMilliseconds = directMeasurement.first;
    state_.stats.soaSolverMilliseconds = soaMeasurement.first;
    state_.stats.threadedSolverMilliseconds = threadedMeasurement.first;
    state_.stats.barnesHutMilliseconds = barnesHutMeasurement.first;
    const auto& directResult = directMeasurement.second;
    const auto& barnesHutResult = barnesHutMeasurement.second;

    double errorSquared = 0.0;
    double referenceSquared = 0.0;
    for (std::size_t index = 0; index < bodies.size(); ++index) {
        errorSquared += (barnesHutResult[index] - directResult[index]).lengthSquared();
        referenceSquared += directResult[index].lengthSquared();
    }
    state_.stats.solverRelativeError =
        referenceSquared > 0.0 ? std::sqrt(errorSquared / referenceSquared) : 0.0;
    state_.stats.solverComparisonAvailable = true;
    state_.notification =
        "Compared four solver strategies on the current " + std::to_string(bodies.size()) +
        "-body state";
    state_.notificationIsError = false;
}

void Application::runIntegratorComparison() {
    const auto bodyCount = simulation_.bodies().size();
    if (bodyCount == 0) {
        state_.notification = "Add at least one body before comparing integrators";
        state_.notificationIsError = true;
        return;
    }
    if (bodyCount > 512) {
        state_.notification =
            "Integrator comparison is limited to 512 bodies to keep the interface responsive";
        state_.notificationIsError = true;
        return;
    }

    constexpr std::array integrators{
        IntegratorType::VelocityVerlet,
        IntegratorType::SymplecticEuler,
        IntegratorType::RungeKutta4,
        IntegratorType::Yoshida4,
    };
    const int steps = bodyCount > 128 ? 20 : (bodyCount > 32 ? 60 : 240);
    const std::string snapshot = serializeSimulation(simulation_);
    for (std::size_t index = 0; index < integrators.size(); ++index) {
        Simulation candidate;
        deserializeSimulation(candidate, snapshot);
        candidate.settings().integratorType = integrators[index];
        candidate.settings().collisionMode = CollisionMode::None;
        const double initialEnergy = candidate.diagnostics().totalEnergy;
        const auto start = std::chrono::steady_clock::now();
        for (int step = 0; step < steps; ++step) {
            candidate.step(candidate.settings().fixedTimeStep);
        }
        const double elapsed = std::chrono::duration<double, std::milli>(
                                   std::chrono::steady_clock::now() - start)
                                   .count();
        const double finalEnergy = candidate.diagnostics().totalEnergy;
        const double denominator = std::abs(initialEnergy);
        state_.integratorComparison[index] = {
            elapsed,
            denominator > 1.0e-15
                ? (finalEnergy - initialEnergy) / denominator * 100.0
                : 0.0,
        };
    }
    state_.integratorComparisonAvailable = true;
    state_.notification =
        "Compared four integrators over " + std::to_string(steps) +
        " fixed steps without changing the current scene";
    state_.notificationIsError = false;
}

void Application::recordTrails() {
    if (!simulation_.settings().trailsEnabled || ++trailSampleCounter_ % 8 != 0) {
        return;
    }
    constexpr std::size_t maximumTrailPoints = 900;
    for (const auto& body : simulation_.bodies()) {
        auto& trail = trails_[body.id];
        trail.push_back(body.position);
        if (trail.size() > maximumTrailPoints) {
            trail.pop_front();
        }
    }
}

std::optional<std::uint64_t> Application::bodyAt(const float x, const float y) const {
    int width = 0;
    int height = 0;
    SDL_GetWindowSize(window_, &width, &height);
    const double canvasWidth = static_cast<double>(width) - UserInterface::panelWidth;
    const Vec2 clicked{x - UserInterface::panelWidth, y};
    double closestDistanceSquared = 18.0 * 18.0;
    double closestDepth = std::numeric_limits<double>::max();
    std::optional<std::uint64_t> closest;
    for (const auto& body : simulation_.bodies()) {
        const auto projection =
            camera_.project(body.position, canvasWidth, static_cast<double>(height));
        if (!projection) {
            continue;
        }
        const double pickRadius =
            std::clamp(body.radius * projection->pixelsPerUnit + 5.0, 9.0, 42.0);
        const double distanceSquared =
            (projection->screen - clicked).lengthSquared();
        if (distanceSquared <= pickRadius * pickRadius &&
            (distanceSquared < closestDistanceSquared ||
             projection->depth < closestDepth)) {
            closestDistanceSquared = distanceSquared;
            closestDepth = projection->depth;
            closest = body.id;
        }
    }
    return closest;
}

void Application::beginBodyDrag(
    const std::uint64_t bodyId,
    const float x,
    const float y) {
    Body* body = simulation_.findBody(bodyId);
    if (body == nullptr) {
        return;
    }
    int width = 0;
    int height = 0;
    SDL_GetWindowSize(window_, &width, &height);
    const Vec3 pointerWorld = camera_.screenToWorldOnPlane(
        {x - UserInterface::panelWidth, y},
        body->position,
        static_cast<double>(width) - UserInterface::panelWidth,
        static_cast<double>(height));
    dragOffset_ = body->position - pointerWorld;
    draggedBodyId_ = bodyId;
    history_.commit(simulation_);
    pausedBeforeDrag_ = state_.paused;
    state_.paused = true;
    trails_.erase(bodyId);
    SDL_SetCursor(moveCursor_);
}

void Application::updateBodyDrag(const float x, const float y) {
    Body* body = draggedBodyId_ ? simulation_.findBody(*draggedBodyId_) : nullptr;
    if (body == nullptr) {
        draggedBodyId_.reset();
        return;
    }
    int width = 0;
    int height = 0;
    SDL_GetWindowSize(window_, &width, &height);
    const double canvasWidth = static_cast<double>(width) - UserInterface::panelWidth;
    const Vec2 clampedScreen{
        std::clamp(
            static_cast<double>(x - UserInterface::panelWidth),
            0.0,
            std::max(0.0, canvasWidth)),
        std::clamp(static_cast<double>(y), 0.0, static_cast<double>(height)),
    };
    body->position = camera_.screenToWorldOnPlane(
                         clampedScreen,
                         body->position,
                         canvasWidth,
                         static_cast<double>(height)) +
                     dragOffset_;
}

void Application::endBodyDrag() {
    if (!draggedBodyId_) {
        return;
    }
    const Body* body = simulation_.findBody(*draggedBodyId_);
    state_.paused = pausedBeforeDrag_;
    if (body != nullptr) {
        state_.notification = "Moved " + body->name + "; velocity preserved";
        state_.notificationIsError = false;
    }
    history_.commit(simulation_);
    ++state_.sceneRevision;
    draggedBodyId_.reset();
    SDL_SetCursor(defaultCursor_);
}

bool Application::velocityHandleAt(const float x, const float y) const {
    if (!state_.selectedBodyId) {
        return false;
    }
    const Body* body = simulation_.findBody(*state_.selectedBodyId);
    if (body == nullptr) {
        return false;
    }
    int width = 0;
    int height = 0;
    SDL_GetWindowSize(window_, &width, &height);
    const double canvasWidth = static_cast<double>(width) - UserInterface::panelWidth;
    Vec2 handle = camera_.worldToScreen(
        body->position + velocityHandleOffset(*body, camera_),
        canvasWidth,
        static_cast<double>(height));
    handle.x += UserInterface::panelWidth;
    return (handle - Vec2{x, y}).lengthSquared() <= 12.0 * 12.0;
}

void Application::beginVelocityDrag() {
    if (!state_.selectedBodyId || simulation_.findBody(*state_.selectedBodyId) == nullptr) {
        return;
    }
    history_.commit(simulation_);
    pausedBeforeDrag_ = state_.paused;
    state_.paused = true;
    draggingVelocity_ = true;
    SDL_SetCursor(crosshairCursor_);
}

void Application::updateVelocityDrag(const float x, const float y) {
    Body* body = state_.selectedBodyId ? simulation_.findBody(*state_.selectedBodyId) : nullptr;
    if (body == nullptr) {
        draggingVelocity_ = false;
        return;
    }
    int width = 0;
    int height = 0;
    SDL_GetWindowSize(window_, &width, &height);
    const double canvasWidth = static_cast<double>(width) - UserInterface::panelWidth;
    const Vec3 pointerWorld = camera_.screenToWorldOnPlane(
        {
            std::clamp(
                static_cast<double>(x - UserInterface::panelWidth),
                0.0,
                std::max(0.0, canvasWidth)),
            std::clamp(static_cast<double>(y), 0.0, static_cast<double>(height)),
        },
        body->position,
        canvasWidth,
        static_cast<double>(height));
    body->velocity = (pointerWorld - body->position) / velocityHandleScale;
}

void Application::endVelocityDrag() {
    Body* body = state_.selectedBodyId ? simulation_.findBody(*state_.selectedBodyId) : nullptr;
    state_.paused = pausedBeforeDrag_;
    draggingVelocity_ = false;
    history_.commit(simulation_);
    ++state_.sceneRevision;
    if (body != nullptr) {
        state_.notification = "Updated " + body->name + "'s velocity vector";
        state_.notificationIsError = false;
    }
    SDL_SetCursor(defaultCursor_);
}

void Application::updateHover(const float x, const float y) {
    if (x <= UserInterface::panelWidth) {
        state_.hoveredBodyId.reset();
        SDL_SetCursor(defaultCursor_);
        return;
    }
    if (velocityHandleAt(x, y)) {
        state_.hoveredBodyId = state_.selectedBodyId;
        SDL_SetCursor(crosshairCursor_);
        return;
    }
    state_.hoveredBodyId = bodyAt(x, y);
    SDL_SetCursor(state_.hoveredBodyId ? pointerCursor_ : defaultCursor_);
}

void Application::beginBodyCreation(const float x, const float y) {
    int width = 0;
    int height = 0;
    SDL_GetWindowSize(window_, &width, &height);
    state_.creationStart = camera_.screenToWorld(
        {x - UserInterface::panelWidth, y},
        static_cast<double>(width) - UserInterface::panelWidth,
        static_cast<double>(height));
    state_.creationCurrent = state_.creationStart;
    state_.creatingBody = true;
    pausedBeforeDrag_ = state_.paused;
    state_.paused = true;
    history_.commit(simulation_);
    SDL_SetCursor(crosshairCursor_);
}

void Application::updateBodyCreation(const float x, const float y) {
    int width = 0;
    int height = 0;
    SDL_GetWindowSize(window_, &width, &height);
    const double canvasWidth = static_cast<double>(width) - UserInterface::panelWidth;
    state_.creationCurrent = camera_.screenToWorld(
        {
            std::clamp(
                static_cast<double>(x - UserInterface::panelWidth),
                0.0,
                std::max(0.0, canvasWidth)),
            std::clamp(static_cast<double>(y), 0.0, static_cast<double>(height)),
        },
        canvasWidth,
        static_cast<double>(height));
}

void Application::endBodyCreation() {
    const Vec3 gesture = state_.creationCurrent - state_.creationStart;
    const bool longEnough =
        gesture.length() * camera_.pixelsPerUnit() >= 7.0;
    state_.creatingBody = false;
    state_.paused = pausedBeforeDrag_;
    if (longEnough) {
        const auto number = simulation_.bodies().size() + 1;
        Body& body = simulation_.addBody(
            {0,
             "Body " + std::to_string(number),
             1.0e-4,
             0.018,
             {0.53F, 0.84F, 0.67F, 1.0F},
             state_.creationStart,
             gesture / velocityHandleScale});
        state_.selectedBodyId = body.id;
        history_.commit(simulation_);
        ++state_.sceneRevision;
        state_.notification =
            "Created " + body.name + " with velocity from the drag gesture";
        state_.notificationIsError = false;
    }
    SDL_SetCursor(defaultCursor_);
}

void Application::markSceneChanged() {
    ++state_.sceneRevision;
    state_.selectedBodyId.reset();
    state_.hoveredBodyId.reset();
}

} // namespace orbitlab::app
