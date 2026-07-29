#pragma once

#include "app/AppState.hpp"
#include "app/Camera.hpp"
#include "app/History.hpp"
#include "app/GpuPlatform.hpp"
#include "app/Renderer.hpp"
#include "app/UserInterface.hpp"
#include "orbitlab/Simulation.hpp"

#include <SDL3/SDL.h>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>

namespace orbitlab::app {

class GpuGravitySolver;

class Application {
public:
    Application();
    ~Application();

    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    int run();

private:
    void processEvent(const SDL_Event& event);
    void update(double frameSeconds);
    void updateDiagnostics(double frameSeconds);
    void updateReferenceFrame();
    void updatePrediction();
    void runSolverComparison();
    void runIntegratorComparison();
    void synchronizeGpuSolver();
    void recordTrails();
    [[nodiscard]] std::optional<std::uint64_t> bodyAt(float x, float y) const;
    void beginBodyDrag(std::uint64_t bodyId, float x, float y);
    void updateBodyDrag(float x, float y);
    void endBodyDrag();
    [[nodiscard]] bool velocityHandleAt(float x, float y) const;
    void beginVelocityDrag();
    void updateVelocityDrag(float x, float y);
    void endVelocityDrag();
    void updateHover(float x, float y);
    void beginBodyCreation(float x, float y);
    void updateBodyCreation(float x, float y);
    void endBodyCreation();
    void markSceneChanged();

    SDL_Window* window_{nullptr};
    std::unique_ptr<GpuPlatform> gpuPlatform_;
    SDL_Renderer* sdlRenderer_{nullptr};
    GpuGravitySolver* activeGpuSolver_{nullptr};
    std::unique_ptr<Renderer> renderer_;
    Simulation simulation_;
    AdaptiveFidelityController adaptiveController_;
    Camera camera_;
    UserInterface userInterface_;
    History history_;
    AppState state_;
    TrailMap trails_;
    PredictionPath prediction_;
    bool running_{true};
    bool panning_{false};
    bool orbitingCamera_{false};
    bool draggingVelocity_{false};
    std::optional<std::uint64_t> draggedBodyId_;
    Vec3 dragOffset_{};
    bool pausedBeforeDrag_{false};
    SDL_Cursor* defaultCursor_{nullptr};
    SDL_Cursor* pointerCursor_{nullptr};
    SDL_Cursor* moveCursor_{nullptr};
    SDL_Cursor* crosshairCursor_{nullptr};
    Vec2 previousMouse_{};
    double accumulator_{0.0};
    std::uint64_t trailSampleCounter_{0};
    std::uint64_t observedSceneRevision_{0};
    double diagnosticsAccumulator_{0.0};
    double baselineEnergy_{0.0};
    bool baselineEnergyValid_{false};
    bool adaptiveWasEnabled_{false};
    std::chrono::steady_clock::time_point lastPredictionUpdate_{};
    std::optional<std::uint64_t> predictedBodyId_;
    Vec3 predictedPosition_{};
    Vec3 predictedVelocity_{};
};

} // namespace orbitlab::app
