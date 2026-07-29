#pragma once

#include "app/AppState.hpp"
#include "app/Camera.hpp"
#include "app/History.hpp"
#include "orbitlab/Simulation.hpp"

namespace orbitlab::app {

class UserInterface {
public:
    static constexpr float panelWidth = 368.0F;

    void applyStyle();
    void draw(
        Simulation& simulation,
        Camera& camera,
        History& history,
        AppState& state,
        int windowHeight);

private:
    void drawTransport(Simulation& simulation, History& history, AppState& state);
    void drawScenario(
        Simulation& simulation, Camera& camera, History& history, AppState& state);
    void drawBodyList(Simulation& simulation, Camera& camera, AppState& state);
    void drawInspector(Simulation& simulation, History& history, AppState& state);
    void drawDisplay(
        Simulation& simulation, Camera& camera, History& history, AppState& state);
    void drawPersistence(
        Simulation& simulation, Camera& camera, History& history, AppState& state);
    void drawCollisionLog(const Simulation& simulation);
    void drawDiagnosticPlots(const AppState& state);
    void drawNumericalValidation(AppState& state);
    void drawAdaptiveMethod(
        Simulation& simulation,
        History& history,
        AppState& state);
    void setNotification(AppState& state, std::string message, bool error);
};

} // namespace orbitlab::app
