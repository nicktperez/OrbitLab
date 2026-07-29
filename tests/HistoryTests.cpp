#include "app/History.hpp"

#include "orbitlab/Presets.hpp"

#include <catch2/catch_test_macros.hpp>

using namespace orbitlab;
using namespace orbitlab::app;

TEST_CASE("History supports undo redo and initial restoration", "[history]") {
    Simulation simulation;
    loadPreset(simulation, Preset::SunEarth);
    History history;
    history.reset(simulation);

    simulation.bodies()[1].position = {2.0, 0.0};
    history.commit(simulation);
    simulation.bodies()[1].position = {3.0, 0.0};
    history.commit(simulation);

    REQUIRE(history.canUndo());
    REQUIRE(history.undo(simulation));
    REQUIRE(simulation.bodies()[1].position.x == 2.0);
    REQUIRE(history.redo(simulation));
    REQUIRE(simulation.bodies()[1].position.x == 3.0);
    REQUIRE(history.restoreInitial(simulation));
    REQUIRE(simulation.bodies()[1].position.x == 1.0);
}

TEST_CASE("Committing after undo discards the redo branch", "[history]") {
    Simulation simulation;
    loadPreset(simulation, Preset::SunEarth);
    History history;
    history.reset(simulation);
    simulation.bodies()[1].mass = 4.0;
    history.commit(simulation);
    REQUIRE(history.undo(simulation));

    simulation.bodies()[1].mass = 5.0;
    history.commit(simulation);
    REQUIRE_FALSE(history.canRedo());
}
