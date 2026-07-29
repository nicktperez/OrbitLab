#include "orbitlab/AdaptiveExperiment.hpp"
#include "orbitlab/AdaptiveFidelity.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>

#include <cmath>
#include <vector>

using namespace orbitlab;

TEST_CASE("OrbitLab method reduces its step for an approaching encounter", "[adaptive]") {
    const std::vector<Body> bodies{
        {1, "Left", 1.0, 0.01, {}, {-0.5, 0.0, 0.0}, {20.0, 0.0, 0.0}},
        {2, "Right", 1.0, 0.01, {}, {0.5, 0.0, 0.0}, {-20.0, 0.0, 0.0}},
    };
    AdaptiveFidelitySettings settings;
    settings.safetyFactor = 0.5;
    settings.jerkWeight = 0.0;
    settings.encounterWeight = 1.0;
    settings.minimumTimeStep = 0.0001;
    settings.maximumTimeStep = 0.1;

    const auto decision =
        AdaptiveFidelityController{}.propose(bodies, 1.0, 0.0, settings);

    CHECK(decision.timeStep < settings.maximumTimeStep);
    CHECK(decision.reason == AdaptiveLimitReason::Encounter);
    CHECK(decision.encounterTimescale == Catch::Approx(0.025));
}

TEST_CASE("OrbitLab method emits bounded deterministic dyadic steps", "[adaptive]") {
    const std::vector<Body> bodies{
        {1, "Primary", 1.0, 0.01, {}, {}, {}},
        {2, "Orbiter", 0.001, 0.001, {}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}},
    };
    AdaptiveFidelitySettings settings;
    settings.minimumTimeStep = 0.000125;
    settings.maximumTimeStep = 0.008;
    AdaptiveFidelityController controller;
    const auto first = controller.propose(bodies, 1.0, 0.0, settings);
    const auto second = controller.propose(bodies, 1.0, 0.0, settings);

    CHECK(first.timeStep == second.timeStep);
    CHECK(first.timeStep >= settings.minimumTimeStep);
    CHECK(first.timeStep <= settings.maximumTimeStep);
    const double dyadicLevel =
        std::log2(settings.maximumTimeStep / first.timeStep);
    CHECK(dyadicLevel == Catch::Approx(std::round(dyadicLevel)).margin(1.0e-12));
}

TEST_CASE("OrbitLab adaptive experiment is reproducible and falsifiable", "[adaptive][validation]") {
    const auto report = runAdaptiveMethodExperiment();
    const auto json =
        nlohmann::json::parse(serializeAdaptiveMethodReport(report));

    CHECK(json.at("format") == "OrbitLabAdaptiveMethodExperiment");
    CHECK(report.hypothesisPassed);
    CHECK(report.adaptive.finalPositionError <
          report.coarseFixed.finalPositionError);
    CHECK(std::abs(report.adaptive.energyDriftPercent) <
          std::abs(report.coarseFixed.energyDriftPercent));
    CHECK(report.adaptive.steps < report.fineFixed.steps);
    CHECK(report.adaptive.minimumStepUsed < report.adaptive.maximumStepUsed);
}
