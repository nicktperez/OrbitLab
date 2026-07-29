#include "orbitlab/Validation.hpp"

#include <catch2/catch_test_macros.hpp>

using namespace orbitlab;

TEST_CASE("Numerical validation suite meets deterministic thresholds", "[validation]") {
    const auto report = runNumericalValidation();

    REQUIRE(report.passed);
    REQUIRE(report.centerOfMassDrift < 1.0e-9);
    REQUIRE(report.reversibilityError < 1.0e-8);
    REQUIRE(report.integratorSamples.size() == 16);
    REQUIRE(report.barnesHutSamples.size() == 4);
    for (std::size_t integrator = 0; integrator < 4; ++integrator) {
        const std::size_t first = integrator * 4;
        REQUIRE(report.integratorSamples[first + 3].positionError <
                report.integratorSamples[first].positionError);
    }
    REQUIRE(report.barnesHutSamples.front().relativeRmsError <
            report.barnesHutSamples.back().relativeRmsError);
}

TEST_CASE("Numerical validation report is machine readable", "[validation]") {
    const auto json = serializeValidationReport(runNumericalValidation());
    REQUIRE(json.find("\"passed\": true") != std::string::npos);
    REQUIRE(json.find("\"integrators\"") != std::string::npos);
    REQUIRE(json.find("\"barnesHut\"") != std::string::npos);
}
