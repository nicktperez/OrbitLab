#include "orbitlab/AdaptiveExperiment.hpp"
#include "orbitlab/AdaptiveFidelity.hpp"
#include "orbitlab/GravitySolver.hpp"
#include "orbitlab/Persistence.hpp"
#include "orbitlab/Replay.hpp"
#include "orbitlab/Validation.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

std::string readText(const std::filesystem::path& path) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) {
        throw std::runtime_error("Could not open '" + path.string() + "'");
    }
    stream.seekg(0, std::ios::end);
    const auto size = stream.tellg();
    constexpr std::streamoff maximumSize = 64 * 1024 * 1024;
    if (size < 0 || size > maximumSize) {
        throw std::runtime_error("Input file exceeds the 64 MiB safety limit");
    }
    stream.seekg(0);
    return {
        std::istreambuf_iterator<char>{stream},
        std::istreambuf_iterator<char>{},
    };
}

void writeText(const std::filesystem::path& path, const std::string& text) {
    std::ofstream stream(path, std::ios::binary);
    if (!stream) {
        throw std::runtime_error("Could not open '" + path.string() + "' for writing");
    }
    stream << text << '\n';
    if (!stream) {
        throw std::runtime_error("Could not finish writing '" + path.string() + "'");
    }
}

std::uint64_t parseCount(const std::string_view text) {
    std::size_t consumed = 0;
    const auto value = std::stoull(std::string{text}, &consumed);
    if (consumed != text.size() || value > 100'000'000) {
        throw std::invalid_argument("Step count must be an integer no greater than 100,000,000");
    }
    return value;
}

double parsePositive(const std::string_view text, const char* name) {
    std::size_t consumed = 0;
    const double value = std::stod(std::string{text}, &consumed);
    if (consumed != text.size() || !std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(std::string{name} + " must be finite and positive");
    }
    return value;
}

orbitlab::Simulation loadSimulationFile(const std::filesystem::path& path) {
    orbitlab::Simulation simulation;
    orbitlab::deserializeSimulation(simulation, readText(path));
    return simulation;
}

void printSummary(const orbitlab::Simulation& simulation) {
    const auto diagnostics = simulation.diagnostics();
    std::cout << "bodies=" << simulation.bodies().size()
              << " time=" << std::setprecision(12) << simulation.elapsedTime()
              << " energy=" << diagnostics.totalEnergy
              << " momentum=" << diagnostics.linearMomentum.length()
              << " hash=" << orbitlab::simulationStateHashHex(simulation) << '\n';
}

void printUsage() {
    std::cout
        << "OrbitLab headless simulation tools\n\n"
        << "Usage:\n"
        << "  orbitlab_cli validate <simulation.json>\n"
        << "  orbitlab_cli hash <simulation.json>\n"
        << "  orbitlab_cli run <simulation.json> --steps N [--dt T] --output result.json\n"
        << "      Add --adaptive to use the saved OrbitLab method settings.\n"
        << "  orbitlab_cli record <simulation.json> --steps N [--dt T] --output run.orbit\n"
        << "  orbitlab_cli replay <run.orbit> [--output result.json]\n"
        << "  orbitlab_cli compare <simulation.json>\n"
        << "  orbitlab_cli validate-numerics [--output report.json]\n"
        << "  orbitlab_cli validate-method [--output report.json]\n";
}

struct RunOptions {
    std::uint64_t steps{0};
    double deltaTime{0.0};
    std::filesystem::path output;
    bool adaptive{false};
};

RunOptions parseRunOptions(
    const int argc,
    char** argv,
    const double defaultDelta,
    const bool outputRequired) {
    RunOptions options;
    options.deltaTime = defaultDelta;
    for (int index = 3; index < argc; ++index) {
        const std::string_view argument = argv[index];
        if (argument == "--steps" && index + 1 < argc) {
            options.steps = parseCount(argv[++index]);
        } else if (argument == "--dt" && index + 1 < argc) {
            options.deltaTime = parsePositive(argv[++index], "Time step");
        } else if (argument == "--output" && index + 1 < argc) {
            options.output = argv[++index];
        } else if (argument == "--adaptive") {
            options.adaptive = true;
        } else {
            throw std::invalid_argument("Unknown or incomplete option: " + std::string{argument});
        }
    }
    if (options.steps == 0) {
        throw std::invalid_argument("--steps must be greater than zero");
    }
    if (outputRequired && options.output.empty()) {
        throw std::invalid_argument("--output is required");
    }
    return options;
}

void compareSolvers(const orbitlab::Simulation& simulation) {
    const orbitlab::DirectGravitySolver direct;
    const orbitlab::SoADirectGravitySolver soa;
    const orbitlab::ThreadedSoAGravitySolver threaded;
    const orbitlab::BarnesHutGravitySolver barnesHut{
        simulation.settings().barnesHutOpeningAngle};
    const std::vector<std::pair<std::string_view, const orbitlab::GravitySolver*>> solvers{
        {"direct", &direct},
        {"soa-direct", &soa},
        {"threaded-soa", &threaded},
        {"barnes-hut", &barnesHut},
    };
    const auto reference = direct.accelerations(
        simulation.bodies(),
        simulation.settings().gravitationalConstant,
        simulation.settings().softeningLength);
    std::cout << std::left << std::setw(18) << "solver" << std::right
              << std::setw(14) << "milliseconds" << std::setw(16)
              << "RMS error (%)" << '\n';
    for (const auto& [name, solver] : solvers) {
        const auto start = std::chrono::steady_clock::now();
        const auto result = solver->accelerations(
            simulation.bodies(),
            simulation.settings().gravitationalConstant,
            simulation.settings().softeningLength);
        const double milliseconds =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start)
                .count();
        double errorSquared = 0.0;
        double referenceSquared = 0.0;
        for (std::size_t index = 0; index < result.size(); ++index) {
            errorSquared += (result[index] - reference[index]).lengthSquared();
            referenceSquared += reference[index].lengthSquared();
        }
        const double error = referenceSquared > 0.0
                                 ? std::sqrt(errorSquared / referenceSquared) * 100.0
                                 : 0.0;
        std::cout << std::left << std::setw(18) << name << std::right
                  << std::fixed << std::setprecision(5) << std::setw(14)
                  << milliseconds << std::setw(16) << error << '\n';
    }
}

} // namespace

int main(const int argc, char** argv) {
    try {
        if (argc >= 2 && std::string_view{argv[1]} == "validate-method") {
            std::filesystem::path output;
            if (argc == 4 && std::string_view{argv[2]} == "--output") {
                output = argv[3];
            } else if (argc != 2) {
                throw std::invalid_argument(
                    "validate-method accepts only an optional --output path");
            }
            const auto report = orbitlab::runAdaptiveMethodExperiment();
            const std::string json =
                orbitlab::serializeAdaptiveMethodReport(report);
            if (!output.empty()) {
                writeText(output, json);
            }
            std::cout << json << '\n';
            return report.hypothesisPassed ? 0 : 3;
        }
        if (argc >= 2 && std::string_view{argv[1]} == "validate-numerics") {
            std::filesystem::path output;
            if (argc == 4 && std::string_view{argv[2]} == "--output") {
                output = argv[3];
            } else if (argc != 2) {
                throw std::invalid_argument(
                    "validate-numerics accepts only an optional --output path");
            }
            const auto report = orbitlab::runNumericalValidation();
            const std::string json = orbitlab::serializeValidationReport(report);
            if (!output.empty()) {
                writeText(output, json);
            }
            std::cout << json << '\n';
            return report.passed ? 0 : 3;
        }
        if (argc < 3) {
            printUsage();
            return argc == 1 ? 0 : 2;
        }
        const std::string_view command = argv[1];
        const std::filesystem::path input = argv[2];

        if (command == "validate" || command == "hash") {
            const auto simulation = loadSimulationFile(input);
            if (command == "hash") {
                std::cout << orbitlab::simulationStateHashHex(simulation) << '\n';
            } else {
                printSummary(simulation);
            }
            return 0;
        }
        if (command == "compare") {
            compareSolvers(loadSimulationFile(input));
            return 0;
        }
        if (command == "run" || command == "record") {
            auto simulation = loadSimulationFile(input);
            const auto options = parseRunOptions(
                argc, argv, simulation.settings().fixedTimeStep, true);
            if (command == "record") {
                if (options.adaptive) {
                    throw std::invalid_argument(
                        "Adaptive runs cannot be encoded as one replay step command; "
                        "use run --adaptive and save the resulting simulation");
                }
                orbitlab::ReplayRecording recording{
                    orbitlab::serializeSimulation(simulation),
                    {{
                        orbitlab::ReplayCommandType::Step,
                        0,
                        options.steps,
                        options.deltaTime,
                    }},
                };
                writeText(options.output, orbitlab::serializeReplay(recording));
                std::cout << "recorded_steps=" << options.steps
                          << " output=" << options.output.string() << '\n';
                return 0;
            }
            orbitlab::AdaptiveFidelityController adaptiveController;
            if (options.adaptive) {
                simulation.settings().adaptiveFidelity.enabled = true;
                simulation.settings().integratorType =
                    orbitlab::IntegratorType::RungeKutta4;
                simulation.settings().adaptiveFidelity.maximumTimeStep =
                    options.deltaTime;
                simulation.settings().adaptiveFidelity.minimumTimeStep =
                    std::min(
                        simulation.settings().adaptiveFidelity.minimumTimeStep,
                        options.deltaTime);
            }
            for (std::uint64_t step = 0; step < options.steps; ++step) {
                if (options.adaptive) {
                    auto decision = adaptiveController.propose(
                        simulation.bodies(),
                        simulation.settings().gravitationalConstant,
                        simulation.settings().softeningLength,
                        simulation.settings().adaptiveFidelity);
                    simulation.step(decision.timeStep);
                    adaptiveController.commit(decision);
                } else {
                    simulation.step(options.deltaTime);
                }
            }
            writeText(options.output, orbitlab::serializeSimulation(simulation));
            printSummary(simulation);
            return 0;
        }
        if (command == "replay") {
            std::filesystem::path output;
            if (argc == 5 && std::string_view{argv[3]} == "--output") {
                output = argv[4];
            } else if (argc != 3) {
                throw std::invalid_argument("Replay accepts only an optional --output path");
            }
            const auto recording = orbitlab::deserializeReplay(readText(input));
            const auto simulation = orbitlab::replaySimulation(recording);
            if (!output.empty()) {
                writeText(output, orbitlab::serializeSimulation(simulation));
            }
            printSummary(simulation);
            return 0;
        }

        throw std::invalid_argument("Unknown command: " + std::string{command});
    } catch (const std::exception& exception) {
        std::cerr << "orbitlab_cli: " << exception.what() << '\n';
        return 1;
    }
}
