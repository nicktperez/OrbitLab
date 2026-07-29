#include "orbitlab/Replay.hpp"

#include "orbitlab/Persistence.hpp"

#include <array>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <sstream>
#include <stdexcept>

namespace orbitlab {
namespace {

using Json = nlohmann::json;

const char* commandName(const ReplayCommandType type) {
    switch (type) {
    case ReplayCommandType::Step:
        return "step";
    case ReplayCommandType::RemoveBody:
        return "remove-body";
    case ReplayCommandType::SetBodyState:
        return "set-body-state";
    case ReplayCommandType::SetSolver:
        return "set-solver";
    case ReplayCommandType::SetIntegrator:
        return "set-integrator";
    }
    return "step";
}

const char* solverName(const SolverType type) {
    switch (type) {
    case SolverType::Direct:
        return "direct";
    case SolverType::SoADirect:
        return "soa-direct";
    case SolverType::ThreadedSoA:
        return "threaded-soa";
    case SolverType::BarnesHut:
        return "barnes-hut";
    }
    return "direct";
}

const char* integratorName(const IntegratorType type) {
    switch (type) {
    case IntegratorType::VelocityVerlet:
        return "velocity-verlet";
    case IntegratorType::SymplecticEuler:
        return "symplectic-euler";
    case IntegratorType::RungeKutta4:
        return "rk4";
    case IntegratorType::Yoshida4:
        return "yoshida4";
    }
    return "velocity-verlet";
}

SolverType parseSolver(const std::string& name) {
    if (name == "direct") {
        return SolverType::Direct;
    }
    if (name == "soa-direct") {
        return SolverType::SoADirect;
    }
    if (name == "threaded-soa") {
        return SolverType::ThreadedSoA;
    }
    if (name == "barnes-hut") {
        return SolverType::BarnesHut;
    }
    throw std::invalid_argument("Unknown replay solver: " + name);
}

IntegratorType parseIntegrator(const std::string& name) {
    if (name == "velocity-verlet") {
        return IntegratorType::VelocityVerlet;
    }
    if (name == "symplectic-euler") {
        return IntegratorType::SymplecticEuler;
    }
    if (name == "rk4") {
        return IntegratorType::RungeKutta4;
    }
    if (name == "yoshida4") {
        return IntegratorType::Yoshida4;
    }
    throw std::invalid_argument("Unknown replay integrator: " + name);
}

Vec3 parseVec3(const Json& json, const char* field) {
    const auto values = json.at(field).get<std::array<double, 3>>();
    const Vec3 result{values[0], values[1], values[2]};
    if (!result.isFinite()) {
        throw std::invalid_argument(std::string{"Replay "} + field + " must be finite");
    }
    return result;
}

} // namespace

std::uint64_t simulationStateHash(const Simulation& simulation) {
    constexpr std::uint64_t offsetBasis = 14695981039346656037ULL;
    constexpr std::uint64_t prime = 1099511628211ULL;
    std::uint64_t hash = offsetBasis;
    for (const char byte : serializeSimulation(simulation)) {
        hash ^= static_cast<unsigned char>(byte);
        hash *= prime;
    }
    return hash;
}

std::string simulationStateHashHex(const Simulation& simulation) {
    std::ostringstream stream;
    stream << std::hex << std::setfill('0') << std::setw(16)
           << simulationStateHash(simulation);
    return stream.str();
}

std::string serializeReplay(const ReplayRecording& recording) {
    Json commands = Json::array();
    for (const auto& command : recording.commands) {
        Json item{{"type", commandName(command.type)}};
        switch (command.type) {
        case ReplayCommandType::Step:
            item["count"] = command.stepCount;
            item["deltaTime"] = command.deltaTime;
            break;
        case ReplayCommandType::RemoveBody:
            item["bodyId"] = command.bodyId;
            break;
        case ReplayCommandType::SetBodyState:
            item["bodyId"] = command.bodyId;
            item["position"] =
                Json::array({command.position.x, command.position.y, command.position.z});
            item["velocity"] =
                Json::array({command.velocity.x, command.velocity.y, command.velocity.z});
            break;
        case ReplayCommandType::SetSolver:
            item["solver"] = solverName(command.solverType);
            break;
        case ReplayCommandType::SetIntegrator:
            item["integrator"] = integratorName(command.integratorType);
            break;
        }
        commands.push_back(std::move(item));
    }
    const Json replay{
        {"format", "OrbitLabReplay"},
        {"version", 1},
        {"initialState", Json::parse(recording.initialSimulationJson)},
        {"commands", std::move(commands)},
    };
    return replay.dump(2);
}

ReplayRecording deserializeReplay(const std::string& jsonText) {
    const Json replay = Json::parse(jsonText);
    if (replay.value("format", "") != "OrbitLabReplay" ||
        replay.value("version", 0) != 1) {
        throw std::invalid_argument("File is not a supported OrbitLab replay");
    }
    const auto& commandJson = replay.at("commands");
    if (!commandJson.is_array() || commandJson.size() > 1'000'000) {
        throw std::invalid_argument("Replay commands must be an array with at most 1,000,000 entries");
    }

    ReplayRecording recording;
    recording.initialSimulationJson = replay.at("initialState").dump();
    for (const auto& item : commandJson) {
        const std::string type = item.at("type").get<std::string>();
        ReplayCommand command;
        if (type == "step") {
            command.type = ReplayCommandType::Step;
            command.stepCount = item.at("count").get<std::uint64_t>();
            command.deltaTime = item.at("deltaTime").get<double>();
            if (command.stepCount > 100'000'000 || !std::isfinite(command.deltaTime) ||
                command.deltaTime <= 0.0) {
                throw std::invalid_argument("Replay step command exceeds safe limits");
            }
        } else if (type == "remove-body") {
            command.type = ReplayCommandType::RemoveBody;
            command.bodyId = item.at("bodyId").get<std::uint64_t>();
        } else if (type == "set-body-state") {
            command.type = ReplayCommandType::SetBodyState;
            command.bodyId = item.at("bodyId").get<std::uint64_t>();
            command.position = parseVec3(item, "position");
            command.velocity = parseVec3(item, "velocity");
        } else if (type == "set-solver") {
            command.type = ReplayCommandType::SetSolver;
            command.solverType = parseSolver(item.at("solver").get<std::string>());
        } else if (type == "set-integrator") {
            command.type = ReplayCommandType::SetIntegrator;
            command.integratorType =
                parseIntegrator(item.at("integrator").get<std::string>());
        } else {
            throw std::invalid_argument("Unknown replay command: " + type);
        }
        recording.commands.push_back(command);
    }
    Simulation validation;
    deserializeSimulation(validation, recording.initialSimulationJson);
    return recording;
}

Simulation replaySimulation(const ReplayRecording& recording) {
    Simulation simulation;
    deserializeSimulation(simulation, recording.initialSimulationJson);
    for (const auto& command : recording.commands) {
        switch (command.type) {
        case ReplayCommandType::Step:
            for (std::uint64_t step = 0; step < command.stepCount; ++step) {
                simulation.step(command.deltaTime);
            }
            break;
        case ReplayCommandType::RemoveBody:
            if (!simulation.removeBody(command.bodyId)) {
                throw std::invalid_argument("Replay references a missing body for removal");
            }
            break;
        case ReplayCommandType::SetBodyState: {
            Body* body = simulation.findBody(command.bodyId);
            if (body == nullptr) {
                throw std::invalid_argument("Replay references a missing body for editing");
            }
            body->position = command.position;
            body->velocity = command.velocity;
            break;
        }
        case ReplayCommandType::SetSolver:
            simulation.settings().solverType = command.solverType;
            break;
        case ReplayCommandType::SetIntegrator:
            simulation.settings().integratorType = command.integratorType;
            break;
        }
    }
    return simulation;
}

} // namespace orbitlab
