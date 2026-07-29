#include "orbitlab/Persistence.hpp"

#include <array>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <sstream>
#include <stdexcept>

namespace orbitlab {
namespace {

using Json = nlohmann::json;

const char* collisionModeName(const CollisionMode mode) {
    switch (mode) {
    case CollisionMode::None:
        return "none";
    case CollisionMode::Merge:
        return "merge";
    case CollisionMode::Elastic:
        return "elastic";
    case CollisionMode::Absorb:
        return "absorb";
    case CollisionMode::Fragment:
        return "fragment";
    }
    return "none";
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

Json bodyToJson(const Body& body) {
    return {
        {"id", body.id},
        {"name", body.name},
        {"mass", body.mass},
        {"radius", body.radius},
        {"color", {body.color.r, body.color.g, body.color.b, body.color.a}},
        {"position", {body.position.x, body.position.y, body.position.z}},
        {"velocity", {body.velocity.x, body.velocity.y, body.velocity.z}},
    };
}

Body bodyFromJson(const Json& json) {
    const auto color = json.at("color").get<std::array<float, 4>>();
    const auto position = json.at("position").get<std::vector<double>>();
    const auto velocity = json.at("velocity").get<std::vector<double>>();
    if ((position.size() != 2 && position.size() != 3) ||
        (velocity.size() != 2 && velocity.size() != 3)) {
        throw std::invalid_argument(
            "Body position and velocity must contain two or three components");
    }
    return {
        json.at("id").get<std::uint64_t>(),
        json.at("name").get<std::string>(),
        json.at("mass").get<double>(),
        json.at("radius").get<double>(),
        {color[0], color[1], color[2], color[3]},
        {position[0], position[1], position.size() == 3 ? position[2] : 0.0},
        {velocity[0], velocity[1], velocity.size() == 3 ? velocity[2] : 0.0},
    };
}

} // namespace

std::string serializeSimulation(const Simulation& simulation) {
    Json json{
        {"format", "OrbitLab"},
        {"version", 3},
        {"elapsedTime", simulation.elapsedTime()},
        {"settings",
         {
             {"gravitationalConstant", simulation.settings().gravitationalConstant},
             {"softeningLength", simulation.settings().softeningLength},
             {"fixedTimeStep", simulation.settings().fixedTimeStep},
             {"collisionMode", collisionModeName(simulation.settings().collisionMode)},
             {"trailsEnabled", simulation.settings().trailsEnabled},
             {"solver", solverName(simulation.settings().solverType)},
             {"barnesHutOpeningAngle", simulation.settings().barnesHutOpeningAngle},
             {"integrator", integratorName(simulation.settings().integratorType)},
             {"fragmentationSpeedThreshold",
              simulation.settings().fragmentationSpeedThreshold},
             {"threadWorkerCount", simulation.settings().threadWorkerCount},
             {"adaptiveFidelity",
              {
                  {"enabled", simulation.settings().adaptiveFidelity.enabled},
                  {"safetyFactor",
                   simulation.settings().adaptiveFidelity.safetyFactor},
                  {"jerkWeight", simulation.settings().adaptiveFidelity.jerkWeight},
                  {"encounterWeight",
                   simulation.settings().adaptiveFidelity.encounterWeight},
                  {"minimumTimeStep",
                   simulation.settings().adaptiveFidelity.minimumTimeStep},
                  {"maximumTimeStep",
                   simulation.settings().adaptiveFidelity.maximumTimeStep},
              }},
         }},
        {"bodies", Json::array()},
    };
    for (const auto& body : simulation.bodies()) {
        json["bodies"].push_back(bodyToJson(body));
    }
    return json.dump(2);
}

void deserializeSimulation(Simulation& simulation, const std::string& jsonText) {
    const Json json = Json::parse(jsonText);
    if (json.value("format", "") != "OrbitLab") {
        throw std::invalid_argument("File is not an OrbitLab simulation");
    }
    const int version = json.value("version", 0);
    if (version != 1 && version != 2 && version != 3) {
        throw std::invalid_argument("Unsupported OrbitLab file version");
    }

    const auto& settingsJson = json.at("settings");
    const auto mode = settingsJson.at("collisionMode").get<std::string>();
    if (mode != "merge" && mode != "none" && mode != "elastic" &&
        mode != "absorb" && mode != "fragment") {
        throw std::invalid_argument("Unknown collision mode: " + mode);
    }
    const auto solver = settingsJson.value("solver", std::string{"direct"});
    if (solver != "direct" && solver != "soa-direct" &&
        solver != "threaded-soa" && solver != "barnes-hut") {
        throw std::invalid_argument("Unknown gravity solver: " + solver);
    }
    const auto integrator =
        settingsJson.value("integrator", std::string{"velocity-verlet"});
    if (integrator != "velocity-verlet" && integrator != "symplectic-euler" &&
        integrator != "rk4" && integrator != "yoshida4") {
        throw std::invalid_argument("Unknown integrator: " + integrator);
    }
    CollisionMode collisionMode = CollisionMode::None;
    if (mode == "merge") {
        collisionMode = CollisionMode::Merge;
    } else if (mode == "elastic") {
        collisionMode = CollisionMode::Elastic;
    } else if (mode == "absorb") {
        collisionMode = CollisionMode::Absorb;
    } else if (mode == "fragment") {
        collisionMode = CollisionMode::Fragment;
    }
    IntegratorType integratorType = IntegratorType::VelocityVerlet;
    if (integrator == "symplectic-euler") {
        integratorType = IntegratorType::SymplecticEuler;
    } else if (integrator == "rk4") {
        integratorType = IntegratorType::RungeKutta4;
    } else if (integrator == "yoshida4") {
        integratorType = IntegratorType::Yoshida4;
    }
    SolverType solverType = SolverType::Direct;
    if (solver == "soa-direct") {
        solverType = SolverType::SoADirect;
    } else if (solver == "threaded-soa") {
        solverType = SolverType::ThreadedSoA;
    } else if (solver == "barnes-hut") {
        solverType = SolverType::BarnesHut;
    }
    SimulationSettings settings;
    settings.gravitationalConstant =
        settingsJson.at("gravitationalConstant").get<double>();
    settings.softeningLength = settingsJson.at("softeningLength").get<double>();
    settings.fixedTimeStep = settingsJson.at("fixedTimeStep").get<double>();
    settings.collisionMode = collisionMode;
    settings.trailsEnabled = settingsJson.at("trailsEnabled").get<bool>();
    settings.solverType = solverType;
    settings.barnesHutOpeningAngle =
        settingsJson.value("barnesHutOpeningAngle", 0.6);
    settings.integratorType = integratorType;
    settings.fragmentationSpeedThreshold =
        settingsJson.value("fragmentationSpeedThreshold", 1.25);
    settings.threadWorkerCount = settingsJson.value("threadWorkerCount", 0U);
    if (const auto adaptive = settingsJson.find("adaptiveFidelity");
        adaptive != settingsJson.end()) {
        settings.adaptiveFidelity.enabled = adaptive->value("enabled", false);
        settings.adaptiveFidelity.safetyFactor =
            adaptive->value("safetyFactor", 0.15);
        settings.adaptiveFidelity.jerkWeight =
            adaptive->value("jerkWeight", 0.35);
        settings.adaptiveFidelity.encounterWeight =
            adaptive->value("encounterWeight", 0.65);
        settings.adaptiveFidelity.minimumTimeStep =
            adaptive->value("minimumTimeStep", 0.0000625);
        settings.adaptiveFidelity.maximumTimeStep =
            adaptive->value("maximumTimeStep", 0.004);
    }

    std::vector<Body> bodies;
    const auto& bodyArray = json.at("bodies");
    if (!bodyArray.is_array() || bodyArray.size() > 100'000) {
        throw std::invalid_argument("Bodies must be an array with at most 100,000 entries");
    }
    bodies.reserve(bodyArray.size());
    for (const auto& item : bodyArray) {
        bodies.push_back(bodyFromJson(item));
    }

    simulation.replace(std::move(bodies), settings, json.at("elapsedTime").get<double>());
}

PersistenceResult saveSimulation(
    const Simulation& simulation,
    const std::filesystem::path& path) {
    try {
        std::ofstream stream(path);
        if (!stream) {
            return {false, "Could not open '" + path.string() + "' for writing"};
        }
        stream << serializeSimulation(simulation) << '\n';
        if (!stream) {
            return {false, "Writing the simulation file failed"};
        }
        return {true, "Saved " + std::to_string(simulation.bodies().size()) + " bodies"};
    } catch (const std::exception& exception) {
        return {false, std::string{"Could not save simulation: "} + exception.what()};
    }
}

PersistenceResult loadSimulation(
    Simulation& simulation,
    const std::filesystem::path& path) {
    try {
        std::ifstream stream(path);
        if (!stream) {
            return {false, "Could not open '" + path.string() + "'"};
        }
        std::ostringstream contents;
        contents << stream.rdbuf();
        deserializeSimulation(simulation, contents.str());
        return {true, "Loaded " + std::to_string(simulation.bodies().size()) + " bodies"};
    } catch (const nlohmann::json::exception& exception) {
        return {false, std::string{"Invalid simulation file: "} + exception.what()};
    } catch (const std::exception& exception) {
        return {false, std::string{"Could not load simulation: "} + exception.what()};
    }
}

} // namespace orbitlab
