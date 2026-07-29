#pragma once

#include "orbitlab/AdaptiveFidelity.hpp"
#include "orbitlab/GravitySolver.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace orbitlab {

enum class CollisionMode { None, Merge, Elastic, Absorb, Fragment };
enum class SolverType { Direct, SoADirect, ThreadedSoA, BarnesHut };
enum class IntegratorType { VelocityVerlet, SymplecticEuler, RungeKutta4, Yoshida4 };

struct SimulationSettings {
    double gravitationalConstant{1.0};
    double softeningLength{1.0e-5};
    double fixedTimeStep{0.001};
    CollisionMode collisionMode{CollisionMode::Merge};
    bool trailsEnabled{true};
    SolverType solverType{SolverType::Direct};
    double barnesHutOpeningAngle{0.6};
    IntegratorType integratorType{IntegratorType::VelocityVerlet};
    double fragmentationSpeedThreshold{1.25};
    unsigned int threadWorkerCount{0};
    AdaptiveFidelitySettings adaptiveFidelity{};
};

struct SystemDiagnostics {
    double kineticEnergy{0.0};
    double potentialEnergy{0.0};
    double totalEnergy{0.0};
    Vec3 linearMomentum{};
    double angularMomentum{0.0};
};

struct CollisionEvent {
    double simulationTime{0.0};
    std::string firstBody;
    std::string secondBody;
    std::string mergedBody;
    double combinedMass{0.0};
    double momentumError{0.0};
    std::string eventType{"merge"};
};

struct OrbitalElements {
    std::uint64_t primaryBodyId{0};
    double separation{0.0};
    double relativeSpeed{0.0};
    double eccentricity{0.0};
    double semiMajorAxis{0.0};
    double periapsis{0.0};
    double apoapsis{0.0};
    double period{0.0};
    bool bound{false};
};

class Simulation {
public:
    explicit Simulation(std::unique_ptr<GravitySolver> solver =
                            std::make_unique<DirectGravitySolver>());

    [[nodiscard]] const std::vector<Body>& bodies() const noexcept { return bodies_; }
    [[nodiscard]] std::vector<Body>& bodies() noexcept { return bodies_; }
    [[nodiscard]] const SimulationSettings& settings() const noexcept { return settings_; }
    [[nodiscard]] SimulationSettings& settings() noexcept { return settings_; }
    [[nodiscard]] double elapsedTime() const noexcept { return elapsedTime_; }
    [[nodiscard]] const std::vector<CollisionEvent>& collisionEvents() const noexcept {
        return collisionEvents_;
    }

    Body& addBody(Body body);
    bool removeBody(std::uint64_t id);
    [[nodiscard]] Body* findBody(std::uint64_t id) noexcept;
    [[nodiscard]] const Body* findBody(std::uint64_t id) const noexcept;
    void clear() noexcept;
    void replace(std::vector<Body> bodies, SimulationSettings settings, double elapsedTime);
    void setGravitySolverOverride(std::unique_ptr<GravitySolver> solver);
    void clearGravitySolverOverride();
    [[nodiscard]] bool hasGravitySolverOverride() const noexcept {
        return solverOverrideActive_;
    }
    void step(double deltaTime);
    [[nodiscard]] SystemDiagnostics diagnostics() const;
    [[nodiscard]] std::optional<OrbitalElements> orbitalElements(
        std::uint64_t bodyId) const;

    [[nodiscard]] static std::optional<std::string> validateSettings(
        const SimulationSettings& settings);

private:
    void integrateVelocityVerlet(double deltaTime);
    void integrateSymplecticEuler(double deltaTime);
    void integrateRungeKutta4(double deltaTime);
    void integrateYoshida4(double deltaTime);
    void mergeCollisions();
    void resolveElasticCollisions();
    void fragmentCollisions();
    void pushCollisionEvent(CollisionEvent event);
    void synchronizeSolver();
    void validateBodyOrThrow(const Body& body) const;

    std::vector<Body> bodies_;
    SimulationSettings settings_;
    std::unique_ptr<GravitySolver> solver_;
    std::uint64_t nextId_{1};
    double elapsedTime_{0.0};
    SolverType activeSolverType_{SolverType::Direct};
    double activeOpeningAngle_{0.6};
    unsigned int activeWorkerCount_{0};
    bool solverOverrideActive_{false};
    bool solverSynchronizationRequired_{false};
    std::vector<CollisionEvent> collisionEvents_;
};

} // namespace orbitlab
