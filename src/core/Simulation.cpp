#include "orbitlab/Simulation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace orbitlab {

Simulation::Simulation(std::unique_ptr<GravitySolver> solver) : solver_(std::move(solver)) {
    if (!solver_) {
        throw std::invalid_argument("Simulation requires a gravity solver");
    }
}

Body& Simulation::addBody(Body body) {
    if (body.id == 0) {
        body.id = nextId_++;
    } else {
        if (findBody(body.id) != nullptr) {
            throw std::invalid_argument("Body IDs must be unique");
        }
        nextId_ = std::max(nextId_, body.id + 1);
    }
    validateBodyOrThrow(body);
    bodies_.push_back(std::move(body));
    return bodies_.back();
}

bool Simulation::removeBody(const std::uint64_t id) {
    const auto oldSize = bodies_.size();
    std::erase_if(bodies_, [id](const Body& body) { return body.id == id; });
    return bodies_.size() != oldSize;
}

Body* Simulation::findBody(const std::uint64_t id) noexcept {
    const auto found =
        std::find_if(bodies_.begin(), bodies_.end(), [id](const Body& body) {
            return body.id == id;
        });
    return found == bodies_.end() ? nullptr : &*found;
}

const Body* Simulation::findBody(const std::uint64_t id) const noexcept {
    const auto found =
        std::find_if(bodies_.begin(), bodies_.end(), [id](const Body& body) {
            return body.id == id;
        });
    return found == bodies_.end() ? nullptr : &*found;
}

void Simulation::clear() noexcept {
    bodies_.clear();
    collisionEvents_.clear();
    elapsedTime_ = 0.0;
    nextId_ = 1;
}

void Simulation::replace(
    std::vector<Body> bodies,
    const SimulationSettings settings,
    const double elapsedTime) {
    if (const auto error = validateSettings(settings)) {
        throw std::invalid_argument(*error);
    }
    if (!std::isfinite(elapsedTime) || elapsedTime < 0.0) {
        throw std::invalid_argument("Elapsed time must be finite and non-negative");
    }

    std::uint64_t maximumId = 0;
    for (const auto& body : bodies) {
        validateBodyOrThrow(body);
        if (body.id <= maximumId &&
            std::any_of(bodies.begin(), bodies.end(), [&](const Body& other) {
                return &body != &other && body.id == other.id;
            })) {
            throw std::invalid_argument("Body IDs must be unique");
        }
        maximumId = std::max(maximumId, body.id);
    }

    bodies_ = std::move(bodies);
    settings_ = settings;
    synchronizeSolver();
    collisionEvents_.clear();
    elapsedTime_ = elapsedTime;
    nextId_ = maximumId + 1;
}

void Simulation::setGravitySolverOverride(std::unique_ptr<GravitySolver> solver) {
    if (!solver) {
        throw std::invalid_argument("Gravity solver override cannot be null");
    }
    solver_ = std::move(solver);
    solverOverrideActive_ = true;
}

void Simulation::clearGravitySolverOverride() {
    if (!solverOverrideActive_) {
        return;
    }
    solverOverrideActive_ = false;
    solverSynchronizationRequired_ = true;
    synchronizeSolver();
}

void Simulation::step(const double deltaTime) {
    if (!std::isfinite(deltaTime) || deltaTime <= 0.0) {
        throw std::invalid_argument("Simulation step must be finite and positive");
    }
    if (const auto error = validateSettings(settings_)) {
        throw std::invalid_argument(*error);
    }
    if (bodies_.empty()) {
        elapsedTime_ += deltaTime;
        return;
    }
    synchronizeSolver();

    switch (settings_.integratorType) {
    case IntegratorType::VelocityVerlet:
        integrateVelocityVerlet(deltaTime);
        break;
    case IntegratorType::SymplecticEuler:
        integrateSymplecticEuler(deltaTime);
        break;
    case IntegratorType::RungeKutta4:
        integrateRungeKutta4(deltaTime);
        break;
    case IntegratorType::Yoshida4:
        integrateYoshida4(deltaTime);
        break;
    }

    for (const auto& body : bodies_) {
        if (!body.position.isFinite() || !body.velocity.isFinite()) {
            throw std::overflow_error("Simulation produced a non-finite body state");
        }
    }

    if (settings_.collisionMode == CollisionMode::Merge ||
        settings_.collisionMode == CollisionMode::Absorb) {
        mergeCollisions();
    } else if (settings_.collisionMode == CollisionMode::Elastic) {
        resolveElasticCollisions();
    } else if (settings_.collisionMode == CollisionMode::Fragment) {
        fragmentCollisions();
    }
    elapsedTime_ += deltaTime;
}

void Simulation::integrateVelocityVerlet(const double deltaTime) {
    // Kick-drift-kick is symplectic and substantially more stable for orbital systems than
    // forward Euler while retaining a compact fixed-step API.
    const auto initialAccelerations =
        solver_->accelerations(bodies_, settings_.gravitationalConstant, settings_.softeningLength);
    for (std::size_t index = 0; index < bodies_.size(); ++index) {
        bodies_[index].velocity += initialAccelerations[index] * (0.5 * deltaTime);
        bodies_[index].position += bodies_[index].velocity * deltaTime;
    }

    const auto finalAccelerations =
        solver_->accelerations(bodies_, settings_.gravitationalConstant, settings_.softeningLength);
    for (std::size_t index = 0; index < bodies_.size(); ++index) {
        bodies_[index].velocity += finalAccelerations[index] * (0.5 * deltaTime);
    }
}

void Simulation::integrateSymplecticEuler(const double deltaTime) {
    const auto acceleration =
        solver_->accelerations(bodies_, settings_.gravitationalConstant, settings_.softeningLength);
    for (std::size_t index = 0; index < bodies_.size(); ++index) {
        bodies_[index].velocity += acceleration[index] * deltaTime;
        bodies_[index].position += bodies_[index].velocity * deltaTime;
    }
}

void Simulation::integrateRungeKutta4(const double deltaTime) {
    const std::vector<Body> original = bodies_;
    const auto accelerationFor = [&](const std::vector<Body>& state) {
        return solver_->accelerations(
            state, settings_.gravitationalConstant, settings_.softeningLength);
    };
    const auto stagedState = [&](
                                 const std::vector<Vec3>& positionSlope,
                                 const std::vector<Vec3>& velocitySlope,
                                 const double scale) {
        std::vector<Body> state = original;
        for (std::size_t index = 0; index < state.size(); ++index) {
            state[index].position += positionSlope[index] * scale;
            state[index].velocity += velocitySlope[index] * scale;
        }
        return state;
    };

    std::vector<Vec3> k1Position(original.size());
    for (std::size_t index = 0; index < original.size(); ++index) {
        k1Position[index] = original[index].velocity;
    }
    const auto k1Velocity = accelerationFor(original);

    const auto secondState =
        stagedState(k1Position, k1Velocity, deltaTime * 0.5);
    std::vector<Vec3> k2Position(original.size());
    for (std::size_t index = 0; index < original.size(); ++index) {
        k2Position[index] = secondState[index].velocity;
    }
    const auto k2Velocity = accelerationFor(secondState);

    const auto thirdState =
        stagedState(k2Position, k2Velocity, deltaTime * 0.5);
    std::vector<Vec3> k3Position(original.size());
    for (std::size_t index = 0; index < original.size(); ++index) {
        k3Position[index] = thirdState[index].velocity;
    }
    const auto k3Velocity = accelerationFor(thirdState);

    const auto fourthState = stagedState(k3Position, k3Velocity, deltaTime);
    std::vector<Vec3> k4Position(original.size());
    for (std::size_t index = 0; index < original.size(); ++index) {
        k4Position[index] = fourthState[index].velocity;
    }
    const auto k4Velocity = accelerationFor(fourthState);

    for (std::size_t index = 0; index < bodies_.size(); ++index) {
        bodies_[index].position =
            original[index].position +
            (k1Position[index] + k2Position[index] * 2.0 +
             k3Position[index] * 2.0 + k4Position[index]) *
                (deltaTime / 6.0);
        bodies_[index].velocity =
            original[index].velocity +
            (k1Velocity[index] + k2Velocity[index] * 2.0 +
             k3Velocity[index] * 2.0 + k4Velocity[index]) *
                (deltaTime / 6.0);
    }
}

void Simulation::integrateYoshida4(const double deltaTime) {
    const double cubeRootTwo = std::cbrt(2.0);
    const double firstWeight = 1.0 / (2.0 - cubeRootTwo);
    const double middleWeight = -cubeRootTwo / (2.0 - cubeRootTwo);
    integrateVelocityVerlet(deltaTime * firstWeight);
    integrateVelocityVerlet(deltaTime * middleWeight);
    integrateVelocityVerlet(deltaTime * firstWeight);
}

SystemDiagnostics Simulation::diagnostics() const {
    SystemDiagnostics result;
    Vec3 angularMomentum{};
    for (const auto& body : bodies_) {
        result.kineticEnergy += 0.5 * body.mass * body.velocity.lengthSquared();
        result.linearMomentum += body.velocity * body.mass;
        angularMomentum += body.position.cross(body.velocity) * body.mass;
    }
    result.angularMomentum = angularMomentum.length();
    for (std::size_t first = 0; first < bodies_.size(); ++first) {
        for (std::size_t second = first + 1; second < bodies_.size(); ++second) {
            const double softenedDistance = std::sqrt(
                (bodies_[second].position - bodies_[first].position).lengthSquared() +
                settings_.softeningLength * settings_.softeningLength);
            if (softenedDistance > 0.0) {
                result.potentialEnergy -= settings_.gravitationalConstant *
                                          bodies_[first].mass * bodies_[second].mass /
                                          softenedDistance;
            }
        }
    }
    result.totalEnergy = result.kineticEnergy + result.potentialEnergy;
    return result;
}

std::optional<OrbitalElements> Simulation::orbitalElements(
    const std::uint64_t bodyId) const {
    const Body* body = findBody(bodyId);
    if (body == nullptr || bodies_.size() < 2) {
        return std::nullopt;
    }
    const Body* primary = nullptr;
    for (const auto& candidate : bodies_) {
        if (candidate.id != bodyId &&
            (primary == nullptr || candidate.mass > primary->mass)) {
            primary = &candidate;
        }
    }
    if (primary == nullptr) {
        return std::nullopt;
    }

    const Vec3 relativePosition = body->position - primary->position;
    const Vec3 relativeVelocity = body->velocity - primary->velocity;
    const double separation = relativePosition.length();
    const double gravitationalParameter =
        settings_.gravitationalConstant * (body->mass + primary->mass);
    if (separation <= 0.0 || gravitationalParameter <= 0.0) {
        return std::nullopt;
    }

    const double speedSquared = relativeVelocity.lengthSquared();
    const double radialVelocity = relativePosition.dot(relativeVelocity);
    const Vec3 eccentricityVector =
        (relativePosition * (speedSquared - gravitationalParameter / separation) -
         relativeVelocity * radialVelocity) /
        gravitationalParameter;
    const double eccentricity = eccentricityVector.length();
    const double specificEnergy =
        speedSquared * 0.5 - gravitationalParameter / separation;
    const bool bound = specificEnergy < 0.0 && eccentricity < 1.0;
    const double semiMajorAxis =
        specificEnergy != 0.0 ? -gravitationalParameter / (2.0 * specificEnergy) : 0.0;

    OrbitalElements result;
    result.primaryBodyId = primary->id;
    result.separation = separation;
    result.relativeSpeed = std::sqrt(speedSquared);
    result.eccentricity = eccentricity;
    result.semiMajorAxis = semiMajorAxis;
    result.bound = bound;
    if (bound) {
        result.periapsis = semiMajorAxis * (1.0 - eccentricity);
        result.apoapsis = semiMajorAxis * (1.0 + eccentricity);
        result.period =
            2.0 * std::numbers::pi *
            std::sqrt(semiMajorAxis * semiMajorAxis * semiMajorAxis /
                      gravitationalParameter);
    }
    return result;
}

std::optional<std::string> Simulation::validateSettings(const SimulationSettings& settings) {
    if (!std::isfinite(settings.gravitationalConstant) ||
        settings.gravitationalConstant <= 0.0) {
        return "Gravitational constant must be finite and positive";
    }
    if (!std::isfinite(settings.softeningLength) || settings.softeningLength < 0.0) {
        return "Softening length must be finite and non-negative";
    }
    if (!std::isfinite(settings.fixedTimeStep) || settings.fixedTimeStep <= 0.0 ||
        settings.fixedTimeStep > 1.0) {
        return "Fixed time step must be finite and in the range (0, 1]";
    }
    if (!std::isfinite(settings.barnesHutOpeningAngle) ||
        settings.barnesHutOpeningAngle <= 0.0 || settings.barnesHutOpeningAngle > 2.0) {
        return "Barnes-Hut opening angle must be finite and in the range (0, 2]";
    }
    if (!std::isfinite(settings.fragmentationSpeedThreshold) ||
        settings.fragmentationSpeedThreshold <= 0.0) {
        return "Fragmentation speed threshold must be finite and positive";
    }
    if (settings.threadWorkerCount > 256) {
        return "Worker thread count must be zero (automatic) or at most 256";
    }
    if (const std::string error =
            AdaptiveFidelityController::validate(settings.adaptiveFidelity);
        !error.empty()) {
        return error;
    }
    if (settings.adaptiveFidelity.enabled &&
        settings.integratorType != IntegratorType::RungeKutta4) {
        return "OrbitLab adaptive fidelity currently requires Runge-Kutta 4";
    }
    return std::nullopt;
}

void Simulation::mergeCollisions() {
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
        std::size_t j = i + 1;
        while (j < bodies_.size()) {
            Body& first = bodies_[i];
            const Body& second = bodies_[j];
            const double combinedRadius = first.radius + second.radius;
            if ((second.position - first.position).lengthSquared() >
                combinedRadius * combinedRadius) {
                ++j;
                continue;
            }

            const double totalMass = first.mass + second.mass;
            const std::string firstName = first.name;
            const std::string secondName = second.name;
            const Vec3 momentumBefore =
                first.velocity * first.mass + second.velocity * second.mass;
            first.position = (first.position * first.mass + second.position * second.mass) /
                             totalMass;
            first.velocity = (first.velocity * first.mass + second.velocity * second.mass) /
                             totalMass;
            const bool secondKeepsName = second.mass > first.mass;
            if (settings_.collisionMode == CollisionMode::Merge) {
                first.color = {
                    static_cast<float>(
                        (first.color.r * first.mass + second.color.r * second.mass) /
                        totalMass),
                    static_cast<float>(
                        (first.color.g * first.mass + second.color.g * second.mass) /
                        totalMass),
                    static_cast<float>(
                        (first.color.b * first.mass + second.color.b * second.mass) /
                        totalMass),
                    1.0F,
                };
            } else if (secondKeepsName) {
                first.color = second.color;
            }
            first.mass = totalMass;
            first.radius = std::cbrt(
                first.radius * first.radius * first.radius +
                second.radius * second.radius * second.radius);
            if (secondKeepsName) {
                first.name = second.name;
            }
            const double momentumError =
                (first.velocity * first.mass - momentumBefore).length();
            pushCollisionEvent(
                {elapsedTime_,
                 firstName,
                 secondName,
                 first.name,
                 totalMass,
                 momentumError,
                 settings_.collisionMode == CollisionMode::Absorb ? "absorb" : "merge"});
            bodies_.erase(bodies_.begin() + static_cast<std::ptrdiff_t>(j));
        }
    }
}

void Simulation::resolveElasticCollisions() {
    for (std::size_t firstIndex = 0; firstIndex < bodies_.size(); ++firstIndex) {
        for (std::size_t secondIndex = firstIndex + 1; secondIndex < bodies_.size();
             ++secondIndex) {
            Body& first = bodies_[firstIndex];
            Body& second = bodies_[secondIndex];
            Vec3 displacement = second.position - first.position;
            const double minimumDistance = first.radius + second.radius;
            const double distance = displacement.length();
            if (distance >= minimumDistance) {
                continue;
            }
            const Vec3 normal =
                distance > 1.0e-12 ? displacement / distance : Vec3{1.0, 0.0, 0.0};
            const Vec3 relativeVelocity = second.velocity - first.velocity;
            const double normalSpeed = relativeVelocity.dot(normal);
            if (normalSpeed < 0.0) {
                const Vec3 momentumBefore =
                    first.velocity * first.mass + second.velocity * second.mass;
                const double inverseMassSum = 1.0 / first.mass + 1.0 / second.mass;
                const double impulseMagnitude = -2.0 * normalSpeed / inverseMassSum;
                const Vec3 impulse = normal * impulseMagnitude;
                first.velocity -= impulse / first.mass;
                second.velocity += impulse / second.mass;
                const Vec3 momentumAfter =
                    first.velocity * first.mass + second.velocity * second.mass;
                pushCollisionEvent(
                    {elapsedTime_,
                     first.name,
                     second.name,
                     "elastic rebound",
                     first.mass + second.mass,
                     (momentumAfter - momentumBefore).length(),
                     "elastic"});
            }

            const double penetration = minimumDistance - distance;
            const double inverseMassSum = 1.0 / first.mass + 1.0 / second.mass;
            const Vec3 correction = normal * (penetration * 0.82 / inverseMassSum);
            first.position -= correction / first.mass;
            second.position += correction / second.mass;
        }
    }
}

void Simulation::fragmentCollisions() {
    constexpr std::size_t maximumBodies = 10'000;
    constexpr int maximumFragmentationsPerStep = 16;
    int fragmentationCount = 0;
    bool fragmented = true;
    while (fragmented && fragmentationCount < maximumFragmentationsPerStep) {
        fragmented = false;
        for (std::size_t firstIndex = 0; firstIndex < bodies_.size() && !fragmented;
             ++firstIndex) {
            for (std::size_t secondIndex = firstIndex + 1; secondIndex < bodies_.size();
                 ++secondIndex) {
                const Body first = bodies_[firstIndex];
                const Body second = bodies_[secondIndex];
                if ((second.position - first.position).length() >=
                    first.radius + second.radius) {
                    continue;
                }
                const double impactSpeed = (second.velocity - first.velocity).length();
                if (impactSpeed < settings_.fragmentationSpeedThreshold ||
                    bodies_.size() + 4 > maximumBodies) {
                    continue;
                }

                const double totalMass = first.mass + second.mass;
                const Vec3 momentumBefore =
                    first.velocity * first.mass + second.velocity * second.mass;
                const Vec3 center =
                    (first.position * first.mass + second.position * second.mass) /
                    totalMass;
                const Vec3 centerVelocity = momentumBefore / totalMass;
                const double combinedRadius =
                    std::cbrt(
                        first.radius * first.radius * first.radius +
                        second.radius * second.radius * second.radius);
                const Color color{
                    static_cast<float>(
                        (first.color.r * first.mass + second.color.r * second.mass) /
                        totalMass),
                    static_cast<float>(
                        (first.color.g * first.mass + second.color.g * second.mass) /
                        totalMass),
                    static_cast<float>(
                        (first.color.b * first.mass + second.color.b * second.mass) /
                        totalMass),
                    1.0F,
                };
                const std::string eventName = first.name + " / " + second.name;
                bodies_.erase(
                    bodies_.begin() + static_cast<std::ptrdiff_t>(secondIndex));
                bodies_.erase(
                    bodies_.begin() + static_cast<std::ptrdiff_t>(firstIndex));

                constexpr std::array<Vec3, 6> directions{
                    Vec3{1.0, 0.0, 0.0},
                    Vec3{-1.0, 0.0, 0.0},
                    Vec3{0.0, 1.0, 0.0},
                    Vec3{0.0, -1.0, 0.0},
                    Vec3{0.0, 0.0, 1.0},
                    Vec3{0.0, 0.0, -1.0},
                };
                const double spreadSpeed =
                    std::max(impactSpeed * 0.35,
                             settings_.fragmentationSpeedThreshold * 0.25);
                for (std::size_t fragmentIndex = 0; fragmentIndex < directions.size();
                     ++fragmentIndex) {
                    addBody(
                        {0,
                         "Fragment " + std::to_string(nextId_),
                         totalMass / static_cast<double>(directions.size()),
                         combinedRadius / std::cbrt(
                                              static_cast<double>(directions.size())),
                         color,
                         center + directions[fragmentIndex] * (combinedRadius * 0.85),
                         centerVelocity + directions[fragmentIndex] * spreadSpeed});
                }
                Vec3 momentumAfter{};
                for (std::size_t index = bodies_.size() - directions.size();
                     index < bodies_.size();
                     ++index) {
                    momentumAfter += bodies_[index].velocity * bodies_[index].mass;
                }
                pushCollisionEvent(
                    {elapsedTime_,
                     first.name,
                     second.name,
                     eventName,
                     totalMass,
                     (momentumAfter - momentumBefore).length(),
                     "fragment"});
                ++fragmentationCount;
                fragmented = true;
                break;
            }
        }
    }
    resolveElasticCollisions();
}

void Simulation::pushCollisionEvent(CollisionEvent event) {
    collisionEvents_.push_back(std::move(event));
    constexpr std::size_t maximumEvents = 128;
    if (collisionEvents_.size() > maximumEvents) {
        collisionEvents_.erase(collisionEvents_.begin());
    }
}

void Simulation::synchronizeSolver() {
    if (solverOverrideActive_) {
        return;
    }
    if (!solverSynchronizationRequired_ &&
        settings_.solverType == activeSolverType_ &&
        (settings_.solverType != SolverType::BarnesHut ||
         settings_.barnesHutOpeningAngle == activeOpeningAngle_) &&
        (settings_.solverType != SolverType::ThreadedSoA ||
         settings_.threadWorkerCount == activeWorkerCount_)) {
        return;
    }
    if (settings_.solverType == SolverType::BarnesHut) {
        solver_ = std::make_unique<BarnesHutGravitySolver>(
            settings_.barnesHutOpeningAngle);
        activeOpeningAngle_ = settings_.barnesHutOpeningAngle;
    } else if (settings_.solverType == SolverType::SoADirect) {
        solver_ = std::make_unique<SoADirectGravitySolver>();
    } else if (settings_.solverType == SolverType::ThreadedSoA) {
        solver_ =
            std::make_unique<ThreadedSoAGravitySolver>(settings_.threadWorkerCount);
        activeWorkerCount_ = settings_.threadWorkerCount;
    } else {
        solver_ = std::make_unique<DirectGravitySolver>();
    }
    activeSolverType_ = settings_.solverType;
    solverSynchronizationRequired_ = false;
}

void Simulation::validateBodyOrThrow(const Body& body) const {
    if (!body.isValid()) {
        throw std::invalid_argument(
            "Body must have a unique ID, non-empty name, positive finite mass and radius, "
            "and finite position and velocity");
    }
}

} // namespace orbitlab
