#include "orbitlab/GravitySolver.hpp"

#include <cmath>
#include <stdexcept>

namespace orbitlab {

std::vector<Vec3> DirectGravitySolver::accelerations(
    std::span<const Body> bodies,
    const double gravitationalConstant,
    const double softeningLength) const {
    if (!std::isfinite(gravitationalConstant) || gravitationalConstant <= 0.0) {
        throw std::invalid_argument("Gravitational constant must be finite and positive");
    }
    if (!std::isfinite(softeningLength) || softeningLength < 0.0) {
        throw std::invalid_argument("Softening length must be finite and non-negative");
    }

    std::vector<Vec3> result(bodies.size());
    const double softeningSquared = softeningLength * softeningLength;

    // Pairwise accumulation applies Newton's third law explicitly. This halves the
    // interaction work and keeps equal-and-opposite force contributions symmetric.
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        for (std::size_t j = i + 1; j < bodies.size(); ++j) {
            const Vec3 displacement = bodies[j].position - bodies[i].position;
            const double distanceSquared = displacement.lengthSquared() + softeningSquared;
            if (distanceSquared == 0.0) {
                continue;
            }
            const double inverseDistance = 1.0 / std::sqrt(distanceSquared);
            const double inverseDistanceCubed =
                inverseDistance * inverseDistance * inverseDistance;
            const Vec3 common = displacement * (gravitationalConstant * inverseDistanceCubed);
            result[i] += common * bodies[j].mass;
            result[j] -= common * bodies[i].mass;
        }
    }
    return result;
}

} // namespace orbitlab
