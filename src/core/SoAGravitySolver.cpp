#include "orbitlab/GravitySolver.hpp"
#include "orbitlab/WorkerPool.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace orbitlab {
namespace {

struct BodyArrays {
    explicit BodyArrays(const std::span<const Body> bodies) {
        x.reserve(bodies.size());
        y.reserve(bodies.size());
        z.reserve(bodies.size());
        mass.reserve(bodies.size());
        for (const auto& body : bodies) {
            x.push_back(body.position.x);
            y.push_back(body.position.y);
            z.push_back(body.position.z);
            mass.push_back(body.mass);
        }
    }

    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    std::vector<double> mass;
};

void validateParameters(
    const double gravitationalConstant,
    const double softeningLength) {
    if (!std::isfinite(gravitationalConstant) || gravitationalConstant <= 0.0) {
        throw std::invalid_argument("Gravitational constant must be finite and positive");
    }
    if (!std::isfinite(softeningLength) || softeningLength < 0.0) {
        throw std::invalid_argument("Softening length must be finite and non-negative");
    }
}

void accumulateTargets(
    const BodyArrays& bodies,
    std::vector<Vec3>& result,
    const std::size_t first,
    const std::size_t last,
    const double gravitationalConstant,
    const double softeningSquared) {
    for (std::size_t target = first; target < last; ++target) {
        double accelerationX = 0.0;
        double accelerationY = 0.0;
        double accelerationZ = 0.0;
        for (std::size_t source = 0; source < bodies.x.size(); ++source) {
            if (source == target) {
                continue;
            }
            const double dx = bodies.x[source] - bodies.x[target];
            const double dy = bodies.y[source] - bodies.y[target];
            const double dz = bodies.z[source] - bodies.z[target];
            const double distanceSquared =
                dx * dx + dy * dy + dz * dz + softeningSquared;
            if (distanceSquared == 0.0) {
                continue;
            }
            const double inverseDistance = 1.0 / std::sqrt(distanceSquared);
            const double scale = gravitationalConstant * bodies.mass[source] *
                                 inverseDistance * inverseDistance * inverseDistance;
            accelerationX += dx * scale;
            accelerationY += dy * scale;
            accelerationZ += dz * scale;
        }
        result[target] = {accelerationX, accelerationY, accelerationZ};
    }
}

} // namespace

std::vector<Vec3> SoADirectGravitySolver::accelerations(
    const std::span<const Body> bodies,
    const double gravitationalConstant,
    const double softeningLength) const {
    validateParameters(gravitationalConstant, softeningLength);
    const BodyArrays arrays{bodies};
    std::vector<Vec3> result(bodies.size());
    accumulateTargets(
        arrays,
        result,
        0,
        bodies.size(),
        gravitationalConstant,
        softeningLength * softeningLength);
    return result;
}

ThreadedSoAGravitySolver::ThreadedSoAGravitySolver(const unsigned int workerCount)
    : workerPool_(std::make_shared<WorkerPool>(workerCount)) {}

std::vector<Vec3> ThreadedSoAGravitySolver::accelerations(
    const std::span<const Body> bodies,
    const double gravitationalConstant,
    const double softeningLength) const {
    validateParameters(gravitationalConstant, softeningLength);
    const BodyArrays arrays{bodies};
    std::vector<Vec3> result(bodies.size());

    workerPool_->parallelFor(
        bodies.size(),
        64,
        [&](const std::size_t first, const std::size_t last) {
            accumulateTargets(
                arrays,
                result,
                first,
                last,
                gravitationalConstant,
                softeningLength * softeningLength);
        });
    return result;
}

unsigned int ThreadedSoAGravitySolver::workerCount() const noexcept {
    return workerPool_->workerCount() + 1;
}

} // namespace orbitlab
