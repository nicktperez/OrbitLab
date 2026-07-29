#pragma once

#include "orbitlab/Body.hpp"

#include <span>
#include <memory>
#include <vector>

namespace orbitlab {

class WorkerPool;

class GravitySolver {
public:
    virtual ~GravitySolver() = default;
    [[nodiscard]] virtual std::vector<Vec3> accelerations(
        std::span<const Body> bodies,
        double gravitationalConstant,
        double softeningLength) const = 0;
};

class DirectGravitySolver final : public GravitySolver {
public:
    [[nodiscard]] std::vector<Vec3> accelerations(
        std::span<const Body> bodies,
        double gravitationalConstant,
        double softeningLength) const override;
};

// These direct solvers deliberately share the same public contract as Barnes-Hut.
// SoA isolates the data-layout experiment; ThreadedSoA partitions independent target
// bodies across worker threads without exposing concurrency to Simulation.
class SoADirectGravitySolver final : public GravitySolver {
public:
    [[nodiscard]] std::vector<Vec3> accelerations(
        std::span<const Body> bodies,
        double gravitationalConstant,
        double softeningLength) const override;
};

class ThreadedSoAGravitySolver final : public GravitySolver {
public:
    explicit ThreadedSoAGravitySolver(unsigned int workerCount = 0);

    [[nodiscard]] std::vector<Vec3> accelerations(
        std::span<const Body> bodies,
        double gravitationalConstant,
        double softeningLength) const override;

    [[nodiscard]] unsigned int workerCount() const noexcept;

private:
    std::shared_ptr<WorkerPool> workerPool_;
};

class BarnesHutGravitySolver final : public GravitySolver {
public:
    explicit BarnesHutGravitySolver(double openingAngle = 0.6);

    [[nodiscard]] std::vector<Vec3> accelerations(
        std::span<const Body> bodies,
        double gravitationalConstant,
        double softeningLength) const override;

    [[nodiscard]] double openingAngle() const noexcept { return openingAngle_; }

private:
    double openingAngle_;
};

} // namespace orbitlab
