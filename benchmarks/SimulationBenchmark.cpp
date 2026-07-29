#include "orbitlab/GravitySolver.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

namespace {

std::vector<orbitlab::Body> makeSystem(const std::size_t count) {
    std::vector<orbitlab::Body> bodies;
    bodies.reserve(count);
    std::mt19937_64 generator(42);
    std::uniform_real_distribution<double> distribution(-2.0, 2.0);
    for (std::size_t index = 0; index < count; ++index) {
        bodies.push_back({
            index + 1,
            "Body " + std::to_string(index + 1),
            1.0 / static_cast<double>(count),
            0.001,
            {},
            {distribution(generator), distribution(generator), distribution(generator)},
            {},
        });
    }
    return bodies;
}

double measure(
    const orbitlab::GravitySolver& solver,
    const std::vector<orbitlab::Body>& bodies,
    const int repetitions) {
    std::vector<orbitlab::Vec3> result;
    const auto start = std::chrono::steady_clock::now();
    for (int repetition = 0; repetition < repetitions; ++repetition) {
        result = solver.accelerations(bodies, 1.0, 0.002);
    }
    const auto elapsed =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start);
    // Keep the evaluated result observable to the optimizer without platform intrinsics.
    volatile double observation = result.empty() ? 0.0 : result.front().x;
    static_cast<void>(observation);
    return elapsed.count() / repetitions;
}

} // namespace

int main() {
    std::cout << "OrbitLab gravity-solver performance harness\n"
              << "Times are measurements from this run, not published product claims.\n\n"
              << std::left << std::setw(9) << "Bodies" << std::right << std::setw(15)
              << "Pairwise" << std::setw(15) << "SoA" << std::setw(15)
              << "Threaded SoA" << std::setw(15) << "Barnes-Hut" << std::setw(15)
              << "BH error %" << '\n';

    for (const auto count : {16U, 64U, 128U, 256U, 512U, 1'024U}) {
        const auto bodies = makeSystem(count);
        const orbitlab::DirectGravitySolver direct;
        const orbitlab::SoADirectGravitySolver soa;
        const orbitlab::ThreadedSoAGravitySolver threaded;
        const orbitlab::BarnesHutGravitySolver barnesHut;
        const int repetitions = count >= 512 ? 4 : 12;

        const auto directAcceleration = direct.accelerations(bodies, 1.0, 0.002);
        const auto approximateAcceleration = barnesHut.accelerations(bodies, 1.0, 0.002);
        double errorSquared = 0.0;
        double referenceSquared = 0.0;
        for (std::size_t index = 0; index < directAcceleration.size(); ++index) {
            errorSquared +=
                (approximateAcceleration[index] - directAcceleration[index]).lengthSquared();
            referenceSquared += directAcceleration[index].lengthSquared();
        }
        const double relativeError =
            referenceSquared > 0.0 ? std::sqrt(errorSquared / referenceSquared) * 100.0 : 0.0;

        std::cout << std::left << std::setw(9) << count << std::right << std::fixed
                  << std::setprecision(4) << std::setw(15)
                  << measure(direct, bodies, repetitions) << std::setw(15)
                  << measure(soa, bodies, repetitions) << std::setw(15)
                  << measure(threaded, bodies, repetitions) << std::setw(15)
                  << measure(barnesHut, bodies, repetitions) << std::setw(15)
                  << relativeError << '\n';
    }
}
