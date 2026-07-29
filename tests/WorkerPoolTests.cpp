#include "orbitlab/WorkerPool.hpp"

#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <stdexcept>
#include <vector>

using namespace orbitlab;

TEST_CASE("WorkerPool reuses workers across parallel ranges", "[concurrency]") {
    WorkerPool pool{2};
    std::vector<int> values(1'000);
    for (int pass = 1; pass <= 4; ++pass) {
        pool.parallelFor(
            values.size(),
            32,
            [&](const std::size_t first, const std::size_t last) {
                for (std::size_t index = first; index < last; ++index) {
                    values[index] += pass;
                }
            });
    }

    REQUIRE(pool.workerCount() == 2);
    for (const int value : values) {
        REQUIRE(value == 10);
    }
}

TEST_CASE("WorkerPool propagates task exceptions after joining work", "[concurrency]") {
    WorkerPool pool{2};
    std::atomic<int> visited{0};
    REQUIRE_THROWS_AS(
        pool.parallelFor(
            256,
            32,
            [&](const std::size_t first, const std::size_t last) {
                visited.fetch_add(
                    static_cast<int>(last - first),
                    std::memory_order_relaxed);
                if (first == 0) {
                    throw std::runtime_error("expected test failure");
                }
            }),
        std::runtime_error);
    REQUIRE(visited.load(std::memory_order_relaxed) == 256);
}
