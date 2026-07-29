#pragma once

#include <cstddef>
#include <functional>
#include <memory>

namespace orbitlab {

class WorkerPool {
public:
    explicit WorkerPool(unsigned int workerCount = 0);
    ~WorkerPool();

    WorkerPool(const WorkerPool&) = delete;
    WorkerPool& operator=(const WorkerPool&) = delete;
    WorkerPool(WorkerPool&&) = delete;
    WorkerPool& operator=(WorkerPool&&) = delete;

    void parallelFor(
        std::size_t itemCount,
        std::size_t minimumItemsPerTask,
        const std::function<void(std::size_t first, std::size_t last)>& function);

    [[nodiscard]] unsigned int workerCount() const noexcept;

private:
    class Implementation;
    std::unique_ptr<Implementation> implementation_;
};

} // namespace orbitlab
