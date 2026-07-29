#include "orbitlab/WorkerPool.hpp"

#include <algorithm>
#include <condition_variable>
#include <deque>
#include <exception>
#include <latch>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace orbitlab {

class WorkerPool::Implementation {
public:
    explicit Implementation(const unsigned int requestedWorkerCount) {
        const unsigned int hardwareThreads =
            std::max(1U, std::thread::hardware_concurrency());
        const unsigned int automaticWorkers = hardwareThreads > 1 ? hardwareThreads - 1 : 0;
        const unsigned int count =
            requestedWorkerCount == 0 ? automaticWorkers : requestedWorkerCount;
        workers_.reserve(count);
        for (unsigned int index = 0; index < count; ++index) {
            workers_.emplace_back([this](const std::stop_token stopToken) {
                workerLoop(stopToken);
            });
        }
    }

    ~Implementation() {
        for (auto& worker : workers_) {
            worker.request_stop();
        }
        condition_.notify_all();
    }

    void parallelFor(
        const std::size_t itemCount,
        const std::size_t minimumItemsPerTask,
        const std::function<void(std::size_t, std::size_t)>& function) {
        if (itemCount == 0) {
            return;
        }
        if (minimumItemsPerTask == 0) {
            throw std::invalid_argument("WorkerPool task size must be greater than zero");
        }
        const std::size_t desiredTasks =
            std::min<std::size_t>(workers_.size() + 1,
                                  (itemCount + minimumItemsPerTask - 1) /
                                      minimumItemsPerTask);
        if (desiredTasks <= 1) {
            function(0, itemCount);
            return;
        }

        const std::size_t backgroundTasks = desiredTasks - 1;
        std::latch completion{static_cast<std::ptrdiff_t>(backgroundTasks)};
        std::mutex exceptionMutex;
        std::exception_ptr firstException;
        const std::size_t batchSize = (itemCount + desiredTasks - 1) / desiredTasks;

        for (std::size_t taskIndex = 0; taskIndex < backgroundTasks; ++taskIndex) {
            const std::size_t first = taskIndex * batchSize;
            const std::size_t last = std::min(first + batchSize, itemCount);
            enqueue([&, first, last] {
                try {
                    function(first, last);
                } catch (...) {
                    std::scoped_lock lock{exceptionMutex};
                    if (!firstException) {
                        firstException = std::current_exception();
                    }
                }
                completion.count_down();
            });
        }

        const std::size_t callerFirst = backgroundTasks * batchSize;
        try {
            function(callerFirst, itemCount);
        } catch (...) {
            std::scoped_lock lock{exceptionMutex};
            firstException = std::current_exception();
        }
        completion.wait();
        if (firstException) {
            std::rethrow_exception(firstException);
        }
    }

    [[nodiscard]] unsigned int workerCount() const noexcept {
        return static_cast<unsigned int>(workers_.size());
    }

private:
    void enqueue(std::function<void()> task) {
        {
            std::scoped_lock lock{mutex_};
            tasks_.push_back(std::move(task));
        }
        condition_.notify_one();
    }

    void workerLoop(const std::stop_token stopToken) {
        while (!stopToken.stop_requested()) {
            std::function<void()> task;
            {
                std::unique_lock lock{mutex_};
                condition_.wait(lock, stopToken, [this] { return !tasks_.empty(); });
                if (stopToken.stop_requested()) {
                    return;
                }
                task = std::move(tasks_.front());
                tasks_.pop_front();
            }
            task();
        }
    }

    std::mutex mutex_;
    std::condition_variable_any condition_;
    std::deque<std::function<void()>> tasks_;
    std::vector<std::jthread> workers_;
};

WorkerPool::WorkerPool(const unsigned int workerCount)
    : implementation_(std::make_unique<Implementation>(workerCount)) {}

WorkerPool::~WorkerPool() = default;

void WorkerPool::parallelFor(
    const std::size_t itemCount,
    const std::size_t minimumItemsPerTask,
    const std::function<void(std::size_t, std::size_t)>& function) {
    implementation_->parallelFor(itemCount, minimumItemsPerTask, function);
}

unsigned int WorkerPool::workerCount() const noexcept {
    return implementation_->workerCount();
}

} // namespace orbitlab
