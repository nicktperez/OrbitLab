#pragma once

#include "orbitlab/GravitySolver.hpp"

#include <SDL3/SDL_gpu.h>

#include <cstddef>
#include <string>

namespace orbitlab::app {

class GpuGravitySolver final : public GravitySolver {
public:
    explicit GpuGravitySolver(SDL_GPUDevice* device);
    ~GpuGravitySolver() override;

    GpuGravitySolver(const GpuGravitySolver&) = delete;
    GpuGravitySolver& operator=(const GpuGravitySolver&) = delete;

    [[nodiscard]] std::vector<Vec3> accelerations(
        std::span<const Body> bodies,
        double gravitationalConstant,
        double softeningLength) const override;

    [[nodiscard]] bool available() const noexcept { return pipeline_ != nullptr; }
    [[nodiscard]] const std::string& status() const noexcept { return status_; }

private:
    void ensureCapacity(std::size_t bodyCount) const;
    void releaseBuffers() const noexcept;

    SDL_GPUDevice* device_{nullptr};
    SDL_GPUComputePipeline* pipeline_{nullptr};
    mutable SDL_GPUBuffer* inputBuffer_{nullptr};
    mutable SDL_GPUBuffer* outputBuffer_{nullptr};
    mutable SDL_GPUTransferBuffer* uploadBuffer_{nullptr};
    mutable SDL_GPUTransferBuffer* downloadBuffer_{nullptr};
    mutable std::size_t capacity_{0};
    mutable bool runtimeFailed_{false};
    mutable std::string status_;
};

} // namespace orbitlab::app
