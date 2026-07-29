#pragma once

#include <SDL3/SDL.h>
#include <SDL3/SDL_gpu.h>

#include <string>

namespace orbitlab::app {

// Owns the native rendering resources and makes the GPU/compatibility fallback
// explicit instead of leaking backend selection throughout Application.
class GpuPlatform {
public:
    explicit GpuPlatform(SDL_Window* window);
    ~GpuPlatform();

    GpuPlatform(const GpuPlatform&) = delete;
    GpuPlatform& operator=(const GpuPlatform&) = delete;

    [[nodiscard]] SDL_Renderer* renderer() const noexcept { return renderer_; }
    [[nodiscard]] SDL_GPUDevice* device() const noexcept { return device_; }
    [[nodiscard]] bool gpuRendererActive() const noexcept { return device_ != nullptr; }
    [[nodiscard]] const std::string& backendName() const noexcept { return backendName_; }
    [[nodiscard]] const std::string& fallbackReason() const noexcept {
        return fallbackReason_;
    }

private:
    SDL_Renderer* renderer_{nullptr};
    SDL_GPUDevice* device_{nullptr};
    std::string backendName_;
    std::string fallbackReason_;
};

} // namespace orbitlab::app
