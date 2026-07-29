#include "app/GpuPlatform.hpp"

#include <stdexcept>

namespace orbitlab::app {

GpuPlatform::GpuPlatform(SDL_Window* window) {
    constexpr SDL_GPUShaderFormat formats = static_cast<SDL_GPUShaderFormat>(
        SDL_GPU_SHADERFORMAT_SPIRV | SDL_GPU_SHADERFORMAT_DXIL |
        SDL_GPU_SHADERFORMAT_MSL);
    device_ = SDL_CreateGPUDevice(formats, false, nullptr);
    if (device_ != nullptr) {
        renderer_ = SDL_CreateGPURenderer(device_, window);
        if (renderer_ != nullptr) {
            const char* driver = SDL_GetGPUDeviceDriver(device_);
            backendName_ = driver != nullptr ? driver : "SDL GPU";
            return;
        }
        fallbackReason_ = std::string{"SDL GPU renderer unavailable: "} + SDL_GetError();
        SDL_DestroyGPUDevice(device_);
        device_ = nullptr;
    } else {
        fallbackReason_ = std::string{"SDL GPU device unavailable: "} + SDL_GetError();
    }

    renderer_ = SDL_CreateRenderer(window, nullptr);
    if (renderer_ == nullptr) {
        throw std::runtime_error(
            std::string{"Could not create a compatible SDL renderer: "} + SDL_GetError());
    }
    const char* name = SDL_GetRendererName(renderer_);
    backendName_ = name != nullptr ? name : "SDL renderer";
}

GpuPlatform::~GpuPlatform() {
    if (renderer_ != nullptr) {
        SDL_DestroyRenderer(renderer_);
    }
    if (device_ != nullptr) {
        SDL_DestroyGPUDevice(device_);
    }
}

} // namespace orbitlab::app
