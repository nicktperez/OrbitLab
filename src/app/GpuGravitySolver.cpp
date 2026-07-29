#include "app/GpuGravitySolver.hpp"

#include <SDL3/SDL.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

namespace orbitlab::app {
namespace {

constexpr char gravityShaderMsl[] = R"msl(
#include <metal_stdlib>
using namespace metal;

struct Parameters {
    uint bodyCount;
    float gravitationalConstant;
    float softeningSquared;
    uint padding;
};

kernel void nbody_accelerations(
    constant Parameters& parameters [[buffer(0)]],
    device const float4* bodies [[buffer(1)]],
    device float4* accelerations [[buffer(2)]],
    uint index [[thread_position_in_grid]]) {
    if (index >= parameters.bodyCount) {
        return;
    }
    const float3 origin = bodies[index].xyz;
    float3 acceleration = float3(0.0f);
    for (uint other = 0; other < parameters.bodyCount; ++other) {
        if (other == index) {
            continue;
        }
        const float3 delta = bodies[other].xyz - origin;
        const float distanceSquared =
            dot(delta, delta) + parameters.softeningSquared;
        const float inverseDistance = rsqrt(distanceSquared);
        const float scale = parameters.gravitationalConstant *
            bodies[other].w * inverseDistance * inverseDistance * inverseDistance;
        acceleration += delta * scale;
    }
    accelerations[index] = float4(acceleration, 0.0f);
}
)msl";

struct alignas(16) GpuBody {
    float x;
    float y;
    float z;
    float mass;
};

struct alignas(16) GpuParameters {
    std::uint32_t bodyCount;
    float gravitationalConstant;
    float softeningSquared;
    std::uint32_t padding;
};

std::string sdlError(const char* action) {
    return std::string{action} + ": " + SDL_GetError();
}

} // namespace

GpuGravitySolver::GpuGravitySolver(SDL_GPUDevice* device) : device_(device) {
    if (device_ == nullptr) {
        status_ = "No SDL GPU device is active";
        return;
    }
    if ((SDL_GetGPUShaderFormats(device_) & SDL_GPU_SHADERFORMAT_MSL) == 0) {
        status_ =
            "Compute shader unavailable for this driver; CPU solver remains active";
        return;
    }

    SDL_GPUComputePipelineCreateInfo createInfo{};
    createInfo.code = reinterpret_cast<const Uint8*>(gravityShaderMsl);
    createInfo.code_size = sizeof(gravityShaderMsl);
    createInfo.entrypoint = "nbody_accelerations";
    createInfo.format = SDL_GPU_SHADERFORMAT_MSL;
    createInfo.num_readonly_storage_buffers = 1;
    createInfo.num_readwrite_storage_buffers = 1;
    createInfo.num_uniform_buffers = 1;
    createInfo.threadcount_x = 64;
    createInfo.threadcount_y = 1;
    createInfo.threadcount_z = 1;
    pipeline_ = SDL_CreateGPUComputePipeline(device_, &createInfo);
    status_ = pipeline_ != nullptr
                  ? "Ready (64-thread direct N-body kernel)"
                  : sdlError("Could not create the gravity compute pipeline");
}

GpuGravitySolver::~GpuGravitySolver() {
    releaseBuffers();
    if (pipeline_ != nullptr) {
        SDL_ReleaseGPUComputePipeline(device_, pipeline_);
    }
}

void GpuGravitySolver::releaseBuffers() const noexcept {
    if (inputBuffer_ != nullptr) {
        SDL_ReleaseGPUBuffer(device_, inputBuffer_);
    }
    if (outputBuffer_ != nullptr) {
        SDL_ReleaseGPUBuffer(device_, outputBuffer_);
    }
    if (uploadBuffer_ != nullptr) {
        SDL_ReleaseGPUTransferBuffer(device_, uploadBuffer_);
    }
    if (downloadBuffer_ != nullptr) {
        SDL_ReleaseGPUTransferBuffer(device_, downloadBuffer_);
    }
    inputBuffer_ = nullptr;
    outputBuffer_ = nullptr;
    uploadBuffer_ = nullptr;
    downloadBuffer_ = nullptr;
    capacity_ = 0;
}

void GpuGravitySolver::ensureCapacity(const std::size_t bodyCount) const {
    if (bodyCount <= capacity_) {
        return;
    }
    releaseBuffers();
    capacity_ = std::max<std::size_t>(bodyCount, 64);
    const std::size_t byteCount = capacity_ * sizeof(GpuBody);
    if (byteCount > std::numeric_limits<Uint32>::max()) {
        throw std::overflow_error("GPU solver buffer size exceeds SDL limits");
    }

    SDL_GPUBufferCreateInfo inputInfo{};
    inputInfo.usage = SDL_GPU_BUFFERUSAGE_COMPUTE_STORAGE_READ;
    inputInfo.size = static_cast<Uint32>(byteCount);
    inputBuffer_ = SDL_CreateGPUBuffer(device_, &inputInfo);
    SDL_GPUBufferCreateInfo outputInfo{};
    outputInfo.usage = SDL_GPU_BUFFERUSAGE_COMPUTE_STORAGE_WRITE;
    outputInfo.size = static_cast<Uint32>(byteCount);
    outputBuffer_ = SDL_CreateGPUBuffer(device_, &outputInfo);
    SDL_GPUTransferBufferCreateInfo uploadInfo{};
    uploadInfo.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
    uploadInfo.size = static_cast<Uint32>(byteCount);
    uploadBuffer_ = SDL_CreateGPUTransferBuffer(device_, &uploadInfo);
    SDL_GPUTransferBufferCreateInfo downloadInfo{};
    downloadInfo.usage = SDL_GPU_TRANSFERBUFFERUSAGE_DOWNLOAD;
    downloadInfo.size = static_cast<Uint32>(byteCount);
    downloadBuffer_ = SDL_CreateGPUTransferBuffer(device_, &downloadInfo);
    if (inputBuffer_ == nullptr || outputBuffer_ == nullptr ||
        uploadBuffer_ == nullptr || downloadBuffer_ == nullptr) {
        releaseBuffers();
        throw std::runtime_error(sdlError("Could not allocate GPU gravity buffers"));
    }
}

std::vector<Vec3> GpuGravitySolver::accelerations(
    const std::span<const Body> bodies,
    const double gravitationalConstant,
    const double softeningLength) const {
    if (pipeline_ == nullptr || runtimeFailed_) {
        throw std::runtime_error(status_);
    }
    if (bodies.empty()) {
        return {};
    }
    if (bodies.size() > std::numeric_limits<std::uint32_t>::max() ||
        !std::isfinite(gravitationalConstant) ||
        !std::isfinite(softeningLength)) {
        throw std::invalid_argument("GPU solver received unsafe numeric input");
    }
    constexpr double floatLimit = std::numeric_limits<float>::max();
    if (std::abs(gravitationalConstant) > floatLimit ||
        std::abs(softeningLength) > std::sqrt(floatLimit) ||
        std::ranges::any_of(bodies, [](const Body& body) {
            constexpr double limit = std::numeric_limits<float>::max();
            return body.mass > limit || std::abs(body.position.x) > limit ||
                   std::abs(body.position.y) > limit ||
                   std::abs(body.position.z) > limit;
        })) {
        status_ = "CPU fallback active: current state exceeds float GPU range";
        return DirectGravitySolver{}.accelerations(
            bodies, gravitationalConstant, softeningLength);
    }

    try {
        ensureCapacity(bodies.size());
        auto* mapped = static_cast<GpuBody*>(
            SDL_MapGPUTransferBuffer(device_, uploadBuffer_, true));
        if (mapped == nullptr) {
            throw std::runtime_error(sdlError("Could not map GPU upload buffer"));
        }
        for (std::size_t index = 0; index < bodies.size(); ++index) {
            const Body& body = bodies[index];
            mapped[index] = {
                static_cast<float>(body.position.x),
                static_cast<float>(body.position.y),
                static_cast<float>(body.position.z),
                static_cast<float>(body.mass)};
        }
        SDL_UnmapGPUTransferBuffer(device_, uploadBuffer_);

        SDL_GPUCommandBuffer* commandBuffer = SDL_AcquireGPUCommandBuffer(device_);
        if (commandBuffer == nullptr) {
            throw std::runtime_error(sdlError("Could not acquire GPU command buffer"));
        }
        const Uint32 byteCount = static_cast<Uint32>(bodies.size() * sizeof(GpuBody));
        SDL_GPUCopyPass* uploadPass = SDL_BeginGPUCopyPass(commandBuffer);
        const SDL_GPUTransferBufferLocation uploadSource{uploadBuffer_, 0};
        const SDL_GPUBufferRegion uploadDestination{inputBuffer_, 0, byteCount};
        SDL_UploadToGPUBuffer(
            uploadPass, &uploadSource, &uploadDestination, true);
        SDL_EndGPUCopyPass(uploadPass);

        const GpuParameters parameters{
            static_cast<std::uint32_t>(bodies.size()),
            static_cast<float>(gravitationalConstant),
            static_cast<float>(softeningLength * softeningLength),
            0};
        SDL_PushGPUComputeUniformData(
            commandBuffer, 0, &parameters, sizeof(parameters));
        const SDL_GPUStorageBufferReadWriteBinding outputBinding{
            outputBuffer_, true, 0, 0, 0};
        SDL_GPUComputePass* computePass = SDL_BeginGPUComputePass(
            commandBuffer, nullptr, 0, &outputBinding, 1);
        SDL_BindGPUComputePipeline(computePass, pipeline_);
        SDL_GPUBuffer* readonlyBuffers[] = {inputBuffer_};
        SDL_BindGPUComputeStorageBuffers(computePass, 0, readonlyBuffers, 1);
        SDL_DispatchGPUCompute(
            computePass,
            static_cast<Uint32>((bodies.size() + 63) / 64),
            1,
            1);
        SDL_EndGPUComputePass(computePass);

        SDL_GPUCopyPass* downloadPass = SDL_BeginGPUCopyPass(commandBuffer);
        const SDL_GPUBufferRegion downloadSource{outputBuffer_, 0, byteCount};
        const SDL_GPUTransferBufferLocation downloadDestination{downloadBuffer_, 0};
        SDL_DownloadFromGPUBuffer(
            downloadPass, &downloadSource, &downloadDestination);
        SDL_EndGPUCopyPass(downloadPass);

        SDL_GPUFence* fence =
            SDL_SubmitGPUCommandBufferAndAcquireFence(commandBuffer);
        if (fence == nullptr) {
            throw std::runtime_error(sdlError("Could not submit GPU gravity work"));
        }
        SDL_GPUFence* fences[] = {fence};
        const bool completed = SDL_WaitForGPUFences(device_, true, fences, 1);
        SDL_ReleaseGPUFence(device_, fence);
        if (!completed) {
            throw std::runtime_error(sdlError("GPU gravity work did not complete"));
        }

        const auto* result = static_cast<const GpuBody*>(
            SDL_MapGPUTransferBuffer(device_, downloadBuffer_, false));
        if (result == nullptr) {
            throw std::runtime_error(sdlError("Could not map GPU download buffer"));
        }
        std::vector<Vec3> output(bodies.size());
        for (std::size_t index = 0; index < bodies.size(); ++index) {
            output[index] = {result[index].x, result[index].y, result[index].z};
        }
        SDL_UnmapGPUTransferBuffer(device_, downloadBuffer_);
        return output;
    } catch (const std::exception& error) {
        runtimeFailed_ = true;
        status_ = std::string{"Compute disabled after runtime failure: "} + error.what();
        return DirectGravitySolver{}.accelerations(
            bodies, gravitationalConstant, softeningLength);
    }
}

} // namespace orbitlab::app
