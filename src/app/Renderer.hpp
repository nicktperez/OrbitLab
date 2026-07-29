#pragma once

#include "app/Camera.hpp"
#include "app/AppState.hpp"
#include "orbitlab/Simulation.hpp"

#include <SDL3/SDL.h>
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <vector>

namespace orbitlab::app {

using TrailMap = std::unordered_map<std::uint64_t, std::deque<Vec3>>;
using PredictionPath = std::vector<Vec3>;
inline constexpr double velocityHandleScale = 0.28;

[[nodiscard]] inline Vec3 velocityHandleOffset(
    const Body& body,
    const Camera& camera) noexcept {
    const Vec3 physicalOffset = body.velocity * velocityHandleScale;
    constexpr double minimumHandlePixels = 34.0;
    if (physicalOffset.length() * camera.pixelsPerUnit() >= minimumHandlePixels) {
        return physicalOffset;
    }
    const Vec3 direction =
        body.velocity.lengthSquared() > 1.0e-20 ? body.velocity.normalized()
                                                : Vec3{1.0, 0.0, 0.0};
    return direction * (minimumHandlePixels / camera.pixelsPerUnit());
}

class Renderer {
public:
    explicit Renderer(SDL_Renderer* renderer) : renderer_(renderer) {}

    void draw(
        const Simulation& simulation,
        const Camera& camera,
        const TrailMap& trails,
        const PredictionPath& prediction,
        const AppState& state,
        int width,
        int height,
        float panelWidth);

private:
    void drawCircle(float x, float y, float radius, const Color& color, bool filled);
    void drawVelocityHandle(
        const Body& body,
        const Camera& camera,
        double canvasWidth,
        double canvasHeight,
        float panelWidth);
    SDL_Renderer* renderer_;
};

} // namespace orbitlab::app
