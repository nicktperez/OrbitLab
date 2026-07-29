#include "app/Renderer.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

namespace orbitlab::app {
namespace {

void setColor(SDL_Renderer* renderer, const Color& color, const float alpha = 1.0F) {
    SDL_SetRenderDrawColorFloat(
        renderer, color.r, color.g, color.b, std::clamp(color.a * alpha, 0.0F, 1.0F));
}

} // namespace

void Renderer::draw(
    const Simulation& simulation,
    const Camera& camera,
    const TrailMap& trails,
    const PredictionPath& prediction,
    const AppState& state,
    const int width,
    const int height,
    const float panelWidth) {
    SDL_SetRenderDrawColor(renderer_, 9, 10, 11, 255);
    SDL_RenderClear(renderer_);
    SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_BLEND);

    const double canvasWidth = static_cast<double>(width) - panelWidth;
    const double canvasHeight = static_cast<double>(height);
    SDL_Rect clip{
        static_cast<int>(panelWidth),
        0,
        std::max(0, width - static_cast<int>(panelWidth)),
        height,
    };
    SDL_SetRenderClipRect(renderer_, &clip);

    if (state.creatingBody) {
        Vec2 start =
            camera.worldToScreen(state.creationStart, canvasWidth, canvasHeight);
        Vec2 current =
            camera.worldToScreen(state.creationCurrent, canvasWidth, canvasHeight);
        start.x += panelWidth;
        current.x += panelWidth;
        setColor(renderer_, {0.53F, 0.84F, 0.67F, 1.0F}, 0.9F);
        SDL_RenderLine(
            renderer_,
            static_cast<float>(start.x),
            static_cast<float>(start.y),
            static_cast<float>(current.x),
            static_cast<float>(current.y));
        drawCircle(
            static_cast<float>(start.x),
            static_cast<float>(start.y),
            5.0F,
            {0.53F, 0.84F, 0.67F, 1.0F},
            true);
        drawCircle(
            static_cast<float>(current.x),
            static_cast<float>(current.y),
            5.0F,
            {0.53F, 0.84F, 0.67F, 1.0F},
            false);
    }

    if (state.showPrediction && prediction.size() > 1) {
        setColor(renderer_, {0.48F, 0.88F, 0.70F, 1.0F}, 0.48F);
        for (std::size_t index = 1; index < prediction.size(); ++index) {
            if (index % 3 == 0) {
                continue;
            }
            const auto firstProjection =
                camera.project(prediction[index - 1], canvasWidth, canvasHeight);
            const auto secondProjection =
                camera.project(prediction[index], canvasWidth, canvasHeight);
            if (!firstProjection || !secondProjection) {
                continue;
            }
            const Vec2 first = firstProjection->screen;
            const Vec2 second = secondProjection->screen;
            SDL_RenderLine(
                renderer_,
                static_cast<float>(first.x + panelWidth),
                static_cast<float>(first.y),
                static_cast<float>(second.x + panelWidth),
                static_cast<float>(second.y));
        }
    }

    if (simulation.settings().trailsEnabled) {
        for (const auto& body : simulation.bodies()) {
            const auto trail = trails.find(body.id);
            if (trail == trails.end() || trail->second.size() < 2) {
                continue;
            }
            setColor(renderer_, body.color, 0.36F);
            for (std::size_t index = 1; index < trail->second.size(); ++index) {
                const auto firstProjection =
                    camera.project(trail->second[index - 1], canvasWidth, canvasHeight);
                const auto secondProjection =
                    camera.project(trail->second[index], canvasWidth, canvasHeight);
                if (!firstProjection || !secondProjection) {
                    continue;
                }
                const Vec2 first = firstProjection->screen;
                const Vec2 second = secondProjection->screen;
                SDL_RenderLine(
                    renderer_,
                    static_cast<float>(first.x + panelWidth),
                    static_cast<float>(first.y),
                    static_cast<float>(second.x + panelWidth),
                    static_cast<float>(second.y));
            }
        }
    }

    struct VisibleBody {
        const Body* body;
        ProjectedPoint projection;
    };
    std::vector<VisibleBody> visibleBodies;
    visibleBodies.reserve(simulation.bodies().size());
    for (const auto& body : simulation.bodies()) {
        if (const auto projection =
                camera.project(body.position, canvasWidth, canvasHeight)) {
            visibleBodies.push_back({&body, *projection});
        }
    }
    std::ranges::sort(
        visibleBodies,
        std::greater{},
        [](const VisibleBody& visible) { return visible.projection.depth; });
    for (const auto& visible : visibleBodies) {
        const Body& body = *visible.body;
        Vec2 screen = visible.projection.screen;
        screen.x += panelWidth;
        const float visualRadius = std::clamp(
            static_cast<float>(body.radius * visible.projection.pixelsPerUnit),
            3.5F,
            38.0F);
        drawCircle(
            static_cast<float>(screen.x),
            static_cast<float>(screen.y),
            visualRadius,
            body.color,
            true);
        if (state.selectedBodyId == body.id) {
            drawCircle(
                static_cast<float>(screen.x),
                static_cast<float>(screen.y),
                visualRadius + 4.0F,
                {0.48F, 0.88F, 0.70F, 1.0F},
                false);
            drawVelocityHandle(body, camera, canvasWidth, canvasHeight, panelWidth);
        } else if (state.hoveredBodyId == body.id) {
            drawCircle(
                static_cast<float>(screen.x),
                static_cast<float>(screen.y),
                visualRadius + 3.0F,
                {0.67F, 0.73F, 0.71F, 1.0F},
                false);
        }
    }

    SDL_SetRenderClipRect(renderer_, nullptr);
}

void Renderer::drawVelocityHandle(
    const Body& body,
    const Camera& camera,
    const double canvasWidth,
    const double canvasHeight,
    const float panelWidth) {
    const auto startProjection =
        camera.project(body.position, canvasWidth, canvasHeight);
    const auto endProjection = camera.project(
        body.position + velocityHandleOffset(body, camera), canvasWidth, canvasHeight);
    if (!startProjection || !endProjection) {
        return;
    }
    Vec2 start = startProjection->screen;
    Vec2 end = endProjection->screen;
    start.x += panelWidth;
    end.x += panelWidth;
    setColor(renderer_, {0.48F, 0.88F, 0.70F, 1.0F}, 0.9F);
    SDL_RenderLine(
        renderer_,
        static_cast<float>(start.x),
        static_cast<float>(start.y),
        static_cast<float>(end.x),
        static_cast<float>(end.y));

    const Vec2 direction = (end - start).normalized();
    const Vec2 perpendicular{-direction.y, direction.x};
    constexpr double arrowLength = 8.0;
    const Vec2 arrowBase = end - direction * arrowLength;
    SDL_RenderLine(
        renderer_,
        static_cast<float>(end.x),
        static_cast<float>(end.y),
        static_cast<float>(arrowBase.x + perpendicular.x * 4.0),
        static_cast<float>(arrowBase.y + perpendicular.y * 4.0));
    SDL_RenderLine(
        renderer_,
        static_cast<float>(end.x),
        static_cast<float>(end.y),
        static_cast<float>(arrowBase.x - perpendicular.x * 4.0),
        static_cast<float>(arrowBase.y - perpendicular.y * 4.0));
    drawCircle(
        static_cast<float>(end.x),
        static_cast<float>(end.y),
        5.0F,
        {0.48F, 0.88F, 0.70F, 1.0F},
        false);
}

void Renderer::drawCircle(
    const float x,
    const float y,
    const float radius,
    const Color& color,
    const bool filled) {
    setColor(renderer_, color);
    if (filled) {
        const int extent = static_cast<int>(std::ceil(radius));
        for (int offset = -extent; offset <= extent; ++offset) {
            const float horizontal =
                std::sqrt(std::max(0.0F, radius * radius - static_cast<float>(offset * offset)));
            SDL_RenderLine(renderer_, x - horizontal, y + static_cast<float>(offset),
                           x + horizontal, y + static_cast<float>(offset));
        }
        return;
    }

    constexpr int segments = 48;
    constexpr float fullCircle = 6.28318530718F;
    for (int index = 0; index < segments; ++index) {
        const float firstAngle = fullCircle * static_cast<float>(index) / segments;
        const float secondAngle = fullCircle * static_cast<float>(index + 1) / segments;
        SDL_RenderLine(
            renderer_, x + std::cos(firstAngle) * radius, y + std::sin(firstAngle) * radius,
            x + std::cos(secondAngle) * radius, y + std::sin(secondAngle) * radius);
    }
}

} // namespace orbitlab::app
