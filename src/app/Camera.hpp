#pragma once

#include "orbitlab/Vec2.hpp"
#include "orbitlab/Vec3.hpp"

#include <optional>

namespace orbitlab::app {

struct ProjectedPoint {
    Vec2 screen{};
    double depth{0.0};
    double pixelsPerUnit{0.0};
};

class Camera {
public:
    [[nodiscard]] std::optional<ProjectedPoint> project(
        const Vec3& world,
        double width,
        double height) const noexcept;
    [[nodiscard]] Vec2 worldToScreen(
        const Vec3& world,
        double width,
        double height) const noexcept;
    [[nodiscard]] Vec3 screenToWorld(
        const Vec2& screen,
        double width,
        double height) const noexcept;
    [[nodiscard]] Vec3 screenToWorldOnPlane(
        const Vec2& screen,
        const Vec3& planePoint,
        double width,
        double height) const noexcept;

    void panPixels(
        const Vec2& delta,
        double width,
        double height) noexcept;
    void orbitPixels(const Vec2& delta) noexcept;
    void zoomAt(
        double factor,
        const Vec2& screenPoint,
        double width,
        double height) noexcept;
    void reset() noexcept;
    void focus(const Vec3& center) noexcept { center_ = center; }
    void setAzimuth(double radians) noexcept;
    void setElevation(double radians) noexcept;

    [[nodiscard]] double pixelsPerUnit(double height = 900.0) const noexcept;
    [[nodiscard]] const Vec3& center() const noexcept { return center_; }
    [[nodiscard]] double azimuth() const noexcept { return azimuth_; }
    [[nodiscard]] double elevation() const noexcept { return elevation_; }
    [[nodiscard]] double distance() const noexcept { return distance_; }
    [[nodiscard]] Vec3 forward() const noexcept;
    [[nodiscard]] Vec3 right() const noexcept;
    [[nodiscard]] Vec3 up() const noexcept;
    [[nodiscard]] Vec3 position() const noexcept;

private:
    [[nodiscard]] double focalLength(double height) const noexcept;
    [[nodiscard]] Vec3 rayDirection(
        const Vec2& screen,
        double width,
        double height) const noexcept;

    Vec3 center_{};
    double distance_{4.0};
    double azimuth_{-1.5707963267948966};
    double elevation_{1.0};
    double fieldOfViewRadians_{0.8028514559173916};
};

} // namespace orbitlab::app
