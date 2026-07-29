#include "app/Camera.hpp"

#include <algorithm>
#include <cmath>

namespace orbitlab::app {

double Camera::focalLength(const double height) const noexcept {
    return height * 0.5 / std::tan(fieldOfViewRadians_ * 0.5);
}

Vec3 Camera::position() const noexcept {
    const Vec3 back{
        std::cos(elevation_) * std::cos(azimuth_),
        std::cos(elevation_) * std::sin(azimuth_),
        std::sin(elevation_),
    };
    return center_ + back * distance_;
}

Vec3 Camera::forward() const noexcept {
    const double cosineElevation = std::cos(elevation_);
    return {
        -cosineElevation * std::cos(azimuth_),
        -cosineElevation * std::sin(azimuth_),
        -std::sin(elevation_),
    };
}

Vec3 Camera::right() const noexcept {
    return {-std::sin(azimuth_), std::cos(azimuth_), 0.0};
}

Vec3 Camera::up() const noexcept {
    const double sineElevation = std::sin(elevation_);
    return {
        -sineElevation * std::cos(azimuth_),
        -sineElevation * std::sin(azimuth_),
        std::cos(elevation_),
    };
}

std::optional<ProjectedPoint> Camera::project(
    const Vec3& world,
    const double width,
    const double height) const noexcept {
    const Vec3 relative = world - position();
    const double depth = relative.dot(forward());
    constexpr double nearPlane = 1.0e-4;
    if (depth <= nearPlane) {
        return std::nullopt;
    }
    const double focal = focalLength(height);
    const double scale = focal / depth;
    return ProjectedPoint{
        {
            width * 0.5 + relative.dot(right()) * scale,
            height * 0.5 - relative.dot(up()) * scale,
        },
        depth,
        scale,
    };
}

Vec2 Camera::worldToScreen(
    const Vec3& world,
    const double width,
    const double height) const noexcept {
    if (const auto projected = project(world, width, height)) {
        return projected->screen;
    }
    return {-1.0e9, -1.0e9};
}

Vec3 Camera::rayDirection(
    const Vec2& screen,
    const double width,
    const double height) const noexcept {
    const double focal = focalLength(height);
    const Vec3 direction =
        forward() + right() * ((screen.x - width * 0.5) / focal) +
        up() * (-(screen.y - height * 0.5) / focal);
    return direction * (1.0 / std::sqrt(direction.lengthSquared()));
}

Vec3 Camera::screenToWorldOnPlane(
    const Vec2& screen,
    const Vec3& planePoint,
    const double width,
    const double height) const noexcept {
    const Vec3 origin = position();
    const Vec3 direction = rayDirection(screen, width, height);
    const Vec3 normal = forward();
    const double denominator = direction.dot(normal);
    if (std::abs(denominator) <= 1.0e-12) {
        return planePoint;
    }
    const double distanceAlongRay = (planePoint - origin).dot(normal) / denominator;
    return origin + direction * distanceAlongRay;
}

Vec3 Camera::screenToWorld(
    const Vec2& screen,
    const double width,
    const double height) const noexcept {
    return screenToWorldOnPlane(screen, center_, width, height);
}

void Camera::panPixels(
    const Vec2& delta,
    const double /*width*/,
    const double height) noexcept {
    const double worldPerPixel = distance_ / focalLength(height);
    center_ += right() * (-delta.x * worldPerPixel) +
               up() * (delta.y * worldPerPixel);
}

void Camera::orbitPixels(const Vec2& delta) noexcept {
    setAzimuth(azimuth_ - delta.x * 0.006);
    setElevation(elevation_ + delta.y * 0.006);
}

void Camera::zoomAt(
    const double factor,
    const Vec2& screenPoint,
    const double width,
    const double height) noexcept {
    const Vec3 before = screenToWorld(screenPoint, width, height);
    distance_ = std::clamp(distance_ / factor, 0.05, 2'000.0);
    const Vec3 after = screenToWorld(screenPoint, width, height);
    center_ += before - after;
}

void Camera::setElevation(const double radians) noexcept {
    constexpr double limit = 1.5533430342749532;
    elevation_ = std::clamp(radians, -limit, limit);
}

void Camera::setAzimuth(const double radians) noexcept {
    constexpr double fullTurn = 6.2831853071795865;
    azimuth_ = std::remainder(radians, fullTurn);
}

double Camera::pixelsPerUnit(const double height) const noexcept {
    return focalLength(height) / distance_;
}

void Camera::reset() noexcept {
    center_ = {};
    distance_ = 4.0;
    azimuth_ = -1.5707963267948966;
    elevation_ = 1.0;
}

} // namespace orbitlab::app
