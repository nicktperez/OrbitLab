#pragma once

#include <cmath>
#include <stdexcept>

namespace orbitlab {

struct Vec2 {
    double x{0.0};
    double y{0.0};

    [[nodiscard]] constexpr Vec2 operator+(const Vec2& other) const noexcept {
        return {x + other.x, y + other.y};
    }
    [[nodiscard]] constexpr Vec2 operator-(const Vec2& other) const noexcept {
        return {x - other.x, y - other.y};
    }
    [[nodiscard]] constexpr Vec2 operator*(double scalar) const noexcept {
        return {x * scalar, y * scalar};
    }
    [[nodiscard]] constexpr Vec2 operator/(double scalar) const {
        if (scalar == 0.0) {
            throw std::domain_error("Cannot divide a vector by zero");
        }
        return {x / scalar, y / scalar};
    }
    constexpr Vec2& operator+=(const Vec2& other) noexcept {
        x += other.x;
        y += other.y;
        return *this;
    }
    constexpr Vec2& operator-=(const Vec2& other) noexcept {
        x -= other.x;
        y -= other.y;
        return *this;
    }
    constexpr Vec2& operator*=(double scalar) noexcept {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    [[nodiscard]] constexpr double lengthSquared() const noexcept { return x * x + y * y; }
    [[nodiscard]] double length() const noexcept { return std::sqrt(lengthSquared()); }
    [[nodiscard]] Vec2 normalized() const {
        const auto magnitude = length();
        return magnitude > 0.0 ? *this / magnitude : Vec2{};
    }
    [[nodiscard]] constexpr double dot(const Vec2& other) const noexcept {
        return x * other.x + y * other.y;
    }
    [[nodiscard]] constexpr bool operator==(const Vec2&) const noexcept = default;
    [[nodiscard]] bool isFinite() const noexcept {
        return std::isfinite(x) && std::isfinite(y);
    }
};

[[nodiscard]] constexpr Vec2 operator*(double scalar, const Vec2& value) noexcept {
    return value * scalar;
}

} // namespace orbitlab
