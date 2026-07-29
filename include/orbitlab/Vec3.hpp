#pragma once

#include <cmath>
#include <stdexcept>

namespace orbitlab {

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    [[nodiscard]] constexpr Vec3 operator+(const Vec3& other) const noexcept {
        return {x + other.x, y + other.y, z + other.z};
    }
    [[nodiscard]] constexpr Vec3 operator-(const Vec3& other) const noexcept {
        return {x - other.x, y - other.y, z - other.z};
    }
    [[nodiscard]] constexpr Vec3 operator-() const noexcept { return {-x, -y, -z}; }
    [[nodiscard]] constexpr Vec3 operator*(double scalar) const noexcept {
        return {x * scalar, y * scalar, z * scalar};
    }
    [[nodiscard]] constexpr Vec3 operator/(double scalar) const {
        if (scalar == 0.0) {
            throw std::domain_error("Cannot divide a vector by zero");
        }
        return {x / scalar, y / scalar, z / scalar};
    }
    constexpr Vec3& operator+=(const Vec3& other) noexcept {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }
    constexpr Vec3& operator-=(const Vec3& other) noexcept {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }
    constexpr Vec3& operator*=(double scalar) noexcept {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    [[nodiscard]] constexpr double lengthSquared() const noexcept {
        return x * x + y * y + z * z;
    }
    [[nodiscard]] double length() const noexcept { return std::sqrt(lengthSquared()); }
    [[nodiscard]] Vec3 normalized() const {
        const auto magnitude = length();
        return magnitude > 0.0 ? *this / magnitude : Vec3{};
    }
    [[nodiscard]] constexpr double dot(const Vec3& other) const noexcept {
        return x * other.x + y * other.y + z * other.z;
    }
    [[nodiscard]] constexpr Vec3 cross(const Vec3& other) const noexcept {
        return {
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x,
        };
    }
    [[nodiscard]] constexpr bool operator==(const Vec3&) const noexcept = default;
    [[nodiscard]] bool isFinite() const noexcept {
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }
};

[[nodiscard]] constexpr Vec3 operator*(double scalar, const Vec3& value) noexcept {
    return value * scalar;
}

} // namespace orbitlab
