#pragma once

#include "orbitlab/Vec3.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace orbitlab {

struct Color {
    float r{1.0F};
    float g{1.0F};
    float b{1.0F};
    float a{1.0F};
    [[nodiscard]] constexpr bool operator==(const Color&) const noexcept = default;
};

struct Body {
    std::uint64_t id{0};
    std::string name{"Body"};
    double mass{1.0};
    double radius{0.02};
    Color color{};
    Vec3 position{};
    Vec3 velocity{};

    [[nodiscard]] bool isValid() const noexcept {
        return id != 0 && !name.empty() && std::isfinite(mass) && mass > 0.0 &&
               std::isfinite(radius) && radius > 0.0 && position.isFinite() &&
               velocity.isFinite();
    }
};

} // namespace orbitlab
