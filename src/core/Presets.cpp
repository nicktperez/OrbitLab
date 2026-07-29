#include "orbitlab/Presets.hpp"

#include <array>
#include <cmath>
#include <numbers>
#include <random>
#include <stdexcept>

namespace orbitlab {
namespace {

Body makeBody(
    std::string name,
    const double mass,
    const double radius,
    const Color color,
    const Vec3 position,
    const Vec3 velocity) {
    return {0, std::move(name), mass, radius, color, position, velocity};
}

void configureDefaults(Simulation& simulation) {
    simulation.clear();
    simulation.settings() = {};
}

} // namespace

std::string_view presetName(const Preset preset) noexcept {
    switch (preset) {
    case Preset::SunEarth:
        return "Sun and Earth";
    case Preset::EarthMoon:
        return "Earth and Moon";
    case Preset::BinaryStars:
        return "Binary stars";
    case Preset::AsteroidField:
        return "Random asteroid field";
    case Preset::InclinedSystem:
        return "Inclined 3D system";
    }
    return "Unknown";
}

void loadPreset(Simulation& simulation, const Preset preset, const std::uint32_t randomSeed) {
    configureDefaults(simulation);

    switch (preset) {
    case Preset::SunEarth: {
        constexpr double earthMass = 3.003e-6;
        simulation.addBody(makeBody(
            "Sun", 1.0, 0.035, {0.98F, 0.72F, 0.24F, 1.0F},
            {-earthMass, 0.0}, {0.0, -earthMass}));
        simulation.addBody(makeBody(
            "Earth", earthMass, 0.014, {0.25F, 0.56F, 0.95F, 1.0F},
            {1.0, 0.0}, {0.0, 1.0}));
        break;
    }
    case Preset::EarthMoon: {
        constexpr double moonMass = 0.012300;
        constexpr double separation = 1.0;
        const double angularVelocity = std::sqrt(1.0 + moonMass);
        simulation.addBody(makeBody(
            "Earth", 1.0, 0.075, {0.25F, 0.56F, 0.95F, 1.0F},
            {-moonMass / (1.0 + moonMass) * separation, 0.0},
            {0.0, -angularVelocity * moonMass / (1.0 + moonMass) * separation}));
        simulation.addBody(makeBody(
            "Moon", moonMass, 0.035, {0.72F, 0.75F, 0.78F, 1.0F},
            {1.0 / (1.0 + moonMass) * separation, 0.0},
            {0.0, angularVelocity / (1.0 + moonMass) * separation}));
        break;
    }
    case Preset::BinaryStars: {
        const double speed = std::sqrt(0.5);
        simulation.addBody(makeBody(
            "Aster", 1.0, 0.055, {0.98F, 0.66F, 0.28F, 1.0F},
            {-0.5, 0.0}, {0.0, -speed}));
        simulation.addBody(makeBody(
            "Caelum", 1.0, 0.055, {0.48F, 0.72F, 0.98F, 1.0F},
            {0.5, 0.0}, {0.0, speed}));
        break;
    }
    case Preset::AsteroidField: {
        simulation.addBody(makeBody(
            "Helios", 1.0, 0.045, {0.98F, 0.72F, 0.24F, 1.0F},
            {}, {}));
        std::mt19937 generator(randomSeed);
        std::uniform_real_distribution<double> radiusDistribution(0.35, 1.8);
        std::uniform_real_distribution<double> angleDistribution(0.0, 2.0 * std::numbers::pi);
        std::uniform_real_distribution<double> massDistribution(1.0e-9, 5.0e-7);
        std::uniform_real_distribution<float> colorDistribution(0.48F, 0.82F);
        for (int index = 0; index < 96; ++index) {
            const double radius = radiusDistribution(generator);
            const double angle = angleDistribution(generator);
            const Vec3 radial{std::cos(angle), std::sin(angle), 0.0};
            const Vec3 tangent{-radial.y, radial.x, 0.0};
            const double speed = std::sqrt(1.0 / radius);
            simulation.addBody(makeBody(
                "Asteroid " + std::to_string(index + 1),
                massDistribution(generator),
                0.004,
                {colorDistribution(generator), colorDistribution(generator),
                 colorDistribution(generator), 1.0F},
                radial * radius,
                tangent * speed));
        }
        simulation.settings().collisionMode = CollisionMode::None;
        break;
    }
    case Preset::InclinedSystem: {
        simulation.addBody(makeBody(
            "Sol", 1.0, 0.045, {0.98F, 0.72F, 0.24F, 1.0F}, {}, {}));
        const auto addOrbiter = [&](std::string name,
                                    const double radius,
                                    const double mass,
                                    const Color color,
                                    const Vec3 radial,
                                    const Vec3 tangent) {
            const double speed = std::sqrt(1.0 / radius);
            simulation.addBody(makeBody(
                std::move(name),
                mass,
                0.014,
                color,
                radial.normalized() * radius,
                tangent.normalized() * speed));
        };
        addOrbiter(
            "Aurelia",
            0.62,
            2.0e-6,
            {0.92F, 0.48F, 0.30F, 1.0F},
            {1.0, 0.0, 0.0},
            {0.0, std::cos(0.55), std::sin(0.55)});
        addOrbiter(
            "Pelagos",
            1.0,
            3.0e-6,
            {0.25F, 0.63F, 0.95F, 1.0F},
            {0.0, 1.0, 0.0},
            {-std::cos(0.82), 0.0, std::sin(0.82)});
        const Vec3 outerRadial = Vec3{1.0, 1.0, 0.72}.normalized();
        addOrbiter(
            "Viridia",
            1.55,
            1.2e-6,
            {0.42F, 0.82F, 0.57F, 1.0F},
            outerRadial,
            Vec3{0.0, 0.0, 1.0}.cross(outerRadial));
        simulation.settings().collisionMode = CollisionMode::None;
        break;
    }
    }
}

} // namespace orbitlab
