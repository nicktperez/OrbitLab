#include "app/Camera.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace orbitlab;
using namespace orbitlab::app;

TEST_CASE("Perspective camera view-plane transforms round trip", "[camera][3d]") {
    Camera camera;
    const Vec3 world = camera.center() + camera.right() * 1.25 + camera.up() * -0.75;
    const Vec2 screen = camera.worldToScreen(world, 1000.0, 700.0);
    const Vec3 restored = camera.screenToWorld(screen, 1000.0, 700.0);

    REQUIRE(restored.x == Catch::Approx(world.x));
    REQUIRE(restored.y == Catch::Approx(world.y));
    REQUIRE(restored.z == Catch::Approx(world.z));
}

TEST_CASE("Camera zoom keeps the pointed view-plane position anchored", "[camera][3d]") {
    Camera camera;
    const Vec2 pointer{740.0, 210.0};
    const Vec3 before = camera.screenToWorld(pointer, 1000.0, 700.0);

    camera.zoomAt(1.8, pointer, 1000.0, 700.0);

    const Vec3 after = camera.screenToWorld(pointer, 1000.0, 700.0);
    REQUIRE((after - before).length() == Catch::Approx(0.0).margin(1.0e-12));
}

TEST_CASE("Camera projects and unprojects points on an arbitrary drag plane", "[camera][3d]") {
    Camera camera;
    camera.focus({2.0, -1.0, 0.5});
    camera.setAzimuth(0.73);
    camera.setElevation(0.42);
    const Vec3 planePoint = camera.center() + camera.forward() * 0.8;
    const Vec3 world = planePoint + camera.right() * -0.4 + camera.up() * 0.7;
    const Vec2 screen = camera.worldToScreen(world, 960.0, 540.0);
    const Vec3 restored =
        camera.screenToWorldOnPlane(screen, planePoint, 960.0, 540.0);
    REQUIRE((restored - world).length() == Catch::Approx(0.0).margin(1.0e-11));
}

TEST_CASE("Orbit input changes camera orientation without moving its focus", "[camera][3d]") {
    Camera camera;
    const Vec3 focus = camera.center();
    const Vec3 before = camera.position();
    camera.orbitPixels({40.0, -25.0});
    REQUIRE(camera.center() == focus);
    REQUIRE(camera.position() != before);
    REQUIRE(camera.forward().length() == Catch::Approx(1.0));
}
