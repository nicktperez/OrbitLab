#include "orbitlab/GravitySolver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>

namespace orbitlab {
namespace {

struct OctreeNode {
    Vec3 center{};
    double halfSize{1.0};
    double totalMass{0.0};
    Vec3 centerOfMass{};
    std::vector<std::size_t> bodyIndices;
    std::array<std::unique_ptr<OctreeNode>, 8> children;

    [[nodiscard]] bool isLeaf() const noexcept { return children[0] == nullptr; }
    [[nodiscard]] bool contains(const Vec3& point) const noexcept {
        return std::abs(point.x - center.x) <= halfSize &&
               std::abs(point.y - center.y) <= halfSize &&
               std::abs(point.z - center.z) <= halfSize;
    }
};

std::size_t octant(const OctreeNode& node, const Vec3& position) {
    return (position.x >= node.center.x ? 1U : 0U) |
           (position.y >= node.center.y ? 2U : 0U) |
           (position.z >= node.center.z ? 4U : 0U);
}

void subdivide(OctreeNode& node) {
    const double childHalfSize = node.halfSize * 0.5;
    for (std::size_t index = 0; index < node.children.size(); ++index) {
        const double xSign = (index & 1U) != 0 ? 1.0 : -1.0;
        const double ySign = (index & 2U) != 0 ? 1.0 : -1.0;
        const double zSign = (index & 4U) != 0 ? 1.0 : -1.0;
        node.children[index] = std::make_unique<OctreeNode>();
        node.children[index]->center =
            node.center + Vec3{xSign, ySign, zSign} * childHalfSize;
        node.children[index]->halfSize = childHalfSize;
    }
}

void insertBody(
    OctreeNode& node,
    const std::span<const Body> bodies,
    const std::size_t bodyIndex,
    const int depth = 0) {
    const Body& body = bodies[bodyIndex];
    const double previousMass = node.totalMass;
    node.totalMass += body.mass;
    node.centerOfMass =
        previousMass > 0.0
            ? (node.centerOfMass * previousMass + body.position * body.mass) /
                  node.totalMass
            : body.position;

    constexpr int maximumDepth = 56;
    if (node.isLeaf() &&
        (node.bodyIndices.empty() || depth >= maximumDepth ||
         node.halfSize <= std::numeric_limits<double>::epsilon())) {
        node.bodyIndices.push_back(bodyIndex);
        return;
    }
    if (node.isLeaf()) {
        const auto existing = std::move(node.bodyIndices);
        subdivide(node);
        for (const auto existingIndex : existing) {
            insertBody(
                *node.children[octant(node, bodies[existingIndex].position)],
                bodies,
                existingIndex,
                depth + 1);
        }
    }
    insertBody(
        *node.children[octant(node, body.position)], bodies, bodyIndex, depth + 1);
}

Vec3 accelerationFromNode(
    const OctreeNode& node,
    const std::span<const Body> bodies,
    const std::size_t targetIndex,
    const double gravitationalConstant,
    const double softeningSquared,
    const double openingAngle) {
    if (node.totalMass == 0.0) {
        return {};
    }
    const Body& target = bodies[targetIndex];
    if (node.isLeaf()) {
        Vec3 result{};
        for (const auto bodyIndex : node.bodyIndices) {
            if (bodyIndex == targetIndex) {
                continue;
            }
            const Vec3 displacement = bodies[bodyIndex].position - target.position;
            const double distanceSquared =
                displacement.lengthSquared() + softeningSquared;
            if (distanceSquared == 0.0) {
                continue;
            }
            const double inverseDistance = 1.0 / std::sqrt(distanceSquared);
            result += displacement *
                      (gravitationalConstant * bodies[bodyIndex].mass *
                       inverseDistance * inverseDistance * inverseDistance);
        }
        return result;
    }

    const Vec3 displacement = node.centerOfMass - target.position;
    const double geometricDistanceSquared = displacement.lengthSquared();
    const double geometricDistance = std::sqrt(geometricDistanceSquared);
    if (!node.contains(target.position) && geometricDistance > 0.0 &&
        (node.halfSize * 2.0) / geometricDistance < openingAngle) {
        const double softenedDistanceSquared =
            geometricDistanceSquared + softeningSquared;
        const double inverseDistance = 1.0 / std::sqrt(softenedDistanceSquared);
        return displacement *
               (gravitationalConstant * node.totalMass * inverseDistance *
                inverseDistance * inverseDistance);
    }

    Vec3 result{};
    for (const auto& child : node.children) {
        result += accelerationFromNode(
            *child,
            bodies,
            targetIndex,
            gravitationalConstant,
            softeningSquared,
            openingAngle);
    }
    return result;
}

OctreeNode makeRoot(const std::span<const Body> bodies) {
    Vec3 minimum{
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),
    };
    Vec3 maximum{
        std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest(),
    };
    for (const auto& body : bodies) {
        minimum.x = std::min(minimum.x, body.position.x);
        minimum.y = std::min(minimum.y, body.position.y);
        minimum.z = std::min(minimum.z, body.position.z);
        maximum.x = std::max(maximum.x, body.position.x);
        maximum.y = std::max(maximum.y, body.position.y);
        maximum.z = std::max(maximum.z, body.position.z);
    }
    OctreeNode root;
    root.center = (minimum + maximum) * 0.5;
    root.halfSize =
        std::max({maximum.x - minimum.x, maximum.y - minimum.y,
                  maximum.z - minimum.z, 1.0e-9}) *
        0.500001;
    return root;
}

} // namespace

BarnesHutGravitySolver::BarnesHutGravitySolver(const double openingAngle)
    : openingAngle_(openingAngle) {
    if (!std::isfinite(openingAngle_) || openingAngle_ <= 0.0 ||
        openingAngle_ > 2.0) {
        throw std::invalid_argument(
            "Barnes-Hut opening angle must be in the range (0, 2]");
    }
}

std::vector<Vec3> BarnesHutGravitySolver::accelerations(
    const std::span<const Body> bodies,
    const double gravitationalConstant,
    const double softeningLength) const {
    if (!std::isfinite(gravitationalConstant) || gravitationalConstant <= 0.0) {
        throw std::invalid_argument("Gravitational constant must be finite and positive");
    }
    if (!std::isfinite(softeningLength) || softeningLength < 0.0) {
        throw std::invalid_argument("Softening length must be finite and non-negative");
    }
    std::vector<Vec3> result(bodies.size());
    if (bodies.empty()) {
        return result;
    }
    auto root = makeRoot(bodies);
    for (std::size_t index = 0; index < bodies.size(); ++index) {
        insertBody(root, bodies, index);
    }
    const double softeningSquared = softeningLength * softeningLength;
    for (std::size_t index = 0; index < bodies.size(); ++index) {
        result[index] = accelerationFromNode(
            root,
            bodies,
            index,
            gravitationalConstant,
            softeningSquared,
            openingAngle_);
    }
    return result;
}

} // namespace orbitlab
