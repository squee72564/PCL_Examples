#include "Box.hpp"
#include <iostream>
#include <limits>
#include <optional>

Box::Box(const V3f &center_, const V3f &size_) : center{center_}, size{size_} {
  buildFaces();
}

void Box::buildFaces() {
  const V3f h = size * 0.5f; // half extents

  // Define 8 corners
  const V3f c000 = center + V3f(-h.x(), -h.y(), -h.z());
  const V3f c001 = center + V3f(-h.x(), -h.y(), h.z());
  const V3f c010 = center + V3f(-h.x(), h.y(), -h.z());
  const V3f c011 = center + V3f(-h.x(), h.y(), h.z());
  const V3f c100 = center + V3f(h.x(), -h.y(), -h.z());
  const V3f c101 = center + V3f(h.x(), -h.y(), h.z());
  const V3f c110 = center + V3f(h.x(), h.y(), -h.z());
  const V3f c111 = center + V3f(h.x(), h.y(), h.z());

  //  +X face (right)
  faces[0] = RectPlane(c100, V3f(0, size.y(), 0), V3f(0, 0, size.z()));
  // -X face (left)
  faces[1] = RectPlane(c000, V3f(0, size.y(), 0), V3f(0, 0, size.z()));
  // +Y face (front)
  faces[2] = RectPlane(c010, V3f(size.x(), 0, 0), V3f(0, 0, size.z()));
  // -Y face (back)
  faces[3] = RectPlane(c000, V3f(size.x(), 0, 0), V3f(0, 0, size.z()));
  // +Z face (top)
  faces[4] = RectPlane(c001, V3f(size.x(), 0, 0), V3f(0, size.y(), 0));
  // -Z face (bottom)
  faces[5] = RectPlane(c000, V3f(size.x(), 0, 0), V3f(0, size.y(), 0));

  // Sanity check each face
  for (size_t i = 0; i < faces.size(); ++i) {
    const auto &f = faces[i];

    if (f.u.norm() < 1e-6f || f.v.norm() < 1e-6f) {
      std::cerr << "Box::buildFaces: face " << i
                << " has degenerate u or v vector!\n";
    }
    if (f.normal.norm() < 1e-6f) {
      std::cerr << "Box::buildFaces: face " << i << " has degenerate normal!\n";
    }
  }
}

std::optional<HitRecord> Box::intersect(const Ray &ray) const {
  std::optional<HitRecord> closestHit{std::nullopt};

  float closestT{std::numeric_limits<float>::max()};

  for (const auto &face : faces) {
    if (auto hit = face.intersect(ray)) {
      if (hit->t < closestT) {
        closestT = hit->t;
        closestHit = hit;
      }
    }
  }

  return closestHit;
}
