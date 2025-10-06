#include "Plane.hpp"

Plane::Plane(const Eigen::Vector3f &p, const Eigen::Vector3f &n)
    : point{p}, normal{n.normalized()} {}

std::optional<HitRecord> Plane::intersect(const Ray &ray) const {
  float denom{normal.dot(ray.direction)};

  if (std::abs(denom) < 1e-6f) {
    return std::nullopt;
  }

  float t{normal.dot(point - ray.origin) / denom};

  if (t < ray.tMin || t > ray.tMax)
    return std::nullopt;

  return HitRecord{ray.at(t), denom < 0 ? normal : -normal, t};
}
