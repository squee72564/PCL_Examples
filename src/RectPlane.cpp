#include "RectPlane.hpp"
#include <iostream>
#include <stdexcept>

RectPlane::RectPlane(const V3f &origin_, const V3f &u_, const V3f &v_)
    : origin{origin_}, u{u_}, v{v_}, normal{u_.cross(v_).normalized()} {
  if (u_.norm() < 1e-6f || v_.norm() < 1e-6f || normal.norm() < 1e-6f) {
    throw std::runtime_error("Degenerate RectPlane");
  }
}

std::optional<HitRecord> RectPlane::intersect(const Ray &ray) const {
  float denom{normal.dot(ray.direction)};

  if (std::abs(denom) < 1e-6f)
    return std::nullopt;

  float t{normal.dot(origin - ray.origin) / denom};

  if (t < ray.tMin || t > ray.tMax)
    return std::nullopt;

  V3f p{ray.at(t)};
  V3f rel{p - origin};

  float u_sq_norm = u.squaredNorm();
  float v_sq_norm = v.squaredNorm();

  if (u_sq_norm < 1e-6f || v_sq_norm < 1e-6f) {
    std::cerr << "Failed degen check RectPlane.cpp\n";
    return std::nullopt;
  }

  float a = rel.dot(u) / u_sq_norm;
  float b = rel.dot(v) / v_sq_norm;

  if (a < 0.0f || a > 1.0f || b < 0.0f || b > 1.0f) {
    return std::nullopt;
  }

  return HitRecord{p, denom < 0 ? normal : -normal, t};
}
