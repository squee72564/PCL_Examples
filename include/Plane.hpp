#ifndef PLANE_HPP
#define PLANE_HPP

#include "GeometricObject.hpp"
#include <Eigen/Dense>
#include <optional>

class Plane : public GeometricObject {
  Eigen::Vector3f point;
  Eigen::Vector3f normal;

public:
  Plane() = default;
  explicit Plane(const Eigen::Vector3f &p, const Eigen::Vector3f &n);

  std::optional<HitRecord> intersect(const Ray &ray) const override;
};

#endif // PLANE_HPP
