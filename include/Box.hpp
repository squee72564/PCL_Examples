#ifndef BOX_HPP
#define BOX_HPP

#include "GeometricObject.hpp"
#include "RectPlane.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <array>
#include <optional>

using V3f = Eigen::Vector3f;

class Box : public GeometricObject {
  V3f center;
  V3f size;
  std::array<RectPlane, 6> faces;

  void buildFaces();

public:
  Box() = default;
  explicit Box(const V3f &center_, const V3f &size_);

  std::optional<HitRecord> intersect(const Ray &ray) const override;
};

#endif // BOX_HPP
