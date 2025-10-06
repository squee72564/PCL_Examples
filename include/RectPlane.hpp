#ifndef RECT_PLANE_HPP
#define RECT_PLANE_HPP

#include "GeometricObject.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <optional>

using V3f = Eigen::Vector3f;

class RectPlane : public GeometricObject {
public:
    V3f origin;
    V3f u;
    V3f v;
    V3f normal;

    RectPlane() = default;
    RectPlane(const V3f& origin_, const V3f& u_, const V3f& v);

    std::optional<HitRecord> intersect(const Ray& ray) const override;
};

#endif // RECT_PLANE_HPP
