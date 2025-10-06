#ifndef GEOMETRIC_OBJECT_HPP
#define GEOMETRIC_OBJECT_HPP

#include <Eigen/Dense>
#include <optional>

#include "Ray.hpp"

struct HitRecord {
    Eigen::Vector3f point;
    Eigen::Vector3f normal;
    float t;
};

class GeometricObject {
public:
    virtual ~GeometricObject() = default;

    virtual std::optional<HitRecord> intersect(const Ray& ray) const = 0;
};

#endif // GEOMETRIC_OBJECT_HPP
