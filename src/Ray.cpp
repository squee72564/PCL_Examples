#include "Ray.hpp"

Ray::Ray(const Eigen::Vector3f &o, const Eigen::Vector3f &d, float tmin,
         float tmax)
    : origin{o}, direction{d.normalized()}, tMin{tmin}, tMax{tmax} {}

Eigen::Vector3f Ray::at(float t) const {
    return origin + t * direction;
}
