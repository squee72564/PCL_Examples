#ifndef RAY_HPP
#define RAY_HPP
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <limits>

struct Ray {
  Eigen::Vector3f origin;
  Eigen::Vector3f direction;
  float tMin{0.001f};
  float tMax{std::numeric_limits<float>::max()};

  Ray() = default;

  Ray(const Eigen::Vector3f &o, const Eigen::Vector3f &d, float tmin = 0.001f,
      float tmax = std::numeric_limits<float>::max());

  Eigen::Vector3f at(float t) const;
};

#endif // RAY_HPP
