#ifndef LIDAR_HPP
#define LIDAR_HPP

#include <Eigen/Dense>
#include <pcl/memory.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <type_traits>

#include "GeometricObject.hpp"

using V3f = Eigen::Vector3f;

template <typename T, typename = void> struct has_xyz : std::false_type {};

template <typename T>
struct has_xyz<
    T, std::void_t<decltype(std::declval<T>().x), decltype(std::declval<T>().y),
                   decltype(std::declval<T>().z)>> : std::true_type {};

template <typename PointT> class Lidar {
  static_assert(std::is_default_constructible_v<PointT>,
                "PointT must be default constructible");
  static_assert(has_xyz<PointT>::value, "PointT must have members x,y,z");

public:
  struct Params {
    int num_rings{16};
    int points_per_ring{1024};
    float vertical_fov{30.0f};
    float horizontal_fov{360.0f};
    float max_range{100.0f};
  };

  Lidar(const V3f &position_, const Params &params_)
      : position{position_}, params{params_} {}

  void setPointCloud(const typename pcl::PointCloud<PointT>::Ptr &cloud_) {
    cloud = cloud_;
  }

  void scan(const std::vector<pcl::shared_ptr<GeometricObject>> &scene) {
    if (!cloud) {
      std::cerr << "Lidar is trying to scan without having a point cloud to "
                   "add to!\n";
      return;
    }

    cloud->clear();

    cloud->reserve(params.num_rings * params.points_per_ring);

    // Convert FOVs to radians once
    const float vert_fov_rad = params.vertical_fov * float(M_PI / 180.0f);
    const float horiz_fov_rad = params.horizontal_fov * float(M_PI / 180.0f);

    // We'll scan from top (v_max) to bottom (v_min)
    float vert_max = vert_fov_rad / 2.0f;  // top of FOV
    float vert_min = -vert_fov_rad / 2.0f; // bottom of FOV
    float vert_step = (vert_min - vert_max) / float(params.num_rings - 1);

    float horiz_step = horiz_fov_rad / float(params.points_per_ring);

    for (int i = 0; i < params.num_rings; ++i) {
      float v{vert_max + i * vert_step};

      for (int j = 0; j < params.points_per_ring; ++j) {
        float h{j * horiz_step};

        Ray ray{makeRay(h, v)};

        float closestT{params.max_range};
        V3f hitPoint{};
        bool hit{false};

        for (const auto &obj : scene) {
          auto rec = obj->intersect(ray);
          if (rec && rec->t < closestT) {
            closestT = rec->t;
            hitPoint = rec->point;
            hit = true;
          }
        }

        if (hit) {
          PointT pt;
          pt.x = hitPoint.x();
          pt.y = hitPoint.y();
          pt.z = hitPoint.z();
          cloud->points.push_back(pt);
        }
      }
    }
  }

private:
  V3f position;
  Params params;
  typename pcl::PointCloud<PointT>::Ptr cloud;

  Ray makeRay(float h, float v) const {
    // h = azimuth, v = elevation (positive = up, negative = down)
    float cos_v = std::cos(v);
    V3f dir{
        cos_v * std::cos(h), // X
        cos_v * std::sin(h), // Y
        std::sin(v)          // Z: positive = up, negative = down
    };

    return Ray{position, dir.normalized()};
  }
};

#endif // LIDAR_HPP
