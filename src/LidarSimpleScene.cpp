#include "Box.hpp"
#include "Lidar.hpp"
#include "Plane.hpp"
#include <Eigen/Dense>
#include <fmt/format.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/memory.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>

int main() {
  using PointT = pcl::PointXYZ;
  using V3f = Eigen::Vector3f;

  auto cloud{pcl::make_shared<pcl::PointCloud<PointT>>()};

  std::vector<pcl::shared_ptr<GeometricObject>> scene;

  auto ground = pcl::make_shared<Plane>(V3f{0, 0, 0}, V3f{0, 0, -1});
  auto box  = pcl::make_shared<Box>(V3f{5, 5, 1}, V3f{2, 2, 2});
  auto box2 = pcl::make_shared<Box>(V3f{-5, 5, 1}, V3f{2, 2, 2});
  auto box3 = pcl::make_shared<Box>(V3f{5, -5, 1}, V3f{2, 2, 2});
  auto box4 = pcl::make_shared<Box>(V3f{-5, -5, 1}, V3f{2, 2, 2});

  scene.push_back(ground);
  scene.push_back(box);
  scene.push_back(box2);
  scene.push_back(box3);
  scene.push_back(box4);

  Lidar<PointT>::Params params{128, 2048, 180.0f, 360.0f, 20.0f};

  Lidar<PointT> lidar{{0, 0, 15}, params};
  lidar.setPointCloud(cloud);

  lidar.scan(scene);

  std::cout << fmt::format("Scan complete, points collected: {}\n",
                           cloud->size());

  pcl::visualization::PCLVisualizer cloudViewer{"Simple Lidar Test"};

  cloudViewer.addPointCloud(cloud, "Scan");

  cloudViewer.resetCamera();

  while (!cloudViewer.wasStopped()) {
    cloudViewer.spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
