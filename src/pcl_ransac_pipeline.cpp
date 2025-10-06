#include <cstdlib>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/impl/point_types.hpp>
#include <thread>
#include <type_traits>
#include <utility>
#include <variant>

#include <fmt/format.h>

#include "helper_types.hpp"
#include "helpers.hpp"

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/memory.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/pcl_visualizer.h>

[[nodiscard]] std::pair<PointCloudVariantPtr, PointCloudVariantPtr>
extract_inlier_and_outlier_ransac(const PointCloudVariantPtr &inCloud);

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cerr << fmt::format("Usage: {} <pcd file>\n", argv[0]);
    return EXIT_FAILURE;
  }

  pcl::PCLPointCloud2 pointCloudBlob{};
  Eigen::Vector4f origin{};
  Eigen::Quaternionf orientation{};

  std::cout << "Loading point cloud from PCD file.\n";
  if (loadPCDFileManual(argv[1], pointCloudBlob, origin, orientation) != 0) {
    std::cerr << "Error: Failed to load PCD file. Exiting program.\n";
    return EXIT_FAILURE;
  }

  std::cout << fmt::format("-- Original PCLPointCloud2 data size: {}\n",
                           pointCloudBlob.data.size());

  for (const auto &field : pointCloudBlob.fields) {
    std::cout << fmt::format("-- Field: {} (offset {}, type {}, count {})\n",
                             field.name, field.offset, field.datatype,
                             field.count);
  }

  pcl::visualization::PCLVisualizer::Ptr cloudViewer =
      pcl::make_shared<pcl::visualization::PCLVisualizer>(
          "PCD File - Point Cloud Visualization");

  // variant to hold all PointT types for actual point cloud
  PointCloudVariantPtr pointCloudPtr{};
  {
    std::cout
        << "Converting PCLPointCloud2 to actual type PointCloud<PointT>\n";
    PointCloudVariantPtr pointCloudPtrTemp = loadCloud(pointCloudBlob);

    shiftCloudToCentroid(pointCloudPtrTemp);
    pointCloudPtr =
        std::move(downsampleWithVoxelGrid(pointCloudPtrTemp, {0.001, 0.001, 0.001}));
  }

  auto [terrainCloud, obstacleCloud] =
      extract_inlier_and_outlier_ransac(pointCloudPtr);

  addToPointCloudVisualizer(terrainCloud, cloudViewer, "terrain",
                            {0.0f, 1.0f, 0.0f});
  addToPointCloudVisualizer(obstacleCloud, cloudViewer, "obstacles",
                            {1.0f, 0.0f, 0.0f});

  std::cout << "Resetting camera\n";
  cloudViewer->resetCamera();

  std::cout << "Entering visualization loop\n";
  // Start visualization loop
  while (!cloudViewer->wasStopped()) {
    cloudViewer->spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return EXIT_SUCCESS;
}

std::pair<PointCloudVariantPtr, PointCloudVariantPtr>
extract_inlier_and_outlier_ransac(const PointCloudVariantPtr &inCloud) {
  return std::visit(
      [&](auto &&cloudPtr) {
        using CloudT = std::decay_t<decltype(*cloudPtr)>;
        using PointT = typename CloudT::PointType;

        std::cout << "Running PMF to extract inlier / outlier point clouds\n"
                  << fmt::format("-- Total points in cloud: {}\n",
                                 cloudPtr->size());

        auto model =
            pcl::make_shared<pcl::SampleConsensusModelPlane<PointT>>(cloudPtr, true);
        auto ransac =
            pcl::make_shared<pcl::RandomSampleConsensus<PointT>>(model);

        ransac->setDistanceThreshold(0.2);

        ransac->computeModel(1);
        pcl::Indices inliers = ransac->getInliers();

        pcl::ExtractIndices<PointT> extract;

        extract.setInputCloud(cloudPtr);
        extract.setIndices(pcl::make_shared<pcl::Indices>(inliers));

        auto inlierCloud{pcl::make_shared<pcl::PointCloud<PointT>>()};
        extract.setNegative(false);
        extract.filter(*inlierCloud);

        auto outlierCloud{pcl::make_shared<pcl::PointCloud<PointT>>()};
        extract.setNegative(true);
        extract.filter(*outlierCloud);

        std::cout << fmt::format("-- Total points in inlier cloud: {}\n",
                                 inlierCloud->size())
                  << fmt::format("-- Total points in outlier cloud: {}\n",
                                 outlierCloud->size());

        return std::make_pair(PointCloudVariantPtr{inlierCloud},
                              PointCloudVariantPtr{outlierCloud});
      },
      inCloud);
}
