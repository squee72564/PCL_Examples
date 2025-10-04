#include <cstdlib>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/impl/point_types.hpp>
#include <thread>
#include <type_traits>
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
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/visualization/pcl_visualizer.h>

void runPMFAndVisualize(const PointCloudVariantPtr &pointCloudPtr,
                        const pcl::visualization::PCLVisualizer::Ptr &cloudViewer);

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
  // Right now I load into a temporary cloud and then
  PointCloudVariantPtr pointCloudPtr{};

  {
    std::cout
        << "Converting PCLPointCloud2 to actual type PointCloud<PointT>\n";
    PointCloudVariantPtr pointCloudPtrTemp = loadCloud(pointCloudBlob);

    shiftCloudToCentroid(pointCloudPtrTemp);
    pointCloudPtr = std::move(downsampleWithVoxelGrid(pointCloudPtrTemp));
  }

  runPMFAndVisualize(pointCloudPtr, cloudViewer);

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

void runPMFAndVisualize(const PointCloudVariantPtr &pointCloudPtr,
                        const pcl::visualization::PCLVisualizer::Ptr &cloudViewer) {
  std::visit(
      [&](auto &&cloudPtr) {
        using CloudT = std::decay_t<decltype(*cloudPtr)>;
        using PointT = typename CloudT::PointType;

        std::cout << "Running PMF and adding inlier / outlier point clouds to "
                     "visualizer\n"
                  << fmt::format("--Total points in cloud: {}\n",
                                 cloudPtr->size());

        auto pmf =
            pcl::make_shared<pcl::ProgressiveMorphologicalFilter<PointT>>();
        pmf->setInputCloud(cloudPtr);

        pcl::Indices inliers;
        pmf->extract(inliers);

        pcl::ExtractIndices<PointT> extract;

        extract.setInputCloud(cloudPtr);
        extract.setIndices(pcl::make_shared<pcl::Indices>(inliers));

        auto inlierCloud{pcl::make_shared<pcl::PointCloud<PointT>>()};
        extract.setNegative(false);
        extract.filter(*inlierCloud);

        auto outlierCloud{pcl::make_shared<pcl::PointCloud<PointT>>()};
        extract.setNegative(true);
        extract.filter(*outlierCloud);

        std::cout << "Got inliers and outliers\n";

        // Add point clouds to visualizer
        addToPointCloudVisualizer<CloudT>(inlierCloud, cloudViewer, "inliers");

        cloudViewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "inliers");

        addToPointCloudVisualizer<CloudT>(outlierCloud, cloudViewer,
                                          "outliers");

        cloudViewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "outliers");

        std::cout << "Added Point Clouds\n";
      },
      pointCloudPtr);
}
