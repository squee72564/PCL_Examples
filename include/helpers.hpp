#ifndef HELPERS_HPP
#define HELPERS_HPP

#include <fmt/format.h>

#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "helper_types.hpp"

template<class T>
inline constexpr bool always_false_v = false;

// Function to load a PCD file into PCLPointCloud2
int loadPCDFileManual(const std::string &file_name,
                      pcl::PCLPointCloud2 &pointCloud, Eigen::Vector4f &origin,
                      Eigen::Quaternionf &orientation);

PointCloudVariantPtr loadCloud(const pcl::PCLPointCloud2 &pointCloudBlob);

void shiftCloudToCentroid(PointCloudVariantPtr &cloud);

template <typename CloudT>
void addToPointCloudVisualizer(
    typename CloudT::Ptr &pointCloud,
    pcl::visualization::PCLVisualizer::Ptr cloudViewer,
    const std::string &cloudName) {

  if constexpr (std::is_same_v<CloudT, pcl::PointCloud<pcl::PointXYZ>>) {
    cloudViewer->addPointCloud(pointCloud, cloudName);

  } else if constexpr (std::is_same_v<CloudT,
                                      pcl::PointCloud<pcl::PointXYZI>>) {
    auto handler =
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>(
            pointCloud, "intensity");
    cloudViewer->addPointCloud(pointCloud, handler, cloudName);

  } else if constexpr (std::is_same_v<CloudT,
                                      pcl::PointCloud<pcl::PointXYZRGB>> ||
                       std::is_same_v<CloudT,
                                      pcl::PointCloud<pcl::PointXYZRGBA>>) {
    auto handler = pcl::visualization::PointCloudColorHandlerRGBField<
        typename CloudT::PointType>(pointCloud);
    cloudViewer->addPointCloud(pointCloud, handler, cloudName);
  } else {
    static_assert(always_false_v<CloudT>(), "Unsupported point type");
  }
}

#endif // HELPERS_HPP
