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

// Loads a PCD file into PCLPointCloud2
int loadPCDFileManual(const std::string &file_name,
                      pcl::PCLPointCloud2 &pointCloud, Eigen::Vector4f &origin,
                      Eigen::Quaternionf &orientation);

// Gets variant holding PointCloud<PointT> from PCLPointCloud2
PointCloudVariantPtr loadCloud(const pcl::PCLPointCloud2 &pointCloudBlob);

// Translates point cloud with centroid around origin
void shiftCloudToCentroid(PointCloudVariantPtr &cloud);

// Add a point cloud to the visualizer
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

// Downsample a pointcloud using a voxel grid
PointCloudVariantPtr
downsampleWithVoxelGrid(const PointCloudVariantPtr &cloud,
                        const Eigen::Vector3f &leaf_sizes = {0.5f, 0.5f, 0.5f});

#endif // HELPERS_HPP
