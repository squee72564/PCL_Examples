#ifndef HELPERS_HPP
#define HELPERS_HPP

#include <fmt/format.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "helper_types.hpp"

template <class T> inline constexpr bool always_false_v = false;

// Loads a PCD file into PCLPointCloud2
int loadPCDFileManual(const std::string &file_name,
                      pcl::PCLPointCloud2 &pointCloud, Eigen::Vector4f &origin,
                      Eigen::Quaternionf &orientation);

// Gets variant holding PointCloud<PointT> from PCLPointCloud2
PointCloudVariantPtr loadCloud(const pcl::PCLPointCloud2 &pointCloudBlob);

// Translates point cloud with centroid around origin
void shiftCloudToCentroid(PointCloudVariantPtr &cloud);

// Add a point cloud to the visualizer
void addToPointCloudVisualizer(
    const PointCloudVariantPtr &pointCloud,
    const pcl::visualization::PCLVisualizer::Ptr &cloudViewer,
    const std::string &cloudName,
    const Eigen::Vector3f &cloudcolor = {1.0f, 1.0f, 1.0f});

// Downsample a pointcloud using a voxel grid
PointCloudVariantPtr
downsampleWithVoxelGrid(const PointCloudVariantPtr &cloud,
                        const Eigen::Vector3f &leaf_sizes = {0.5f, 0.5f, 0.5f});

#endif // HELPERS_HPP
