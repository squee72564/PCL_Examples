#ifndef HELPERS_HPP
#define HELPERS_HPP

#include <fmt/format.h>

#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>

#include "helper_types.hpp"

template<class T>
inline constexpr bool always_false_v = false;

// Function to load a PCD file into PCLPointCloud2
int loadPCDFileManual(const std::string &file_name,
                      pcl::PCLPointCloud2 &pointCloud, Eigen::Vector4f &origin,
                      Eigen::Quaternionf &orientation);

PointCloudVariantPtr loadCloud(const pcl::PCLPointCloud2 &pointCloudBlob);

#endif // HELPERS_HPP
