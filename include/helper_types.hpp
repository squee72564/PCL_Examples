#ifndef HELPER_TYPES_HPP
#define HELPER_TYPES_HPP

#include <variant>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/sac.h>
#include <pcl/sample_consensus/ransac.h>

using PointCloudVariantPtr = std::variant<
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
        pcl::PointCloud<pcl::PointXYZI>::Ptr,
        pcl::PointCloud<pcl::PointXYZ>::Ptr
    >;

#endif // HELPER_TYPES_HPP
