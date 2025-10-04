#ifndef HELPER_TYPES_HPP
#define HELPER_TYPES_HPP

#include <variant>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

/*
*   This variant is used to represent multiple possible
*   specializations for pcl::PointCloud<PointT>
*
*   Using a variant like this we can use 
*   pcl::PCLPointCloud2 to agnostically read from a
*   .pcd file and then just metadata from the file to 
*   convert to one of the types within this variant.
*
*   using std::visit we can get access to the actual type 
*   held within the variant and apply different algorithms 
*   from PCL on this type
*/
using PointCloudVariantPtr = std::variant<
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
        pcl::PointCloud<pcl::PointXYZI>::Ptr,
        pcl::PointCloud<pcl::PointXYZ>::Ptr
    >;

#endif // HELPER_TYPES_HPP
