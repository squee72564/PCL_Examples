#include "helpers.hpp"
#include <fstream>
#include <algorithm>

#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

namespace {
static bool hasField(const pcl::PCLPointCloud2 &cloud, const std::string &field_name) {
  return std::any_of(
      cloud.fields.cbegin(), cloud.fields.cend(),
      [&](const pcl::PCLPointField &f) { return f.name == field_name; });
}
} // namespace

int loadPCDFileManual(const std::string &file_name,
                      pcl::PCLPointCloud2 &pointCloud, Eigen::Vector4f &origin,
                      Eigen::Quaternionf &orientation) {
  pcl::PCDReader reader{};
  int pcd_version{-1};
  int data_type{-1};
  unsigned data_index{0};

  std::ifstream fstream{file_name, std::ios::binary};

  if (!fstream.is_open()) {
    std::cerr << fmt::format("Could not open pcd file: {}\n", file_name);
    return -1;
  }

  int ret = reader.readHeader(fstream, pointCloud, origin, orientation,
                              pcd_version, data_type, data_index);

  if (ret < 0) {
    std::cerr << fmt::format("Could not read pcd file: {}\n", file_name);
    return -1;
  }

  if (data_type == 0) {

    fstream.seekg(data_index);
    return reader.readBodyASCII(fstream, pointCloud, pcd_version);

  } else {

    fstream.seekg(0, std::ios::end);
    std::streampos file_size = fstream.tellg();
    std::size_t body_size = static_cast<std::size_t>(file_size) - data_index;
    fstream.seekg(0, std::ios::beg);

    std::vector<unsigned char> buff(file_size);
    fstream.read(reinterpret_cast<char *>(buff.data()), file_size);

    if (!fstream) {
      std::cerr << fmt::format("Error: read only {} bytes; expected {}.\n",
                               fstream.gcount(),
                               static_cast<std::size_t>(file_size));
      return -1;
    }

    bool compressed{data_type == 2};

    return reader.readBodyBinary(buff.data(), pointCloud, pcd_version,
                                 compressed, data_index);
  }
}
PointCloudVariantPtr loadCloud(const pcl::PCLPointCloud2 &pointCloudBlob) {
  std::cout << "Loading PointCloud with type: ";
  if (hasField(pointCloudBlob, "rgba")) {
    std::cout << "POINTXYZRGBA\n";
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
    pcl::fromPCLPointCloud2(pointCloudBlob, *cloud);
    return cloud;
  } else if (hasField(pointCloudBlob, "rgb")) {
    std::cout << "POINTXYZRGB\n";
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::fromPCLPointCloud2(pointCloudBlob, *cloud);
    return cloud;
  } else if (hasField(pointCloudBlob, "intensity")) {
    std::cout << "POINTXYZI\n";
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::fromPCLPointCloud2(pointCloudBlob, *cloud);
    return cloud;
  } else {
    std::cout << "POINTXYZ\n";
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromPCLPointCloud2(pointCloudBlob, *cloud);
    return cloud;
  }
}

void shiftCloudToCentroid(PointCloudVariantPtr &cloud) {
  std::visit(
      [&](auto &&cloudPtr) {
        if (!cloudPtr || cloudPtr->points.empty()) return;

        Eigen::Vector4f centroid{};
        pcl::compute3DCentroid(*cloudPtr, centroid);

        Eigen::Affine3f transform{ Eigen::Affine3f::Identity() };
        transform.translation() = -centroid.head<3>();

        pcl::transformPointCloud(*cloudPtr, *cloudPtr, transform);
      },
      cloud);
}

PointCloudVariantPtr
downsampleWithVoxelGrid(const PointCloudVariantPtr &inCloud,
                        const Eigen::Vector3f &leaf_sizes) {
  return std::visit(
      [&](auto &&cloudPtr) {
        using CloudT = std::decay_t<decltype(*cloudPtr)>;
        using PointT = typename CloudT::PointType;

        std::cout << "Downsampling with voxel grid\n";

        // Downsample with voxel grid
        pcl::VoxelGrid<PointT> voxelGrid{};
        pcl::PointCloud<PointT> out{};

        voxelGrid.setInputCloud(cloudPtr);

        voxelGrid.setLeafSize(leaf_sizes.x(), leaf_sizes.y(), leaf_sizes.z());
        voxelGrid.filter(out);

        std::cout << fmt::format(
            "-- Voxel Grid filtered PointCloud data size: {}\n", out.size());

        return PointCloudVariantPtr{
            pcl::make_shared<pcl::PointCloud<PointT>>(out)};
      },
      inCloud);
}
