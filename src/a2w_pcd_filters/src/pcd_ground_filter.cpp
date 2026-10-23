// Offline adapter for the existing parallel PMF implementation.
#include "pcd2pgm/parallel_filters.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <Eigen/Geometry>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <cmath>

using pcd2pgm::Cloud;
namespace fs = std::filesystem;

void save(const fs::path& path, const Cloud& cloud) {
  if (cloud.empty()) {
    std::ofstream f(path, std::ios::binary);
    f << "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
         "WIDTH 0\nHEIGHT 1\nPOINTS 0\nDATA binary\n";
    if (!f) throw std::runtime_error("cannot save empty cloud");
  } else if (pcl::io::savePCDFileBinary(path.string(), cloud) != 0) {
    throw std::runtime_error("cannot save filtered cloud");
  }
}

int main(int argc, char** argv) {
  try {
    if (argc != 22) throw std::invalid_argument("Use scripts/generate_2d_map.sh --method pmf");
    auto number = [&](int i) {
      const double value = std::stod(argv[i]);
      if (!std::isfinite(value)) throw std::invalid_argument("non-finite parameter");
      return value;
    };
    Eigen::Matrix3d rotation;
    for (int i = 0; i < 9; ++i) rotation(i / 3, i % 3) = number(3 + i);
    if (!rotation.transpose().isApprox(rotation.inverse(), 1e-6) || std::abs(rotation.determinant()-1.) > 1e-6)
      throw std::invalid_argument("projection rotation is not a proper rigid rotation");
    const double leaf = number(12);
    const int threads = static_cast<int>(number(13));
    pcd2pgm::GroundOptions options{static_cast<int>(number(14)), static_cast<float>(number(15)),
      static_cast<float>(number(16)), static_cast<float>(number(17))};
    const int mean_k = static_cast<int>(number(18));
    const double stddev = number(19), h_min = number(20), h_max = number(21);
    if (leaf < 0 || threads < 1 || options.max_window_size < 1 || options.max_window_size > 1024
        || options.slope < 0 || options.initial_distance < 0 || options.max_distance < 0
        || mean_k < 0 || stddev < 0 || h_min > h_max)
      throw std::invalid_argument("invalid filter parameter");
    auto raw = pcl::make_shared<Cloud>();
    if (pcl::io::loadPCDFile(argv[1], *raw) != 0) throw std::runtime_error("cannot read source PCD");
    auto cloud = pcl::make_shared<Cloud>();
    cloud->reserve(raw->size());
    for (const auto& p : *raw) {
      if (!(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))) continue;
      const Eigen::Vector3d q = rotation * Eigen::Vector3d(p.x, p.y, p.z);
      cloud->push_back(pcl::PointXYZ(q.x(), q.y(), q.z()));
    }
    if (cloud->empty()) throw std::runtime_error("source PCD has no finite points");
    if (leaf > 0) {
      pcl::VoxelGrid<pcl::PointXYZ> voxel;
      voxel.setInputCloud(cloud); voxel.setLeafSize(leaf, leaf, leaf);
      auto down = pcl::make_shared<Cloud>(); voxel.filter(*down); cloud = down;
    }
    const auto indices = pcd2pgm::extractGroundParallel(cloud, options, threads);
    auto ground = pcl::make_shared<Cloud>(), other = pcl::make_shared<Cloud>();
    std::vector<bool> selected(cloud->size(), false);
    for (int i : indices) selected.at(i) = true;
    for (std::size_t i = 0; i < cloud->size(); ++i) (selected[i] ? ground : other)->push_back((*cloud)[i]);
    if (ground->empty()) throw std::runtime_error("PMF found no ground");
    if (mean_k > 0 && other->size() > 2) {
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setInputCloud(other); sor.setMeanK(std::min<int>(mean_k, other->size()-1));
      sor.setStddevMulThresh(stddev);
      auto filtered = pcl::make_shared<Cloud>(); sor.filter(*filtered); other = filtered;
    }
    auto obstacles = pcd2pgm::extractObstaclesParallel(other, ground, h_min, h_max, threads);
    fs::create_directories(argv[2]);
    save(fs::path(argv[2]) / "ground.pcd", *ground);
    save(fs::path(argv[2]) / "obstacles.pcd", *obstacles);
    std::cout << "地面分割完成：ground=" << ground->size() << " obstacles=" << obstacles->size() << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "二维地图滤波失败：" << e.what() << '\n';
    return 1;
  }
}
