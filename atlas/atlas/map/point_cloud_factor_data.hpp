/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_MAP_POINT_CLOUD_FACTOR_DATA_HPP_
#define ATLAS_MAP_POINT_CLOUD_FACTOR_DATA_HPP_

#include <Eigen/Core>
#include <cstddef>
#include <vector>

#include "map/point_cluster.hpp"

namespace atlas {
namespace map {

template <int Dimension>
using AlignedVectorList =
    std::vector<Eigen::Matrix<double, Dimension, 1>,
                Eigen::aligned_allocator<Eigen::Matrix<double, Dimension, 1>>>;

template <int Dimension>
using AlignedMatrixList = std::vector<
    Eigen::Matrix<double, Dimension, Dimension>,
    Eigen::aligned_allocator<Eigen::Matrix<double, Dimension, Dimension>>>;

/** @brief Map-owned plane observations consumed by local and global solvers. */
struct PointCloudFactorData {
 public:
  using Eigenvalues =
      std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;
  using Eigenvectors =
      std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>;

  explicit PointCloudFactorData(int window_count = 0)
      : window_count(window_count) {}

  void AddPlaneFactor(const std::vector<PointCluster>& window_point_clusters,
                      const PointCluster& fixed_point_cluster,
                      double coefficient,
                      const Eigen::Vector3d& eigenvalue_values,
                      const Eigen::Matrix3d& eigenvector_columns,
                      const PointCluster& accumulated_point_cluster) {
    point_clusters_by_window.push_back(window_point_clusters);
    fixed_point_clusters.push_back(fixed_point_cluster);
    coefficients.push_back(coefficient);
    eigenvalues.push_back(eigenvalue_values);
    eigenvectors.push_back(eigenvector_columns);
    accumulated_point_clusters.push_back(accumulated_point_cluster);
  }

  void Clear() {
    fixed_point_clusters.clear();
    point_clusters_by_window.clear();
    coefficients.clear();
    eigenvalues.clear();
    eigenvectors.clear();
    accumulated_point_clusters.clear();
  }

  std::size_t factor_count() const { return point_clusters_by_window.size(); }

  int window_count = 0;
  std::vector<PointCluster> fixed_point_clusters;
  std::vector<std::vector<PointCluster>> point_clusters_by_window;
  std::vector<double> coefficients;
  Eigenvalues eigenvalues;
  Eigenvectors eigenvectors;
  std::vector<PointCluster> accumulated_point_clusters;
};

}  // namespace map
}  // namespace atlas

#endif  // ATLAS_MAP_POINT_CLOUD_FACTOR_DATA_HPP_
