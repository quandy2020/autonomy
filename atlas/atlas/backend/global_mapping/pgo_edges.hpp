/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_VXSLAM_PGO_EDGES_HPP_
#define ATLAS_VXSLAM_PGO_EDGES_HPP_

#include <Eigen/Core>
#include <string>
#include <vector>

#include "frontend/lidar/imu/imu_state.hpp"
#include "optimization/voxel/pose_graph.hpp"

namespace atlas {
namespace vxslam {

// Resolve a session-local index without crossing into the next session.
// Offsets contain one entry per session and a final total node count.
// Leaves node_index unchanged if the session or local index is invalid.
bool ResolvePoseGraphNodeIndex(int session_index, int local_node_index,
                               const std::vector<int>& session_offsets,
                               int* node_index);

/** @brief Multi-session loop edge batch (Voxel-SLAM PGO_Edge). */
struct PgoEdge {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int m1 = 0;
  int m2 = 0;
  std::vector<int> ids1;
  std::vector<int> ids2;
  std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> rots;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> tras;
  std::vector<Eigen::Matrix<double, 6, 1>,
              Eigen::aligned_allocator<Eigen::Matrix<double, 6, 1>>>
      covs;

  void Push(int id1, int id2, const Eigen::Matrix3d& rot,
            const Eigen::Vector3d& tra, const Eigen::Matrix<double, 6, 1>& v6);

  /** @brief True if both map ids appear in @p maps; writes step indices. */
  bool IsAdapt(const std::vector<int>& maps, std::vector<int>* step) const;
};

struct PgoEdges {
  std::vector<PgoEdge> edges;
  std::vector<std::vector<int>> mates;

  void Push(int m1, int m2, int id1, int id2, const Eigen::Matrix3d& rot,
            const Eigen::Vector3d& tra, const Eigen::Matrix<double, 6, 1>& v6);

  /** @brief Connected component of session maps from @p root (Voxel-SLAM). */
  void Connect(int root, std::vector<int>* ids) const;
};

/** @brief Read/write Voxel-SLAM edge.txt (io: 0=read, 1=write). */
bool PgoEdgesIo(PgoEdges* edges, const std::vector<std::string>& fnames, int io,
                const std::string& save_path, const std::string& bagname);

/**
 * @brief Build PoseGraph from consecutive poses + optional loop factors
 *        (Voxel-SLAM build_graph spirit, Argus PoseGraph backend).
 */
void BuildPoseGraph(backend::PoseGraph* graph,
                    const std::vector<ImuState>& poses,
                    const std::vector<backend::LoopFactor>& loops,
                    double odom_info = 50.0);

}  // namespace vxslam
}  // namespace atlas

#endif
