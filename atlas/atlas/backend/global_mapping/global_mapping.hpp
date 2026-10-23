/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_SYSTEM_VOXEL_GLOBAL_MAPPING_GLOBAL_MAPPING_HPP_
#define ATLAS_SYSTEM_VOXEL_GLOBAL_MAPPING_GLOBAL_MAPPING_HPP_

#include <memory>

#include "backend/global_mapping/gtsam_pgo.hpp"
#include "backend/global_mapping/pgo_edges.hpp"
#include "backend/global_mapping/previous_map.hpp"
#include "optimization/voxel/pose_graph.hpp"

namespace atlas {
namespace vxslam {

/**
 * @brief Global mapping facade: PoseGraph + GTSAM PGO helpers.
 */
class GlobalMapping {
 public:
  GlobalMapping();
  ~GlobalMapping();

  backend::PoseGraph* pose_graph() { return pose_graph_.get(); }
  const backend::PoseGraph* pose_graph() const { return pose_graph_.get(); }

 private:
  std::unique_ptr<backend::PoseGraph> pose_graph_;
};

}  // namespace vxslam
}  // namespace atlas

#endif
