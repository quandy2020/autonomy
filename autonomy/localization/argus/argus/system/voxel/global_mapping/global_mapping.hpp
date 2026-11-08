/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ARGUS_SYSTEM_VOXEL_GLOBAL_MAPPING_GLOBAL_MAPPING_HPP_
#define ARGUS_SYSTEM_VOXEL_GLOBAL_MAPPING_GLOBAL_MAPPING_HPP_

#include <memory>

#include "argus/system/voxel/global_mapping/gtsam_pgo.hpp"
#include "argus/system/voxel/global_mapping/pgo_edges.hpp"
#include "argus/system/voxel/global_mapping/previous_map.hpp"
#include "argus/optimization/pose_graph.hpp"

namespace argus {
namespace system {
namespace voxel {

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

}  // namespace voxel
}  // namespace system
}  // namespace argus

#endif
