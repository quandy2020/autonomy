/*
 * Copyright 2026 The Openbot Authors
 */
#include "argus/system/voxel/global_mapping/global_mapping.hpp"

namespace argus {
namespace system {
namespace voxel {

GlobalMapping::GlobalMapping()
    : pose_graph_(std::make_unique<backend::PoseGraph>()) {}

GlobalMapping::~GlobalMapping() = default;

}  // namespace voxel
}  // namespace system
}  // namespace argus
