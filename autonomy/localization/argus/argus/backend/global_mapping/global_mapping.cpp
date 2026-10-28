/*
 * Copyright 2026 The Openbot Authors
 */
#include "backend/global_mapping/global_mapping.hpp"

namespace argus {
namespace vxslam {

GlobalMapping::GlobalMapping()
    : pose_graph_(std::make_unique<backend::PoseGraph>()) {}

GlobalMapping::~GlobalMapping() = default;

}  // namespace vxslam
}  // namespace argus
