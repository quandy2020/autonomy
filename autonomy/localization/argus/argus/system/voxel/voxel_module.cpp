/*
 * Copyright 2026 The Openbot Authors
 */
#include "argus/system/voxel/voxel_module.hpp"

namespace argus {
namespace system {
namespace voxel {

VoxelModule::VoxelModule() : VoxelModule("voxel") {}

VoxelModule::VoxelModule(std::string name)
    : name_(std::move(name)),
      odometry_(std::make_unique<vxslam::Odometry>()) {
    health_.level = common::HealthLevel::kUnknown;
    health_.message = "unconfigured";
}

const std::string& VoxelModule::name() const {
    return name_;
}

bool VoxelModule::Configure(const common::Component::Config& config) {
    if (!config.name.empty()) {
        name_ = config.name;
    }
    health_.level = common::HealthLevel::kOk;
    health_.message = "configured";
    return true;
}

bool VoxelModule::Start() {
    if (odometry_ == nullptr) {
        odometry_ = std::make_unique<vxslam::Odometry>();
    }
    running_ = true;
    health_.level = common::HealthLevel::kOk;
    health_.message = "running";
    return true;
}

void VoxelModule::Stop() {
    running_ = false;
    health_.level = common::HealthLevel::kOk;
    health_.message = "stopped";
}

common::Health VoxelModule::health() const {
    return health_;
}

}  // namespace voxel
}  // namespace system
}  // namespace argus
