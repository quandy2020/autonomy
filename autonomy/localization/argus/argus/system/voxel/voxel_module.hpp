/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ARGUS_SYSTEM_VOXEL_VOXEL_MODULE_HPP_
#define ARGUS_SYSTEM_VOXEL_VOXEL_MODULE_HPP_

#include <memory>
#include <string>

#include "argus/system/voxel/odometry/odometry.hpp"
#include "argus/common/component.hpp"

namespace argus {
namespace system {
namespace voxel {

/**
 * @brief Pluggable Voxel-SLAM module (common::Component contract).
 *
 * Owns vxslam::Odometry; Configure loads options via load_config / lua bridge.
 */
class VoxelModule : public common::Component {
public:
    VoxelModule();
    explicit VoxelModule(std::string name);

    const std::string& name() const override;
    bool Configure(const common::Component::Config& config) override;
    bool Start() override;
    void Stop() override;
    common::Health health() const override;

    vxslam::Odometry* odometry() { return odometry_.get(); }
    const vxslam::Odometry* odometry() const { return odometry_.get(); }

private:
    std::string name_;
    std::unique_ptr<vxslam::Odometry> odometry_;
    common::Health health_;
    bool running_ = false;
};

}  // namespace voxel
}  // namespace system
}  // namespace argus

#endif
