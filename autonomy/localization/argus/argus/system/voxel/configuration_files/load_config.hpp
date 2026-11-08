/*
 * Copyright 2026 The Openbot Authors
 */

#ifndef ARGUS_VXSLAM_LOAD_CONFIG_HPP_
#define ARGUS_VXSLAM_LOAD_CONFIG_HPP_

#include <string>

#include "argus/system/voxel/odometry/odometry_config.hpp"

namespace argus {
namespace vxslam {

/**
 * @brief Load OdometryConfig from OpenCV FileStorage YAML.
 * @return False on I/O or parse failure.
 */
bool LoadConfig(const std::string& yaml_path,
                                 OdometryConfig* out);

}  // namespace vxslam
}  // namespace argus

#endif  // ARGUS_VXSLAM_LOAD_CONFIG_HPP_
