/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/policy/metadata/metadata_validator.hpp"

#include "autonomy/bridge/policy/metadata/metadata_keys.hpp"

namespace autonomy {
namespace bridge {
namespace policy {

MetadataValidator::MetadataValidator(bool require_robot_id)
    : require_robot_id_(require_robot_id) {}

MetadataValidationResult MetadataValidator::Validate(
    const std::unordered_map<std::string, std::string>& metadata) const {
    MetadataValidationResult result;
    if (!require_robot_id_) {
        return result;
    }
    const auto it = metadata.find(kMetadataKeyRobotId);
    if (it == metadata.end() || it->second.empty()) {
        result.ok = false;
        result.detail = "missing x-robot-id";
    }
    return result;
}

}  // namespace policy
}  // namespace bridge
}  // namespace autonomy
