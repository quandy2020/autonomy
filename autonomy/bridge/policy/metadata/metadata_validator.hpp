/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file metadata_validator.hpp
 * @brief Validates required gRPC metadata keys.
 */

#pragma once

#include <string>
#include <unordered_map>

#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace policy {

/**
 * @brief Result of metadata validation.
 */
struct MetadataValidationResult {
    AUTONOMY_SMART_PTR_DEFINITIONS(MetadataValidationResult)

    bool ok{true};
    std::string detail;
};

/**
 * @brief Checks required metadata entries (see docs/02_policy.md).
 */
class MetadataValidator
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(MetadataValidator)

    /**
     * @brief Construct validator.
     *
     * @param[in] require_robot_id When true, `x-robot-id` must be non-empty.
     */
    explicit MetadataValidator(bool require_robot_id = false);

    /**
     * @brief Validate a flat key→value map (keys expected lower-case).
     */
    MetadataValidationResult Validate(
        const std::unordered_map<std::string, std::string>& metadata) const;

private:
    bool require_robot_id_{false};
};

}  // namespace policy
}  // namespace bridge
}  // namespace autonomy
