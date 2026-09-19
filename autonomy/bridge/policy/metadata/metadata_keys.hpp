/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file metadata_keys.hpp
 * @brief Canonical gRPC metadata key strings for Bridge policy.
 */

#pragma once

namespace autonomy {
namespace bridge {
namespace policy {

/** @brief Optional / required robot identity metadata key. */
inline constexpr const char* kMetadataKeyRobotId = "x-robot-id";

/** @brief Bearer authorization metadata key. */
inline constexpr const char* kMetadataKeyAuthorization = "authorization";

/** @brief Optional request correlation id. */
inline constexpr const char* kMetadataKeyRequestId = "x-request-id";

}  // namespace policy
}  // namespace bridge
}  // namespace autonomy
