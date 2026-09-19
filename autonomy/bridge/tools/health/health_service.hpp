/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file health_service.hpp
 * @brief Helpers around gRPC default health check service.
 */

#pragma once

#include <string>

#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace tools {

/**
 * @brief Tracks desired overall health state for documentation / tests.
 *
 * Default health check is enabled via async_grpc Builder; this type records
 * the logical SERVING flag for ApplyPlatform / tests.
 */
class HealthServiceState
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(HealthServiceState)

    void SetServing(bool serving) { serving_ = serving; }
    bool serving() const { return serving_; }

    /** @brief Service name used with default health (empty = overall). */
    static const char* OverallServiceName() { return ""; }

private:
    bool serving_{true};
};

}  // namespace tools
}  // namespace bridge
}  // namespace autonomy
