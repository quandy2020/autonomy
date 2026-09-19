/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file bootstrap.hpp
 * @brief Apply Bridge gRPC platform settings onto async_grpc::Server::Builder.
 */

#pragma once

#include "autonomy/bridge/proto/grpc_options.pb.h"
#include "autonomy/bridge/tools/health/health_service.hpp"
#include "autonomy/bridge/tools/interceptors/server_interceptor_chain.hpp"
#include "autonomy/common/async_grpc/server.h"
#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace tools {

/**
 * @brief Observable side-effects of ApplyPlatform (for tests / logging).
 */
struct PlatformApplyResult {
    AUTONOMY_SMART_PTR_DEFINITIONS(PlatformApplyResult)

    bool channel_args_applied{false};
    bool credentials_applied{false};
    bool health_enabled{false};
    bool reflection_enabled{false};
    bool reflection_library_available{false};
    bool tls_requested{false};
    bool tls_active{false};
    interceptors::InterceptorChainPlan interceptors;
    HealthServiceState health_state;
};

/**
 * @brief Configure Builder from GrpcOptions (ChannelArgs, credentials, health,
 *        reflection, interceptors).
 *
 * Call after RegisterHandlers and before Builder::Build().
 *
 * @param[in,out] builder async_grpc server builder.
 * @param[in]     options GrpcOptions from Bridge conf.
 * @param[out]    result  Optional apply summary for tests.
 */
void ApplyPlatform(common::async_grpc::Server::Builder& builder,
                   const proto::GrpcOptions& options,
                   PlatformApplyResult* result = nullptr);

}  // namespace tools
}  // namespace bridge
}  // namespace autonomy
