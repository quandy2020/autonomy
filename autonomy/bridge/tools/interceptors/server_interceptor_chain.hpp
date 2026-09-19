/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file server_interceptor_chain.hpp
 * @brief Build the fixed Logging→Auth→Metadata→RateLimit→OTel factory list.
 */

#pragma once

#include <memory>
#include <vector>

#include "autonomy/bridge/proto/grpc_options.pb.h"
#include "autonomy/common/macros.hpp"
#include "grpcpp/support/server_interceptor.h"

namespace autonomy {
namespace bridge {
namespace tools {
namespace interceptors {

/**
 * @brief Observable summary of which factories were appended.
 */
struct InterceptorChainPlan {
    AUTONOMY_SMART_PTR_DEFINITIONS(InterceptorChainPlan)

    bool logging{true};
    bool auth{false};
    bool metadata{false};
    bool rate_limit{false};
    bool otel{false};
};

/**
 * @brief Append interceptor factories in the documented fixed order.
 *
 * @param[in]  options GrpcOptions switches.
 * @param[out] plan    Optional summary for tests / logging.
 * @return Owned factory list ready for Builder::AddInterceptorFactory.
 */
std::vector<
    std::unique_ptr<::grpc::experimental::ServerInterceptorFactoryInterface>>
BuildInterceptorChain(const proto::GrpcOptions& options,
                      InterceptorChainPlan* plan = nullptr);

}  // namespace interceptors
}  // namespace tools
}  // namespace bridge
}  // namespace autonomy
