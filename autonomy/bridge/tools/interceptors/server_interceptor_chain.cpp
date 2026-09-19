/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/tools/interceptors/server_interceptor_chain.hpp"

#include "autonomy/bridge/policy/auth/token_authenticator.hpp"
#include "autonomy/bridge/policy/metadata/metadata_validator.hpp"
#include "autonomy/bridge/policy/rate_limit/token_bucket.hpp"
#include "autonomy/bridge/tools/interceptors/auth_interceptor.hpp"
#include "autonomy/bridge/tools/interceptors/logging_interceptor.hpp"
#include "autonomy/bridge/tools/interceptors/metadata_interceptor.hpp"
#include "autonomy/bridge/tools/interceptors/otel_interceptor.hpp"
#include "autonomy/bridge/tools/interceptors/rate_limit_interceptor.hpp"
#include "autonomy/bridge/tools/otel/tracer_provider.hpp"

namespace autonomy {
namespace bridge {
namespace tools {
namespace interceptors {

std::vector<
    std::unique_ptr<::grpc::experimental::ServerInterceptorFactoryInterface>>
BuildInterceptorChain(const proto::GrpcOptions& options,
                      InterceptorChainPlan* plan) {
    InterceptorChainPlan local;
    InterceptorChainPlan& out = plan != nullptr ? *plan : local;
    out = InterceptorChainPlan{};

    std::vector<std::unique_ptr<
        ::grpc::experimental::ServerInterceptorFactoryInterface>>
        factories;

    // 1. Logging (always on)
    factories.emplace_back(std::make_unique<LoggingInterceptorFactory>());
    out.logging = true;

    // 2. Auth
    auto authenticator =
        std::make_shared<policy::TokenAuthenticator>(options);
    if (authenticator->mode() == proto::AUTH_MODE_BEARER_TOKEN) {
        factories.emplace_back(
            std::make_unique<AuthInterceptorFactory>(authenticator));
        out.auth = true;
    }

    // 3. Metadata
    if (options.enable_metadata_interceptor()) {
        auto validator = std::make_shared<policy::MetadataValidator>(
            options.require_robot_id_metadata());
        factories.emplace_back(
            std::make_unique<MetadataInterceptorFactory>(validator));
        out.metadata = true;
    }

    // 4. Rate limit
    if (options.enable_rate_limit() && options.rate_limit_qps() > 0.0) {
        auto bucket = std::make_shared<policy::TokenBucket>(
            options.rate_limit_qps(), options.rate_limit_burst());
        factories.emplace_back(
            std::make_unique<RateLimitInterceptorFactory>(bucket));
        out.rate_limit = true;
    }

    // 5. OTel (always register a provider; no-op unless enabled)
    if (options.enable_opentelemetry()) {
        auto provider = std::make_shared<otel::NoopTracerProvider>();
        factories.emplace_back(
            std::make_unique<OtelInterceptorFactory>(provider));
        out.otel = true;
    }

    return factories;
}

}  // namespace interceptors
}  // namespace tools
}  // namespace bridge
}  // namespace autonomy
