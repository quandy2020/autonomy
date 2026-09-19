/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/tools/bootstrap.hpp"

#include "autolink/common/log.hpp"
#include "autonomy/bridge/policy/auth/tls_options.hpp"
#include "autonomy/bridge/tools/reflection/reflection_service.hpp"
#include "autonomy/bridge/tools/transport/channel_args_builder.hpp"
#include "autonomy/bridge/tools/transport/credentials_factory.hpp"

namespace autonomy {
namespace bridge {
namespace tools {

void ApplyPlatform(common::async_grpc::Server::Builder& builder,
                   const proto::GrpcOptions& options,
                   PlatformApplyResult* result) {
    PlatformApplyResult local;
    PlatformApplyResult& out = result != nullptr ? *result : local;
    out = PlatformApplyResult{};

    // 1. ChannelArgs (message size + keepalive)
    const auto channel_args = ChannelArgsBuilder::Build(options);
    builder.AddChannelArguments(channel_args);
    // Also mirror max sizes onto classic Builder setters for consistency.
    if (options.max_receive_message_bytes() > 0) {
        builder.SetMaxReceiveMessageSize(options.max_receive_message_bytes());
    } else {
        builder.SetMaxReceiveMessageSize(
            ChannelArgsBuilder::kDefaultMaxMessageBytes);
    }
    if (options.max_send_message_bytes() > 0) {
        builder.SetMaxSendMessageSize(options.max_send_message_bytes());
    } else {
        builder.SetMaxSendMessageSize(
            ChannelArgsBuilder::kDefaultMaxMessageBytes);
    }
    out.channel_args_applied = true;

    // 2. Credentials
    const auto tls = policy::TlsOptions::FromGrpcOptions(options);
    out.tls_requested = tls.enabled;
    auto credentials = CredentialsFactory::Create(options);
    builder.SetServerCredentials(credentials);
    out.credentials_applied = true;
    // Factory falls back to insecure when cert/key missing; approximate
    // "active" by checking that TLS was requested and paths are non-empty.
    out.tls_active =
        tls.enabled && !tls.cert_path.empty() && !tls.key_path.empty();

    // 3. Health (docs default: on when conf sets enable_health_check)
    const bool health = options.enable_health_check();
    builder.EnableDefaultHealthCheckService(health);
    out.health_enabled = health;
    out.health_state.SetServing(health);

    // 4. Reflection
    out.reflection_library_available = ReflectionLibraryAvailable();
    const bool reflection = options.enable_server_reflection();
    if (reflection && !out.reflection_library_available) {
        AWARN << "ApplyPlatform: enable_server_reflection=true but "
                 "AUTONOMY_HAVE_GRPC_REFLECTION is not defined; skipping";
        builder.EnableProtoReflection(false);
        out.reflection_enabled = false;
    } else {
        builder.EnableProtoReflection(reflection);
        out.reflection_enabled = reflection && out.reflection_library_available;
    }

    // 5. Interceptors (fixed order)
    auto factories = interceptors::BuildInterceptorChain(
        options, &out.interceptors);
    for (auto& factory : factories) {
        builder.AddInterceptorFactory(std::move(factory));
    }

    AINFO << "ApplyPlatform: health=" << out.health_enabled
          << " reflection=" << out.reflection_enabled
          << " auth=" << out.interceptors.auth
          << " metadata=" << out.interceptors.metadata
          << " rate_limit=" << out.interceptors.rate_limit
          << " otel=" << out.interceptors.otel;
}

}  // namespace tools
}  // namespace bridge
}  // namespace autonomy
