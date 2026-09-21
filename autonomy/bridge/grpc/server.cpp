/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file server.cpp
 * @brief Implementation of grpc::Server Start / Wait / Shutdown.
 */

#include "autonomy/bridge/grpc/server.hpp"

#include <algorithm>

#include "autolink/autolink.hpp"
#include "autonomy/bridge/grpc/context.hpp"
#include "autonomy/bridge/grpc/handlers/register_handlers.hpp"
#include "autonomy/bridge/grpc/handlers/rpc_navigation_handlers.hpp"
#include "autonomy/bridge/grpc/handlers/rpc_follow_handlers.hpp"
#include "autonomy/bridge/grpc/handlers/rpc_charge_handlers.hpp"
#include "autonomy/bridge/grpc/handlers/rpc_teleop_handlers.hpp"
#include "autonomy/bridge/grpc/handlers/rpc_explore_handlers.hpp"
#include "autonomy/bridge/grpc/handlers/rpc_map_handlers.hpp"
#include "autonomy/bridge/grpc/handlers/rpc_localization_handlers.hpp"
#include "autonomy/bridge/grpc/handlers/rpc_voice_handlers.hpp"
#include "autonomy/bridge/grpc/handlers/rpc_system_handlers.hpp"
#include "autonomy/bridge/grpc/handlers/rpc_sensor_handlers.hpp"
#include "autonomy/bridge/tools/bootstrap.hpp"
#include "autolink/common/log.hpp"
#include "autonomy/common/time.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {

namespace {

constexpr uint32_t kDefaultGrpcPort = 5005;
constexpr size_t kDefaultGrpcThreads = 4;
constexpr size_t kDefaultEventThreads = 4;

std::string ServerAddress(const proto::GrpcOptions& options) {
    const std::string host =
        options.host().empty() ? "127.0.0.1" : options.host();
    const uint32_t port =
        options.port() == 0 ? kDefaultGrpcPort : options.port();
    return host + ":" + std::to_string(port);
}

size_t NumGrpcThreads(const proto::GrpcOptions& options) {
    return options.num_grpc_threads() > 0 ? options.num_grpc_threads()
                                          : kDefaultGrpcThreads;
}

size_t NumEventThreads(const proto::GrpcOptions& options) {
    return options.num_event_threads() > 0 ? options.num_event_threads()
                                           : kDefaultEventThreads;
}

int NumWorkerThreads(const proto::GrpcOptions& options) {
    if (options.num_worker_threads() > 0) {
        return static_cast<int>(options.num_worker_threads());
    }
    const size_t event_threads = NumEventThreads(options);
    return static_cast<int>(std::max<size_t>(2, event_threads / 2));
}

template <typename BuilderT>
void RegisterRpcHandlers(BuilderT& builder) {
    handlers::RegisterHandlers<
        handlers::RpcNavigateHandler, handlers::RpcNavigationCancelHandler,
        handlers::RpcNavigationPauseHandler, handlers::RpcNavigationResumeHandler,
        handlers::RpcNavigationGetStatusHandler, handlers::RpcFollowHandler,
        handlers::RpcFollowCancelHandler, handlers::RpcFollowPauseHandler,
        handlers::RpcFollowResumeHandler, handlers::RpcFollowGetStatusHandler,
        handlers::RpcChargeReturnHandler, handlers::RpcChargeLeaveHandler,
        handlers::RpcChargeCancelHandler, handlers::RpcChargePauseHandler,
        handlers::RpcChargeResumeHandler, handlers::RpcChargeGetStatusHandler,
        handlers::RpcTeleopVelocityHandler, handlers::RpcTeleopDriveOnHeadingHandler,
        handlers::RpcTeleopBackUpHandler, handlers::RpcTeleopSpinHandler,
        handlers::RpcTeleopCancelHandler, handlers::RpcTeleopPauseHandler,
        handlers::RpcTeleopResumeHandler, handlers::RpcTeleopGetStatusHandler,
        handlers::RpcExploreHandler, handlers::RpcExploreCancelHandler,
        handlers::RpcExplorePauseHandler, handlers::RpcExploreResumeHandler,
        handlers::RpcExploreGetStatusHandler, handlers::RpcExploreSetAreaHandler,
        handlers::RpcExploreSaveMapHandler, handlers::RpcVoiceExecuteHandler,
        handlers::RpcStartMappingHandler, handlers::RpcFinishMappingHandler,
        handlers::RpcCancelMappingHandler, handlers::RpcGetMappingStatusHandler,
        handlers::RpcListMapsHandler, handlers::RpcGetMapHandler,
        handlers::RpcGetMapMetadataHandler, handlers::RpcSaveMapHandler,
        handlers::RpcDeleteMapHandler, handlers::RpcSetCurrentMapHandler,
        handlers::RpcLocalizationGetPoseHandler, handlers::RpcLocalizationGetStatusHandler,
        handlers::RpcLocalizationSetInitialPoseHandler, handlers::RpcSystemHeartbeatHandler,
        handlers::RpcSystemGetInfoHandler, handlers::RpcSystemGetStatusHandler,
        handlers::RpcSystemGetHealthHandler,
        handlers::RpcSystemGetRobotFullInfoHandler,
        handlers::RpcSystemGetCapabilitiesHandler, handlers::RpcSystemEmergencyStopHandler,
        handlers::RpcSystemClearEmergencyStopHandler,
        handlers::RpcSystemCancelAllGoalsHandler,
        handlers::RpcSystemGetActiveGoalHandler,
        handlers::RpcSystemRestartModuleHandler,
        handlers::RpcSystemGetProcessStatusHandler,
        handlers::RpcSystemGetEventBundleHandler,
        handlers::RpcSystemStartOtaHandler,
        handlers::RpcSystemGetOtaStatusHandler,
        handlers::RpcSystemAbortOtaHandler,
        handlers::RpcSystemApplyConfigPackageHandler,
        handlers::RpcSensorListSensorsHandler,
        handlers::RpcSensorGetSampleHandler,
        handlers::RpcSensorGetParametersHandler,
        handlers::RpcSensorSetParametersHandler,
        handlers::RpcSensorSaveParametersHandler,
        handlers::RpcSensorLoadParametersHandler,
        handlers::RpcSensorRecordHandler,
        handlers::RpcSensorCancelRecordHandler,
        handlers::RpcSensorGetRecordStatusHandler>(builder);
}

}  // namespace

Server::Server(const proto::GrpcOptions& options,
                                   proto::RobotIdentityOptions identity,
                                   proto::CapabilitiesOptions capabilities)
    : options_{options},
      identity_{std::move(identity)},
      capabilities_{std::move(capabilities)} {
    const std::string server_address = ServerAddress(options_);
    autonomy::common::async_grpc::Server::Builder server_builder;
    server_builder.SetServerAddress(server_address);
    server_builder.SetNumGrpcThreads(NumGrpcThreads(options_));
    server_builder.SetNumEventThreads(NumEventThreads(options_));

    if (!options_.uplink_server_address().empty()) {
        AINFO << "gRPC uplink server address: "
                  << options_.uplink_server_address();
    }

    RegisterRpcHandlers(server_builder);
    tools::ApplyPlatform(server_builder, options_);

    grpc_server_ = server_builder.Build();
    if (!grpc_server_) {
        AERROR << "Failed to build gRPC bridge server for "
                   << server_address;
        return;
    }

    autolink_node_ = autolink::CreateNode("bridge_grpc");
    if (!autolink_node_) {
        AERROR << "Failed to create autolink node for gRPC bridge.";
        grpc_server_.reset();
        return;
    }

    work_scheduler_ =
        WorkScheduler::make_shared(NumWorkerThreads(options_));
    grpc_server_->SetExecutionContext(
        Context::make_unique(
            autolink_node_, work_scheduler_, identity_, capabilities_));
    configured_ = true;
    AINFO << "gRPC bridge configured to listen on " << server_address
              << " (worker_threads=" << work_scheduler_->num_threads() << ")";
}

bool Server::Start() {
    if (!configured_ || !grpc_server_) {
        AERROR << "gRPC bridge is not configured; cannot start.";
        return false;
    }

    if (!grpc_server_->Start()) {
        AERROR << "Failed to start gRPC bridge on "
                   << ServerAddress(options_);
        return false;
    }
    return true;
}

void Server::WaitUntilIdle() {}

void Server::WaitForShutdown() {
    if (grpc_server_) {
        grpc_server_->WaitForShutdown();
    }
}

void Server::Shutdown() {
    if (grpc_server_) {
        grpc_server_->Shutdown();
    }
    // Drop context (via server) before joining workers.
    grpc_server_.reset();
    work_scheduler_.reset();
}

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
