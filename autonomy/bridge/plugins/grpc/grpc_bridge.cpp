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

#include "autonomy/bridge/plugins/grpc/grpc_bridge.hpp"

#include "autolink/autolink.hpp"
#include "autonomy/bridge/plugins/grpc/grpc_bridge_context.hpp"
#include "autonomy/bridge/plugins/grpc/handlers/exploration_handler.hpp"
#include "autonomy/bridge/plugins/grpc/handlers/navigation_handler.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/common/time.hpp"

namespace autonomy {
namespace bridge {
namespace plugins {
namespace grpc {

namespace {

constexpr int kMaxMessageSize = 100 * 1024 * 1024;  // 100 MB
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

}  // namespace

GrpcBridgeServer::GrpcBridgeServer(const proto::GrpcOptions& options)
    : options_{options} {
    const std::string server_address = ServerAddress(options_);
    autonomy::common::async_grpc::Server::Builder server_builder;
    server_builder.SetServerAddress(server_address);
    server_builder.SetNumGrpcThreads(NumGrpcThreads(options_));
    server_builder.SetNumEventThreads(NumEventThreads(options_));
    server_builder.SetMaxSendMessageSize(kMaxMessageSize);

    if (!options_.uplink_server_address().empty()) {
        LOG(INFO) << "gRPC uplink server address: "
                  << options_.uplink_server_address();
    }

    server_builder.RegisterHandler<handlers::SendNavigationHandler>();
    server_builder.RegisterHandler<handlers::SendExplorationHandler>();
    grpc_server_ = server_builder.Build();
    if (!grpc_server_) {
        LOG(ERROR) << "Failed to build gRPC bridge server for "
                   << server_address;
        return;
    }

    autolink_node_ = autolink::CreateNode("bridge_grpc");
    if (!autolink_node_) {
        LOG(ERROR) << "Failed to create autolink node for gRPC bridge.";
        grpc_server_.reset();
        return;
    }

    grpc_server_->SetExecutionContext(
        std::make_unique<GrpcBridgeContextInterface>(autolink_node_));
    configured_ = true;
    LOG(INFO) << "gRPC bridge configured to listen on " << server_address;
}

bool GrpcBridgeServer::Start() {
    if (!configured_ || !grpc_server_) {
        LOG(ERROR) << "gRPC bridge is not configured; cannot start.";
        return false;
    }

    shutting_down_ = false;
    StartThread();
    if (!grpc_server_->Start()) {
        LOG(ERROR) << "Failed to start gRPC bridge on "
                   << ServerAddress(options_);
        shutting_down_ = true;
        if (task_thread_) {
            task_thread_->join();
            task_thread_.reset();
        }
        return false;
    }
    return true;
}

void GrpcBridgeServer::WaitUntilIdle() {}

void GrpcBridgeServer::WaitForShutdown() {
    if (grpc_server_) {
        grpc_server_->WaitForShutdown();
    }

    if (task_thread_) {
        task_thread_->join();
    }
}

void GrpcBridgeServer::Shutdown() {
    shutting_down_ = true;
    if (grpc_server_) {
        grpc_server_->Shutdown();
    }

    if (task_thread_) {
        task_thread_->join();
        task_thread_.reset();
    }
}

void GrpcBridgeServer::ProcessSensorDataQueue() {
    LOG(INFO) << "Starting task handler thread.";
    while (!shutting_down_) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
}

void GrpcBridgeServer::StartThread() {
    CHECK(!task_thread_);

    task_thread_ = std::make_unique<std::thread>(
        [this]() { this->ProcessSensorDataQueue(); });
}

}  // namespace grpc
}  // namespace plugins
}  // namespace bridge
}  // namespace autonomy
