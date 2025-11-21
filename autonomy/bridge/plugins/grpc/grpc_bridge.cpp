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

#include "autonomy/common/time.hpp"
#include "autonomy/bridge/plugins/grpc/grpc_bridge.hpp"
#include "autonomy/bridge/plugins/grpc/handlers/exploration_handler.hpp"
#include "autonomy/bridge/plugins/grpc/handlers/navigation_handler.hpp"

namespace autonomy {
namespace bridge { 
namespace plugins {
namespace grpc {

namespace {

// static auto* kIncomingDataQueueMetric = metrics::Gauge::Null();
constexpr int kMaxMessageSize = 100 * 1024 * 1024;  // 100 MB
const autonomy::common::Duration kPopTimeout = autonomy::common::FromMilliseconds(100);

}  // namespace

GrpcBridgeServer::GrpcBridgeServer()
{
    autonomy::common::async_grpc::Server::Builder server_builder;
    server_builder.SetServerAddress("127.0.0.1");
    server_builder.SetNumGrpcThreads(4);
    server_builder.SetNumEventThreads(4);
    // server_builder.SetServerAddress(options_.host());
    // server_builder.SetNumGrpcThreads(options_.num_grpc_threads());
    // server_builder.SetNumEventThreads(options_.num_event_threads());
    server_builder.SetMaxSendMessageSize(kMaxMessageSize);

    if (!options_.uplink_server_address().empty()) {
    }

    server_builder.RegisterHandler<handlers::SendNavigationHandler>();
    server_builder.RegisterHandler<handlers::SendExplorationHandler>();
    grpc_server_ = server_builder.Build();
}

void GrpcBridgeServer::Start()
{
    shutting_down_ = false;
    StartThread();
    grpc_server_->Start();
}

void GrpcBridgeServer::WaitUntilIdle()
{

}

void GrpcBridgeServer::WaitForShutdown()
{
    grpc_server_->WaitForShutdown();

    if (task_thread_) {
        task_thread_->join();
    }

}

void GrpcBridgeServer::Shutdown()
{
    shutting_down_ = true;
    grpc_server_->Shutdown();

    if (task_thread_) {
        task_thread_->join();
        task_thread_.reset();
    }
}

void GrpcBridgeServer::ProcessSensorDataQueue() 
{
    LOG(INFO) << "Starting task handler thread.";
    while (!shutting_down_) {
        LOG(INFO) << "handler sensor datas.";
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
}

void GrpcBridgeServer::StartThread()
{
    CHECK(!task_thread_);

    // Start the ask handler processing thread.
    task_thread_ = std::make_unique<std::thread>(
        [this]() { this->ProcessSensorDataQueue(); });
}

}   // namespace grpc
}   // namespace plugins 
}   // namespace bridge
}   // namespace autonomy