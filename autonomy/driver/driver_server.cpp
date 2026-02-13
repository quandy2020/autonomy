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

#include "autonomy/driver/driver_server.hpp"

#include <memory>

#include "autolink/common/log.hpp"
#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/driver/data_router.hpp"
#include "autonomy/driver/driver_engine.hpp"

namespace autonomy {
namespace driver {

DriverServer::DriverServer(const proto::DriverOptions& options) : options_(options) {
    // 创建 autolink 节点
    node_ptr_ = ::autolink::CreateNode("driver_server_node", "");
    node_ = node_ptr_.get();

    // 创建驱动引擎
    driver_engine_ = std::make_shared<DriverEngine>(node_);

    // 创建数据路由器
    data_router_ = std::make_shared<DataRouter>(node_, driver_engine_.get());

    AINFO << "DriverServer created.";
}

DriverServer::~DriverServer() {
    Stop();
    WaitForShutdown();
    AINFO << "DriverServer destroyed.";
}

bool DriverServer::Initialize() {
    if (initialized_) {
        AWARN << "DriverServer already initialized";
        return true;
    }

    if (node_ == nullptr) {
        AERROR << "DriverServer: node is null, cannot initialize";
        return false;
    }

    // 初始化驱动引擎
    if (!driver_engine_->Initialize(options_)) {
        AERROR << "DriverServer: failed to initialize DriverEngine";
        return false;
    }

    // 初始化数据路由器
    if (!data_router_->Initialize(options_)) {
        AERROR << "DriverServer: failed to initialize DataRouter";
        return false;
    }

    initialized_ = true;
    AINFO << "DriverServer initialized successfully.";
    return true;
}

void DriverServer::Start() {
    if (!initialized_) {
        AERROR << "DriverServer not initialized. Call Initialize() first.";
        return;
    }

    if (started_) {
        AWARN << "DriverServer already started";
        return;
    }

    // 启动驱动引擎
    driver_engine_->Start();

    // 启动数据路由器
    data_router_->Start();

    // 如果配置了自动启动，则启动所有传感器订阅
    if (options_.auto_start()) {
        AINFO << "DriverServer: auto_start enabled, sensors are ready";
    }

    started_ = true;
    AINFO << "DriverServer started.";
}

void DriverServer::Stop() {
    if (!this->started_) {
        return;
    }

    // 停止数据路由器
    if (this->data_router_ != nullptr) {
        this->data_router_->Stop();
    }

    // 停止驱动引擎
    if (this->driver_engine_ != nullptr) {
        this->driver_engine_->Stop();
    }

    this->started_ = false;
    AINFO << "DriverServer stopped.";
}

void DriverServer::WaitForShutdown() {
    // 等待所有任务完成
    // 目前没有后台线程，所以直接返回
    // 如果需要，可以添加等待逻辑
    AINFO << "DriverServer shutdown complete.";
}

}  // namespace driver
}  // namespace autonomy