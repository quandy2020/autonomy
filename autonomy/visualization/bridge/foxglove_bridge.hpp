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

#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "autonomy/common/macros.hpp"
#include "foxglove/server.hpp"

namespace autonomy {
namespace visualization {

namespace transport {
// 前向声明
class AutoDiscovery;
}  // namespace transport

/**
 * @brief FoxgloveBridge：管理 Foxglove WebSocket 服务器
 *
 * 功能：
 *  - 启动/停止 WebSocket 服务器
 *  - 管理 AutoDiscovery（负责 autolink topic 发现和转发）
 */
class FoxgloveBridge : public std::enable_shared_from_this<FoxgloveBridge>
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(FoxgloveBridge)

    /// 服务器配置选项
    struct Options {
        std::string host = "0.0.0.0";
        uint16_t port = 8765;
        bool enable_client_publish = true;  // 允许 Foxglove Studio 发布消息
        bool enable_connection_graph = true;
        bool enable_parameters = false;  // 启用参数功能
        bool enable_time = false;        // 启用时间功能
        bool enable_services = false;    // 启用服务功能
    };

    FoxgloveBridge();

    explicit FoxgloveBridge(const Options& options);
    ~FoxgloveBridge();

    /// start server
    bool Start();

    /// stop server
    void Stop();

    /// check if server is running
    bool IsRunning() const;

    /// get listening port
    uint16_t GetPort() const;

private:
    void SetupCallbacks(::foxglove::WebSocketServerCallbacks& callbacks);

    // server options
    Options options_;

    // Foxglove WebSocket 服务器
    std::unique_ptr<::foxglove::WebSocketServer> server_{nullptr};

    // WebSocket server options
    foxglove::WebSocketServerOptions ws_options_;

    // autolink topic discovery and forwarding
    std::unique_ptr<transport::AutoDiscovery> discovery_{nullptr};

    // server state
    mutable std::mutex state_mutex_;
    bool is_running_ = false;
};
}  // namespace visualization
}  // namespace autonomy
