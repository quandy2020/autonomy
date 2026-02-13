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

#include <memory>
#include <string>

namespace autonomy {
namespace system {
namespace monitor {

/**
 * 监控器基类接口：采集一次数据并可选择性向 Prometheus 注册指标。
 */
class MonitorBase
{
public:
    virtual ~MonitorBase() = default;

    /// 监控器名称，用于日志与 Prometheus 标签
    virtual std::string Name() const = 0;

    /// 执行一次采集（更新内部状态或 Prometheus 指标）
    virtual void Collect() = 0;

    /// 是否已启用
    bool enabled() const {
        return enabled_;
    }
    void set_enabled(bool v) {
        enabled_ = v;
    }

    /// 向 Prometheus 注册指标（仅当 USE_PROMETHEUS 时由 MonitorRegistry 调用）。
    /// registry 实际类型为 prometheus::Registry*，在实现中转换后注册 Gauge/Counter 等。
    virtual void RegisterWithPrometheus(void* registry) {
        (void)registry;
    }

protected:
    bool enabled_{true};
};

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
