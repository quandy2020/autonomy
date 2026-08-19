/*
 * Copyright 2026 Autodriver contributors
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

#ifndef AUTODRIVER_SENSOR_MODULE_HPP_
#define AUTODRIVER_SENSOR_MODULE_HPP_

#include <functional>
#include <memory>
#include <string>
#include <string_view>

#include "autodriver/config.hpp"
#include "autodriver/sensor_id.hpp"
#include "autodriver/types/sensor_sample.hpp"
#include "autodriver/types/sensor_type.hpp"
#include "autolink/node/node.hpp"
#include "autolink/proto/role_attributes.pb.h"

namespace autodriver {

// class_loader plugin: default-constructible, then Init/Start/Stop.
class SensorModule {
public:
    static constexpr int kWriterDepth = 10;

    using SampleHook = std::function<void(std::shared_ptr<SensorSample>)>;

    struct Context {
        std::shared_ptr<autolink::Node> node;
        Config::Sensor sensor;
        SampleHook hook;
    };

    SensorModule(const SensorModule&) = delete;
    SensorModule& operator=(const SensorModule&) = delete;
    virtual ~SensorModule() = default;

    virtual SensorType GetType() const = 0;
    virtual const SensorId& GetSensorId() const = 0;
    virtual bool Init(const Context& context) = 0;
    virtual bool Start() = 0;
    virtual void Stop() = 0;
    virtual bool IsRunning() const = 0;

    static autolink::proto::RoleAttributes WriterAttr(std::string_view channel) {
        autolink::proto::RoleAttributes attr;
        attr.set_channel_name(std::string(channel));
        auto* qos = attr.mutable_qos_profile();
        qos->set_history(autolink::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
        qos->set_depth(kWriterDepth);
        return attr;
    }

protected:
    SensorModule() = default;
};

}  // namespace autodriver

#endif  // AUTODRIVER_SENSOR_MODULE_HPP_
