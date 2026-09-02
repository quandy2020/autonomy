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

/**
 * @file
 * @brief class_loader plugin interface for sensor modules.
 */

#ifndef AUTODRIVER_SENSOR_MODULE_HPP_
#define AUTODRIVER_SENSOR_MODULE_HPP_

#include <functional>
#include <memory>

#include "autodriver/config.hpp"
#include "autodriver/sensor_id.hpp"
#include "autodriver/types/sensor_sample.hpp"
#include "autodriver/types/sensor_type.hpp"

namespace autodriver {

/**
 * @class autodriver::SensorModule
 * @brief Plugin lifecycle: default-construct, then Init / Start / Stop.
 */
class SensorModule {
public:
    /** @brief Upstream hook invoked when the module produces a sample. */
    using SampleHook = std::function<void(std::shared_ptr<SensorSample>)>;

    /**
     * @brief Per-sensor initialization context passed to Init().
     */
    struct Context {
        /** @brief Sensor entry parsed from YAML configuration. */
        Config::Sensor sensor;
        /** @brief Callback that receives samples after capture or stamping. */
        SampleHook hook;
    };

    /** @brief Copy construction is disabled. */
    SensorModule(const SensorModule&) = delete;
    /** @brief Copy assignment is disabled. */
    SensorModule& operator=(const SensorModule&) = delete;
    /** @brief Virtual destructor for polymorphic modules. */
    virtual ~SensorModule() = default;

    /**
     * @brief Sensor modality implemented by this module.
     * @return The sensor type handled by this plugin.
     */
    virtual SensorType GetType() const = 0;

    /**
     * @brief Stable instance id from configuration.
     * @return Configured sensor identifier.
     */
    virtual const SensorId& GetSensorId() const = 0;

    /**
     * @brief Bind configuration and sample hook before Start().
     * @param context Sensor entry from YAML and upstream sample hook.
     * @return False when driver creation fails.
     */
    virtual bool Init(const Context& context) = 0;

    /**
     * @brief Start hardware capture or mark the module as active.
     * @return True when capture starts successfully.
     */
    virtual bool Start() = 0;

    /**
     * @brief Stop capture and release resources.
     */
    virtual void Stop() = 0;

    /**
     * @brief Whether the module is currently capturing.
     * @return True when the module is running.
     */
    virtual bool IsRunning() const = 0;

protected:
    /** @brief Protected default constructor for derived modules. */
    SensorModule() = default;
};

}  // namespace autodriver

#endif  // AUTODRIVER_SENSOR_MODULE_HPP_
