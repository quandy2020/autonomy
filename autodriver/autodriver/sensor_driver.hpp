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
 * @brief Abstract hardware backend that captures timestamped sensor samples.
 */

#ifndef AUTODRIVER_SENSOR_DRIVER_HPP_
#define AUTODRIVER_SENSOR_DRIVER_HPP_

#include <functional>
#include <memory>

#include "autodriver/sensor_id.hpp"
#include "autodriver/types/sensor_sample.hpp"
#include "autodriver/types/sensor_type.hpp"

namespace autodriver {

/**
 * @class autodriver::SensorDriver
 * @brief Hardware backend that pushes timestamped samples on its capture thread.
 */
class SensorDriver {
public:
    /** @brief Callback invoked for each captured sample on the driver thread. */
    using SampleCallback =
        std::function<void(std::unique_ptr<SensorSample> sample)>;

    /** @brief Copy construction is disabled. */
    SensorDriver(const SensorDriver&) = delete;
    /** @brief Copy assignment is disabled. */
    SensorDriver& operator=(const SensorDriver&) = delete;
    /** @brief Virtual destructor for polymorphic drivers. */
    virtual ~SensorDriver() = default;

    /**
     * @brief Sensor modality implemented by this driver.
     * @return The sensor type handled by this backend.
     */
    virtual SensorType GetType() const = 0;

    /**
     * @brief Stable instance id from configuration.
     * @return Configured sensor identifier.
     */
    virtual const SensorId& GetSensorId() const = 0;

    /**
     * @brief Open the device and start the capture thread.
     * @return True on success.
     */
    virtual bool Start() = 0;

    /**
     * @brief Stop capture and release hardware resources.
     */
    virtual void Stop() = 0;

    /**
     * @brief Whether the capture thread is active.
     * @return True when capture is running.
     */
    virtual bool IsRunning() const = 0;

    /**
     * @brief Register the callback invoked for each captured sample.
     * @param callback Invoked on the driver's thread; must not block for long.
     */
    virtual void SetSampleCallback(SampleCallback callback) = 0;

protected:
    /** @brief Protected default constructor for derived drivers. */
    SensorDriver() = default;
};

}  // namespace autodriver

#endif  // AUTODRIVER_SENSOR_DRIVER_HPP_
