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

#ifndef AUTODRIVER_SENSOR_DRIVER_HPP_
#define AUTODRIVER_SENSOR_DRIVER_HPP_

#include <functional>
#include <memory>

#include "autodriver/sensor_id.hpp"
#include "autodriver/types/sensor_sample.hpp"
#include "autodriver/types/sensor_type.hpp"

namespace autodriver {

// Hardware backend that pushes timestamped samples on its capture thread.
class SensorDriver {
public:
    using SampleCallback =
        std::function<void(std::unique_ptr<SensorSample> sample)>;

    SensorDriver(const SensorDriver&) = delete;
    SensorDriver& operator=(const SensorDriver&) = delete;
    virtual ~SensorDriver() = default;

    virtual SensorType GetType() const = 0;
    virtual const SensorId& GetSensorId() const = 0;
    virtual bool Start() = 0;
    virtual void Stop() = 0;
    virtual bool IsRunning() const = 0;

    // Invoked on the driver's thread; the callee must not block for long.
    virtual void SetSampleCallback(SampleCallback callback) = 0;

protected:
    SensorDriver() = default;
};

}  // namespace autodriver

#endif  // AUTODRIVER_SENSOR_DRIVER_HPP_
