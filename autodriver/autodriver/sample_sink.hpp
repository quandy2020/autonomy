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
 * @brief Output interface for sensor attach/detach and published samples.
 */

#ifndef AUTODRIVER_SAMPLE_SINK_HPP_
#define AUTODRIVER_SAMPLE_SINK_HPP_

#include <memory>

#include "autodriver/config.hpp"
#include "autodriver/sensor_id.hpp"
#include "autodriver/types/sensor_sample.hpp"
#include "autodriver/types/sensor_type.hpp"

namespace autodriver {

/**
 * @class autodriver::SampleSink
 * @brief Receives lifecycle events and samples; Autolink publishing lives in bridge/.
 */
class SampleSink {
public:
    /** @brief Virtual destructor for polymorphic sinks. */
    virtual ~SampleSink() = default;

    /**
     * @brief Open writers or channels when a sensor is attached.
     * @param sensor Sensor configuration used to create output channels.
     * @param type Sensor modality of the attached instance.
     * @return False when channel setup fails.
     */
    virtual bool OnAttach(const Config::Sensor& sensor, SensorType type) = 0;

    /**
     * @brief Tear down writers when a sensor is detached.
     * @param id Sensor identifier whose output channels should be closed.
     */
    virtual void OnDetach(const SensorId& id) = 0;

    /**
     * @brief Publish or forward a captured sample.
     * @param sample Shared sample to emit downstream.
     */
    virtual void OnSample(std::shared_ptr<SensorSample> sample) = 0;

protected:
    /** @brief Protected default constructor for derived sinks. */
    SampleSink() = default;
};

}  // namespace autodriver

#endif  // AUTODRIVER_SAMPLE_SINK_HPP_
