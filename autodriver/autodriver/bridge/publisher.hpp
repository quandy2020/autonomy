/*
 * Copyright 2026 Autodriver contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file
 * @brief Autolink bridge that publishes sensor samples to configured channels.
 */

#ifndef AUTODRIVER_BRIDGE_PUBLISHER_HPP_
#define AUTODRIVER_BRIDGE_PUBLISHER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "autodriver/sample_sink.hpp"
#include "autolink/base/atomic_rw_lock.hpp"
#include "autolink/node/node.hpp"

namespace autodriver {
namespace bridge {

/**
 * @class autodriver::bridge::Publisher
 * @brief Owns the Autolink Node and Writers; core autodriver never calls Write().
 */
class Publisher : public SampleSink {
public:
    /**
     * @brief Stores the Autolink node name used for writers.
     */
    explicit Publisher(std::string node_name = "autodriver");

    /**
     * @brief Tears down all writers and the Autolink node.
     */
    ~Publisher() override;

    Publisher(const Publisher&) = delete;

    /**
     * @brief Copy assignment operator (deleted)
     */
    Publisher& operator=(const Publisher&) = delete;

    /**
     * @brief Creates the Autolink node if not already present.
     */
    bool Initialize();

    /**
     * @brief Access the underlying Autolink node
     * @return Raw pointer to the node, or nullptr before Initialize()
     */
    autolink::Node* node() { return node_.get(); }

    /**
     * @brief Opens protobuf writers for a newly attached sensor.
     */
    bool OnAttach(const Config::Sensor& sensor, SensorType type) override;

    /**
     * @brief Removes writers when a sensor detaches.
     */
    void OnDetach(const SensorId& id) override;

    /**
     * @brief Converts a sample to protobuf and writes it to the sensor channel.
     */
    void OnSample(std::shared_ptr<SensorSample> sample) override;

private:
    // Callable that serializes and writes one sample type.
    using WriteFn = std::function<void(const std::shared_ptr<SensorSample>&)>;

    /**
     * @brief Open a typed Autolink writer for a non-camera sensor
     * @tparam kType SensorType specialization selecting the protobuf message
     * @param sensor Sensor configuration from YAML
     * @return True when the writer was opened successfully
     */
    template <SensorType kType>
    /**
     * @brief Opens typed protobuf writers and registers a multi-channel fanout.
     */
    bool OpenWriter(const Config::Sensor& sensor);

    /**
     * @brief Opens image and optional CameraInfo writers for a camera sensor.
     */
    bool OpenCameraWriter(const Config::Sensor& sensor);

    // Autolink node name passed at construction.
    std::string node_name_;

    // Shared Autolink node owning all writers.
    std::shared_ptr<autolink::Node> node_;

    // Per-sensor write dispatch table keyed by SensorId.
    std::unordered_map<SensorId, WriteFn> writers_;

    // Protects writers_ during attach/detach/sample.
    mutable autolink::base::AtomicRWLock lock_;
};

}  // namespace bridge
}  // namespace autodriver

#endif  // AUTODRIVER_BRIDGE_PUBLISHER_HPP_
