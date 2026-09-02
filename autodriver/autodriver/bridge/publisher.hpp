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
     * @brief Constructor for autodriver::bridge::Publisher
     * @param node_name Autolink node name (default "autodriver")
     */
    explicit Publisher(std::string node_name = "autodriver");

    /**
     * @brief Destructor for autodriver::bridge::Publisher
     */
    ~Publisher() override;

    /**
     * @brief Copy constructor (deleted)
     */
    Publisher(const Publisher&) = delete;

    /**
     * @brief Copy assignment operator (deleted)
     */
    Publisher& operator=(const Publisher&) = delete;

    /**
     * @brief Create the Autolink node.
     * @return True on success
     */
    bool Initialize();

    /**
     * @brief Access the underlying Autolink node
     * @return Raw pointer to the node, or nullptr before Initialize()
     */
    autolink::Node* node() { return node_.get(); }

    /**
     * @brief Register a writer for a sensor channel
     * @param sensor Sensor configuration from YAML
     * @param type Sensor message type
     * @return True when the writer was opened successfully
     */
    bool OnAttach(const Config::Sensor& sensor, SensorType type) override;

    /**
     * @brief Tear down the writer for a detached sensor
     * @param id Sensor identifier
     */
    void OnDetach(const SensorId& id) override;

    /**
     * @brief Publish a sensor sample to its registered writer
     * @param sample Decoded sensor payload
     */
    void OnSample(std::shared_ptr<SensorSample> sample) override;

private:
    /** @brief Callable that serializes and writes one sample type. */
    using WriteFn = std::function<void(const std::shared_ptr<SensorSample>&)>;

    /**
     * @brief Open a typed Autolink writer for a non-camera sensor
     * @tparam kType SensorType specialization selecting the protobuf message
     * @param sensor Sensor configuration from YAML
     * @return True when the writer was opened successfully
     */
    template <SensorType kType>
    bool OpenWriter(const Config::Sensor& sensor);

    /**
     * @brief Open image and paired camera_info writers for RealSense cameras.
     * @param sensor Sensor configuration from YAML
     * @return True when both writers were opened successfully
     */
    bool OpenCameraWriter(const Config::Sensor& sensor);

    /** @brief Autolink node name passed at construction. */
    std::string node_name_;

    /** @brief Shared Autolink node owning all writers. */
    std::shared_ptr<autolink::Node> node_;

    /** @brief Per-sensor write dispatch table keyed by SensorId. */
    std::unordered_map<SensorId, WriteFn> writers_;

    /** @brief Protects writers_ during attach/detach/sample. */
    mutable autolink::base::AtomicRWLock lock_;
};

}  // namespace bridge
}  // namespace autodriver

#endif  // AUTODRIVER_BRIDGE_PUBLISHER_HPP_
