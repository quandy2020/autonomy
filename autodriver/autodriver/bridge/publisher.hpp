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

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include "autodriver/sample_sink.hpp"
#include "autolink/base/atomic_rw_lock.hpp"
#include "autolink/node/node.hpp"
#include "autolink/node/writer.hpp"
#include <automsgs/msgs/diagnostic_msgs/diagnostic_array.pb.h>
#include "autolink/common/macros.hpp"

namespace autodriver {
namespace bridge {

/**
 * @class autodriver::bridge::Publisher
 * @brief Owns the Autolink Node and Writers; core autodriver never calls Write().
 *
 * HandleSensorSample enqueues work for a dedicated publish thread so driver
 * callbacks are not blocked on protobuf/DDS Write. When the queue is full the
 * oldest pending sample is dropped (lossy backpressure).
 */
class Publisher : public SampleSink {
public:
  /**
   * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
   */
  AUTOLINK_SHARED_PTR_DEFINITIONS(Publisher)

  /**
   * @brief Disable copy construction and copy assignment.
   */
  DISALLOW_COPY_AND_ASSIGN(Publisher)

    /**
     * @brief Stores the Autolink node name used for writers.
     * @param node_name Autolink node name.
     * @param async_queue_capacity Bound for the publish queue (min 1).
     */
    explicit Publisher(std::string node_name = "autodriver",
                       std::size_t async_queue_capacity = 64);

    /**
     * @brief Tears down the publish thread, writers, and the Autolink node.
     */
    ~Publisher() override;

  /**
     * @brief Creates the Autolink node if not already present.
     */
    bool Initialize();

    /**
     * @brief Access the underlying Autolink node
     * @return Raw pointer to the node, or nullptr before Initialize()
     */
    autolink::Node* GetNode() { return node_.get(); }

    /**
     * @brief Opens protobuf writers for a newly attached sensor.
     */
    bool HandleSensorAttach(const Config::Sensor& sensor, SensorType type) override;

    /**
     * @brief Removes writers when a sensor detaches.
     */
    void HandleSensorDetach(const SensorId& id) override;

    /**
     * @brief Enqueues a sample for asynchronous protobuf Write.
     */
    void HandleSensorSample(std::shared_ptr<SensorSample> sample) override;

    /**
     * @brief Publishes DiagnosticArray on /diagnostics (created lazily).
     */
    void HandleDiagnostic(const diagnostics::DiagnosticSnapshot& snapshot) override;

    /**
     * @brief Optional override for the diagnostics channel (default /diagnostics).
     */
    void SetDiagnosticsChannel(std::string channel);

private:
    // Callable that serializes and writes one sample type.
    using WriteFn = std::function<void(const std::shared_ptr<SensorSample>&)>;

    struct PendingSample {
        WriteFn write;
        std::shared_ptr<SensorSample> sample;
    };

    /**
     * @brief Open typed protobuf writers and register multi-channel fanout.
     * @tparam kType SensorType specialization selecting the protobuf message.
     * @param sensor Sensor configuration from YAML.
     * @return True when the writer was opened successfully.
     */
    template <SensorType kType>
    bool OpenTypedWriter(const Config::Sensor& sensor);

    /**
     * @brief Opens image and optional CameraInfo writers for a camera sensor.
     */
    bool OpenCameraWriters(const Config::Sensor& sensor);

    /**
     * @brief Starts the background publish worker if not already running.
     */
    void EnsurePublishThread();

    /**
     * @brief Stops the publish worker and drains/drops the queue.
     */
    void StopPublishThread();

    /**
     * @brief Worker loop: pop PendingSample and invoke WriteFn.
     */
    void PublishLoop();

    // Autolink node name passed at construction.
    std::string node_name_;

    // Shared Autolink node owning all writers.
    std::shared_ptr<autolink::Node> node_{nullptr};

    // Per-sensor write dispatch table keyed by SensorId.
    std::unordered_map<SensorId, WriteFn> writers_;

    // Autolink diagnostics writer (lazy).
    std::shared_ptr<
        autolink::Writer<automsgs::msgs::diagnostic_msgs::DiagnosticArray>>
        diagnostics_writer_{nullptr};
    std::string diagnostics_channel_ = "/diagnostics";

    // Protects writers_ during attach/detach/sample lookup.
    mutable autolink::base::AtomicRWLock lock_;

    // Async publish queue (lossy when full).
    std::size_t async_capacity_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::deque<PendingSample> queue_;
    std::atomic<bool> publish_running_{false};
    std::thread publish_thread_;
};

}  // namespace bridge
}  // namespace autodriver

#endif  // AUTODRIVER_BRIDGE_PUBLISHER_HPP_
