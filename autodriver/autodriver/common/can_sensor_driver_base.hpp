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
 * @brief CRTP base for SocketCAN SensorDriver wrappers around CanReceiver.
 *
 * Derived registers ProtocolData on @c receiver().manager() in its constructor
 * and implements `GetSensorType()`. Event → sample conversion stays in Derived.
 */

#ifndef AUTODRIVER_COMMON_CAN_SENSOR_DRIVER_BASE_HPP_
#define AUTODRIVER_COMMON_CAN_SENSOR_DRIVER_BASE_HPP_

#include <memory>
#include <string>
#include <utility>

#include "autodriver/canbus/can_receiver.hpp"
#include "autodriver/driver_params.hpp"
#include "autodriver/sensor_driver.hpp"

namespace autodriver {
namespace hardware {

/**
 * @class autodriver::hardware::CanSensorDriverBase
 * @brief Shared Start/Stop/callback wiring for CAN IMU/GPS-style drivers.
 *
 * @tparam Derived Concrete driver (CRTP).
 * @tparam EventT Message type managed by CanReceiver / MessageManager.
 *
 * YAML knobs used by the base: `interface` (default `can0`). Poll timeout is
 * fixed at construction (default 100 ms).
 */
template <typename Derived, typename EventT>
class CanSensorDriverBase : public SensorDriver {
public:
    /**
     * @brief Store identity / params and the CanReceiver poll timeout.
     * @param id Sensor instance id from YAML.
     * @param params Cold-path driver params (`interface`, CAN ids, scales, …).
     * @param poll_timeout_ms Timeout passed to CanReceiver::Start; ≤0 → 100.
     */
    CanSensorDriverBase(SensorId id, DriverParams params,
                        int poll_timeout_ms = 100)
        : id_(std::move(id)),
          params_(std::move(params)),
          poll_timeout_ms_(poll_timeout_ms > 0 ? poll_timeout_ms : 100) {}

    /**
     * @brief Stops the CAN receive loop if still running.
     */
    ~CanSensorDriverBase() override { Stop(); }

    /**
     * @brief Stable instance id from configuration.
     * @return Configured sensor identifier.
     */
    const SensorId& GetSensorId() const override { return id_; }

    /**
     * @brief Open SocketCAN via @c receiver_ using YAML `interface`.
     * @return true when CanReceiver::Start succeeds (or already running).
     */
    bool Start() override {
        const std::string interface_name =
            GetString(params_, "interface", "can0");
        return receiver_.Start(interface_name, poll_timeout_ms_);
    }

    /**
     * @brief Stop the CanReceiver worker and close the socket.
     */
    void Stop() override { receiver_.Stop(); }

    /**
     * @brief Whether the CAN receive loop is active.
     * @return true while CanReceiver reports running.
     */
    bool IsRunning() const override { return receiver_.IsRunning(); }

    /**
     * @brief Register the sample sink callback.
     * @param callback May be empty to disable emission.
     */
    void SetSampleCallback(SampleCallback callback) override {
        callback_ = std::move(callback);
    }

protected:
    /**
     * @brief CRTP access to the concrete driver.
     * @return Mutable Derived reference.
     */
    Derived& self() { return static_cast<Derived&>(*this); }

    /**
     * @brief Const CRTP access to the concrete driver.
     * @return Const Derived reference.
     */
    const Derived& self() const { return static_cast<const Derived&>(*this); }

    /**
     * @brief Access the embedded CanReceiver (for Register / SetPublishCallback).
     * @return Mutable receiver reference.
     */
    canbus::CanReceiver<EventT>& receiver() { return receiver_; }

    /**
     * @brief Const access to the embedded CanReceiver.
     * @return Const receiver reference.
     */
    const canbus::CanReceiver<EventT>& receiver() const { return receiver_; }

    /**
     * @brief Invoke @p callback_ when set.
     * @param sample Owning sample transferred to the sink.
     */
    void EmitSample(std::unique_ptr<SensorSample> sample) {
        if (callback_ && sample) {
            callback_(std::move(sample));
        }
    }

    // Sensor identifier for this driver instance.
    SensorId id_;

    // Parsed driver parameters from configuration.
    DriverParams params_;

    // User callback for delivered samples.
    SampleCallback callback_;

private:
    // SocketCAN receive loop + MessageManager.
    canbus::CanReceiver<EventT> receiver_;

    // Poll timeout (ms) passed to CanReceiver::Start.
    int poll_timeout_ms_{100};
};

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_COMMON_CAN_SENSOR_DRIVER_BASE_HPP_
