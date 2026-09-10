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
 * @brief CRTP base for serial (Stream) byte-reader sensor drivers.
 *
 * Derived must provide (public, called via CRTP):
 * - `SensorType GetSensorType() const`
 * - `void OnBytes(const std::uint8_t* data, std::size_t n)`
 *
 * Optional hooks (hide base defaults when needed):
 * - `bool PrepareStart()` — before opening the stream; return false to abort
 * - `void OnStopped()` — after join / stream reset (e.g. release parsers)
 *
 * Derived destructor must call `Stop()` before destroying parser state that
 * `OnBytes` may touch (base destructor also calls `Stop()` as a fallback).
 */

#ifndef AUTODRIVER_COMMON_SERIAL_BYTE_DRIVER_BASE_HPP_
#define AUTODRIVER_COMMON_SERIAL_BYTE_DRIVER_BASE_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "autodriver/common/stream.hpp"
#include "autodriver/driver_params.hpp"
#include "autodriver/sensor_driver.hpp"

namespace autodriver {
namespace hardware {

/**
 * @class autodriver::hardware::SerialByteDriverBase
 * @brief Shared Open → Read loop → reconnect → Stop for serial IMU/GPS-style drivers.
 *
 * @tparam Derived Concrete driver (CRTP).
 *
 * YAML knobs used by the base: `device` (default `/dev/ttyUSB0`), `baud`
 * (default 115200). Read timeout is fixed at construction.
 */
template <typename Derived>
class SerialByteDriverBase : public SensorDriver {
public:
    /**
     * @brief Store identity / params and the Stream::Read timeout.
     * @param id Sensor instance id from YAML.
     * @param params Cold-path driver params (`device`, `baud`, …).
     * @param read_timeout_ms Timeout passed to `Stream::Read`; ≤0 → 50.
     */
    SerialByteDriverBase(SensorId id, DriverParams params, int read_timeout_ms)
        : id_(std::move(id)),
          params_(std::move(params)),
          read_timeout_ms_(read_timeout_ms > 0 ? read_timeout_ms : 50) {}

    /**
     * @brief Stops the reader thread if still running.
     */
    ~SerialByteDriverBase() override { Stop(); }

    /**
     * @brief Stable instance id from configuration.
     * @return Configured sensor identifier.
     */
    const SensorId& GetSensorId() const override { return id_; }

    /**
     * @brief Open serial Stream, call Derived::PrepareStart, start ReadLoop.
     * @return true when already running or the worker started successfully.
     */
    bool Start() override {
        if (running_.exchange(true)) {
            return true;
        }
        if (!self().PrepareStart()) {
            running_ = false;
            return false;
        }

        const std::string device = GetString(params_, "device", "/dev/ttyUSB0");
        const int baud = ParseInt(params_, "baud", 115200);
        stream_ = common::CreateSerialStream(device, baud);
        if (!stream_ || !stream_->Connect()) {
            stream_.reset();
            self().OnStopped();
            running_ = false;
            return false;
        }

        worker_ = std::thread([this]() { ReadLoop(); });
        return true;
    }

    /**
     * @brief Disconnect Stream, join the worker, then Derived::OnStopped.
     */
    void Stop() override {
        if (!running_.exchange(false)) {
            return;
        }
        if (stream_) {
            stream_->Disconnect();
        }
        if (worker_.joinable()) {
            worker_.join();
        }
        stream_.reset();
        self().OnStopped();
    }

    /**
     * @brief Whether the reader thread is active.
     * @return true while Start succeeded and Stop has not completed.
     */
    bool IsRunning() const override { return running_.load(); }

    /**
     * @brief Register the sample sink callback (driver thread).
     * @param callback May be empty to disable emission.
     */
    void SetSampleCallback(SampleCallback callback) override {
        callback_ = std::move(callback);
    }

    /**
     * @brief Default PrepareStart hook (no-op success).
     * @return Always true; Derived may hide to create parsers / validate params.
     */
    bool PrepareStart() { return true; }

    /**
     * @brief Default OnStopped hook (no-op).
     *
     * Derived may hide to release parsers after the worker has joined.
     */
    void OnStopped() {}

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

    // User callback for delivered samples (driver thread).
    SampleCallback callback_;

    // Serial (or future TCP/UDP) transport.
    std::unique_ptr<common::Stream> stream_{nullptr};

    // True while Start() succeeded and Stop() has not completed.
    std::atomic<bool> running_{false};

private:
    /**
     * @brief Worker: Read → reconnect on error → Derived::OnBytes.
     */
    void ReadLoop() {
        std::uint8_t chunk[256];
        while (running_.load()) {
            if (!stream_) {
                break;
            }
            if (stream_->status() == common::Stream::Status::kError) {
                if (!common::ReconnectStream(stream_.get(), 3, 200)) {
                    break;
                }
            }
            const std::size_t n =
                stream_->Read(chunk, sizeof(chunk), read_timeout_ms_);
            if (n == 0) {
                continue;
            }
            self().OnBytes(chunk, n);
        }
    }

    // Timeout (ms) passed to Stream::Read.
    int read_timeout_ms_{50};

    // Background byte-reader thread.
    std::thread worker_;
};

}  // namespace hardware
}  // namespace autodriver

#endif  // AUTODRIVER_COMMON_SERIAL_BYTE_DRIVER_BASE_HPP_
