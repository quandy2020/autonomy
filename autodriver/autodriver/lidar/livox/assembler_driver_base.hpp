/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file assembler_driver_base.hpp
 * @brief CRTP base for Livox SDK drivers: FrameAssembler → PointCloud2 publish.
 *
 * Derived must implement InitSdk() / UninitSdk() and friend this base (or make
 * those methods public). SDK packet decoding stays in Derived callbacks.
 */

#ifndef AUTODRIVER_LIDAR_LIVOX_ASSEMBLER_DRIVER_BASE_HPP_
#define AUTODRIVER_LIDAR_LIVOX_ASSEMBLER_DRIVER_BASE_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "autodriver/driver_params.hpp"
#include "autodriver/lidar/lidar_component_base.hpp"
#include "autodriver/lidar/livox/common.hpp"
#include "autodriver/lidar/livox/convert.hpp"
#include "autodriver/lidar/livox/points.hpp"
#include "autodriver/sensor_driver.hpp"
#include "autodriver/types/sensor_sample.hpp"
#include "autolink/common/log.hpp"
#include "autolink/time/time.hpp"
#include "autolink/common/macros.hpp"

namespace autodriver {
namespace lidar {
namespace livox {

/**
 * @class autodriver::lidar::livox::AssemblerDriverBase
 * @brief Shared Start/Stop / PublishLoop for Livox SDK1 and SDK2.
 *
 * @tparam Derived Concrete driver (CRTP); must provide InitSdk / UninitSdk.
 * @tparam Traits `kLogTag`, `kDefaultModel` string constants.
 */
template <typename Derived, typename Traits>
class AssemblerDriverBase : public SensorDriver, public LidarComponentBase {
public:
    /**
     * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
     */
    AUTOLINK_SHARED_PTR_DEFINITIONS(AssemblerDriverBase)

    /**
     * @brief Disable copy construction and copy assignment.
     */
    DISALLOW_COPY_AND_ASSIGN(AssemblerDriverBase)

    /**
     * @brief Parse common YAML params and InitBase.
     * @param[in] id Sensor instance id.
     * @param[in] params DriverParams (cold path).
     */
    AssemblerDriverBase(SensorId id, hardware::DriverParams params)
        : id_(std::move(id)),
          params_(std::move(params)),
          assembler_(IntervalFromHz(10.0)) {
        using hardware::GetString;
        using hardware::ParseDouble;

        model_ = GetString(params_, "model", Traits::kDefaultModel);
        frame_id_ = GetString(params_, "frame_id", id_);
        publish_freq_hz_ = ResolvePublishFreqHz(
            ParseDouble(params_, "publish_freq", 0.0),
            ParseDouble(params_, "fps", 10.0));
        assembler_.SetIntervalNs(IntervalFromHz(publish_freq_hz_));

        LidarBaseOptions options;
        options.source =
            ParseSourceType(GetString(params_, "source_type", "online"));
        options.cloud_channel = GetString(params_, "channel", "");
        options.publish_scan = false;
        InitBase(options);
    }

    /**
     * @brief Joins the publish thread if still running.
     *
     * Derived destructors must call Stop() first so UninitSdk runs while the
     * derived object is still alive.
     */
    ~AssemblerDriverBase() override {
        if (!running_.exchange(false)) {
            return;
        }
        if (publisher_.joinable()) {
            publisher_.join();
        }
        assembler_.Clear();
    }

    /**
     * @brief Report sensor modality.
     * @return SensorType::kLidar3d.
     */
    SensorType GetSensorType() const override { return SensorType::kLidar3d; }

    /**
     * @brief Configured sensor instance id.
     * @return Reference to the id passed at construction.
     */
    const SensorId& GetSensorId() const override { return id_; }

    /**
     * @brief Init SDK via Derived and start the publish loop.
     * @return true on success or already running.
     */
    bool Start() override {
        bool expected = false;
        if (!running_.compare_exchange_strong(expected, true)) {
            return true;
        }
        if (!self().InitSdk()) {
            running_ = false;
            return false;
        }
        publisher_ = std::thread([this] { PublishLoop(); });
        AINFO << Traits::kLogTag << " driver started id=" << id_
              << " model=" << model_;
        return true;
    }

    /**
     * @brief Stop publish loop and UninitSdk via Derived.
     */
    void Stop() override {
        if (!running_.exchange(false)) {
            return;
        }
        if (publisher_.joinable()) {
            publisher_.join();
        }
        assembler_.Clear();
        self().UninitSdk();
    }

    /**
     * @brief Whether the publish loop is active.
     * @return true while Start succeeded and Stop has not completed.
     */
    bool IsRunning() const override { return running_.load(); }

    /**
     * @brief Register the sample sink for PointCloud2 frames.
     * @param[in] callback Invoked with owning sample clones.
     */
    void SetSampleCallback(SampleCallback callback) override {
        callback_ = std::move(callback);
    }

    /**
     * @brief CRTP access to the concrete driver.
     * @return Reference to @p Derived.
     */
    Derived& self() { return static_cast<Derived&>(*this); }

    /**
     * @brief Const CRTP access to the concrete driver.
     * @return Const reference to @p Derived.
     */
    const Derived& self() const {
        return static_cast<const Derived&>(*this);
    }

protected:
    void WritePointCloud(std::shared_ptr<SensorSample> cloud) override {
        if (callback_ && cloud) {
            callback_(cloud->Clone());
        }
    }

    /**
     * @brief Append decoded points into the shared FrameAssembler.
     * @param[in] points Points to move into the current frame buffer.
     */
    void AppendPoints(std::vector<PointXYZIT> points) {
        if (!points.empty()) {
            assembler_.Append(std::move(points));
        }
    }

    FrameAssembler& assembler() { return assembler_; }
    const SensorId& id() const { return id_; }
    const std::string& frame_id() const { return frame_id_; }
    const std::string& model() const { return model_; }
    hardware::DriverParams& params() { return params_; }
    const hardware::DriverParams& params() const { return params_; }
    double publish_freq_hz() const { return publish_freq_hz_; }
    bool sdk_owned() const { return sdk_owned_; }
    void set_sdk_owned(bool owned) { sdk_owned_ = owned; }

private:
    void PublishLoop() {
        while (running_.load()) {
            std::vector<PointXYZIT> frame;
            const std::uint64_t now_ns = static_cast<std::uint64_t>(
                autolink::Time::Now().ToNanosecond());
            if (assembler_.TryFlush(now_ns, &frame) && !frame.empty()) {
                PublishFrame(std::move(frame));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    void PublishFrame(std::vector<PointXYZIT> points) {
        auto cloud_msg = PointsToPointCloud(points, frame_id_);
        auto sample = std::make_shared<LidarCloud>(
            id_, autolink::Time::Now(), std::move(cloud_msg));
        sample->frame_id = frame_id_;
        sample->channel = options().cloud_channel;
        WritePointCloud(sample);
    }

    SensorId id_;
    hardware::DriverParams params_;
    SampleCallback callback_;
    std::atomic<bool> running_{false};
    std::thread publisher_;
    FrameAssembler assembler_;
    std::string model_{Traits::kDefaultModel};
    std::string frame_id_;
    double publish_freq_hz_ = 10.0;
    bool sdk_owned_ = false;
};

}  // namespace livox
}  // namespace lidar
}  // namespace autodriver

#endif  // AUTODRIVER_LIDAR_LIVOX_ASSEMBLER_DRIVER_BASE_HPP_
