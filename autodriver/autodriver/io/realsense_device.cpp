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
 * @brief Implements RealSenseDeviceHub.
 */

#include "autodriver/io/realsense_device.hpp"

#ifdef AUTODRIVER_HAVE_REALSENSE
#include <librealsense2/rs.hpp>
#endif

#include <algorithm>
#include <atomic>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "autodriver/drivers/hardware/driver_params.hpp"

namespace autodriver {
namespace io {
namespace {

std::mutex g_pool_mutex;
std::unordered_map<std::string, std::weak_ptr<RealSenseDeviceHub>> g_pool;

std::string DeviceKey(const hardware::DriverParams & params)
{
  const std::string serial = hardware::GetString(params, "serial");
  if (!serial.empty()) {
    return serial;
  }
  const int index = hardware::ParseInt(params, "index", 0);
  const std::string model = hardware::GetString(params, "model");
  return "index:" + std::to_string(index) + ":model:" + model;
}

}  // namespace

bool RealSenseAvailable()
{
#ifdef AUTODRIVER_HAVE_REALSENSE
  return true;
#else
  return false;
#endif
}

struct RealSenseDeviceHub::Impl
{
  hardware::DriverParams params;
  std::string device_key;
  std::string last_error;
  std::atomic<bool> running{false};
  std::atomic<std::uint64_t> next_subscription_id{1};
  std::mutex mutex;

  struct VideoSubscription
  {
    std::uint64_t id{0};
    hardware::realsense::StreamKind stream{
      hardware::realsense::StreamKind::kColor};
    int width{640};
    int height{480};
    int fps{30};
    RealSenseVideoCallback callback;
  };

  struct ImuSubscription
  {
    std::uint64_t id{0};
    RealSenseImuCallback callback;
  };

  std::vector<VideoSubscription> video_subscriptions;
  std::vector<ImuSubscription> imu_subscriptions;
  std::thread worker;

#ifndef AUTODRIVER_HAVE_REALSENSE
  bool StartPipeline()
  {
    last_error =
      "librealsense2 not found at build time; install librealsense2-dev "
      "and rebuild with AUTODRIVER_WITH_REALSENSE=ON";
    return false;
  }

  void StopPipeline() {}

  void CaptureLoop() {}
#else
  rs2::context context;
  rs2::pipeline pipeline;
  rs2::config config;
  std::string bound_serial;

  rs2::stream_index_pair StreamPair(const hardware::realsense::StreamKind kind) const
  {
    switch (kind) {
      case hardware::realsense::StreamKind::kColor:
        return RS2_STREAM_COLOR;
      case hardware::realsense::StreamKind::kDepth:
        return RS2_STREAM_DEPTH;
      case hardware::realsense::StreamKind::kInfrared1:
        return RS2_STREAM_INFRARED;
      case hardware::realsense::StreamKind::kInfrared2:
        return {RS2_STREAM_INFRARED, 2};
    }
    return RS2_STREAM_COLOR;
  }

  rs2_format StreamFormat(const hardware::realsense::StreamKind kind) const
  {
    switch (kind) {
      case hardware::realsense::StreamKind::kColor:
        return RS2_FORMAT_RGB8;
      case hardware::realsense::StreamKind::kDepth:
        return RS2_FORMAT_Z16;
      case hardware::realsense::StreamKind::kInfrared1:
      case hardware::realsense::StreamKind::kInfrared2:
        return RS2_FORMAT_Y8;
    }
    return RS2_FORMAT_RGB8;
  }

  rs2::device ResolveDevice()
  {
    const std::string serial = hardware::GetString(params, "serial");
    const std::string model = hardware::GetString(params, "model");
    const int index = hardware::ParseInt(params, "index", 0);

    const rs2::device_list devices = context.query_devices();
    if (devices.size() == 0) {
      last_error = "No Intel RealSense device connected";
      return rs2::device();
    }

    if (!serial.empty()) {
      for (rs2::device device : devices) {
        if (device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) == serial) {
          return device;
        }
      }
      last_error = "RealSense serial not found: " + serial;
      return rs2::device();
    }

    std::vector<rs2::device> candidates;
    for (rs2::device device : devices) {
      const std::string name = device.get_info(RS2_CAMERA_INFO_NAME);
      if (hardware::realsense::MatchesModelFilter(name, model)) {
        candidates.push_back(device);
      }
    }

    if (candidates.empty()) {
      last_error = model.empty() ?
        "No RealSense device matched selection" :
        "No RealSense device matched model filter: " + model;
      return rs2::device();
    }

    if (index < 0 || static_cast<std::size_t>(index) >= candidates.size()) {
      last_error =
        "RealSense device index out of range: " + std::to_string(index);
      return rs2::device();
    }

    return candidates[static_cast<std::size_t>(index)];
  }

  bool ConfigurePipeline()
  {
    config = rs2::config();
    const rs2::device device = ResolveDevice();
    if (!device) {
      return false;
    }

    bound_serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    config.enable_device(bound_serial);

    for (const VideoSubscription & sub : video_subscriptions) {
      const rs2::stream_index_pair stream = StreamPair(sub.stream);
      config.enable_stream(
        stream, sub.width, sub.height, StreamFormat(sub.stream), sub.fps);
    }

    if (!imu_subscriptions.empty()) {
      config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
      config.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
    }

    return true;
  }

  bool StartPipeline()
  {
    StopPipeline();
    if (!ConfigurePipeline()) {
      return false;
    }

    try {
      pipeline.start(config);
      last_error.clear();
      return true;
    } catch (const rs2::error & ex) {
      last_error = std::string("RealSense pipeline start failed: ") + ex.what();
      return false;
    }
  }

  void StopPipeline()
  {
    try {
      pipeline.stop();
    } catch (...) {
    }
  }

  void DispatchVideoFrame(
    const VideoSubscription & sub,
    const rs2::video_frame & frame)
  {
    if (!sub.callback || !frame) {
      return;
    }

    RealSenseVideoFrame payload;
    payload.width = static_cast<std::uint32_t>(frame.get_width());
    payload.height = static_cast<std::uint32_t>(frame.get_height());
    payload.encoding =
      hardware::realsense::EncodingForStreamKind(sub.stream);
    const auto * bytes = static_cast<const std::uint8_t *>(frame.get_data());
    const std::size_t size = frame.get_data_size();
    payload.data.assign(bytes, bytes + size);
    sub.callback(std::move(payload));
  }

  void CaptureLoop()
  {
    std::array<double, 3> latest_accel{{0.0, 0.0, 0.0}};
    std::array<double, 3> latest_gyro{{0.0, 0.0, 0.0}};
    bool have_accel = false;
    bool have_gyro = false;

    while (running.load()) {
      rs2::frameset frames;
      try {
        if (!pipeline.poll_for_frames(&frames)) {
          continue;
        }
      } catch (const rs2::error & ex) {
        last_error = std::string("RealSense poll failed: ") + ex.what();
        break;
      }

      std::vector<VideoSubscription> video_subs;
      std::vector<ImuSubscription> imu_subs;
      {
        std::lock_guard<std::mutex> lock(mutex);
        video_subs = video_subscriptions;
        imu_subs = imu_subscriptions;
      }

      for (const VideoSubscription & sub : video_subs) {
        const rs2::stream_index_pair stream = StreamPair(sub.stream);
        const rs2::video_frame frame = frames.first_or_default(stream);
        DispatchVideoFrame(sub, frame);
      }

      if (!imu_subs.empty()) {
        const rs2::motion_frame accel = frames.first_or_default(RS2_STREAM_ACCEL);
        const rs2::motion_frame gyro = frames.first_or_default(RS2_STREAM_GYRO);
        if (accel) {
          const rs2_vector v = accel.get_motion_data();
          latest_accel = {v.x, v.y, v.z};
          have_accel = true;
        }
        if (gyro) {
          const rs2_vector v = gyro.get_motion_data();
          latest_gyro = {v.x, v.y, v.z};
          have_gyro = true;
        }
        if (have_accel && have_gyro) {
          for (const ImuSubscription & sub : imu_subs) {
            if (sub.callback) {
              sub.callback(latest_accel, latest_gyro);
            }
          }
        }
      }
    }
  }
#endif
};

RealSenseDeviceHub::RealSenseDeviceHub(const hardware::DriverParams & params)
: impl_(std::make_unique<Impl>())
{
  impl_->params = params;
  impl_->device_key = DeviceKey(params);
}

RealSenseDeviceHub::~RealSenseDeviceHub()
{
  Stop();
}

std::shared_ptr<RealSenseDeviceHub> RealSenseDeviceHub::Acquire(
  const hardware::DriverParams & params)
{
  const std::string key = DeviceKey(params);
  std::lock_guard<std::mutex> lock(g_pool_mutex);
  if (const auto it = g_pool.find(key); it != g_pool.end()) {
    if (auto existing = it->second.lock()) {
      return existing;
    }
  }

  auto hub = std::shared_ptr<RealSenseDeviceHub>(
    new RealSenseDeviceHub(params));
  g_pool[key] = hub;
  return hub;
}

std::uint64_t RealSenseDeviceHub::SubscribeVideo(
  const hardware::realsense::StreamKind stream,
  const int width,
  const int height,
  const int fps,
  RealSenseVideoCallback callback)
{
  const bool restart = impl_->running.load();
  if (restart) {
    Stop();
  }

  std::uint64_t id = 0;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    id = impl_->next_subscription_id.fetch_add(1);
    impl_->video_subscriptions.push_back(
      Impl::VideoSubscription{
        id, stream, width, height, fps, std::move(callback)});
  }

  if (restart) {
    Start();
  }
  return id;
}

std::uint64_t RealSenseDeviceHub::SubscribeImu(RealSenseImuCallback callback)
{
  const bool restart = impl_->running.load();
  if (restart) {
    Stop();
  }

  std::uint64_t id = 0;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    id = impl_->next_subscription_id.fetch_add(1);
    impl_->imu_subscriptions.push_back(
      Impl::ImuSubscription{id, std::move(callback)});
  }

  if (restart) {
    Start();
  }
  return id;
}

void RealSenseDeviceHub::Unsubscribe(const std::uint64_t subscription_id)
{
  const bool was_running = impl_->running.load();
  if (was_running) {
    Stop();
  }

  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto & video = impl_->video_subscriptions;
    video.erase(
      std::remove_if(
        video.begin(), video.end(),
        [subscription_id](const Impl::VideoSubscription & sub) {
          return sub.id == subscription_id;
        }),
      video.end());
    auto & imu = impl_->imu_subscriptions;
    imu.erase(
      std::remove_if(
        imu.begin(), imu.end(),
        [subscription_id](const Impl::ImuSubscription & sub) {
          return sub.id == subscription_id;
        }),
      imu.end());
  }

  if (was_running &&
    (!impl_->video_subscriptions.empty() || !impl_->imu_subscriptions.empty()))
  {
    Start();
  }
}

bool RealSenseDeviceHub::Start()
{
  if (impl_->running.exchange(true)) {
    return true;
  }

  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->video_subscriptions.empty() && impl_->imu_subscriptions.empty()) {
      impl_->last_error = "RealSense hub has no active subscriptions";
      impl_->running = false;
      return false;
    }
  }

  if (!impl_->StartPipeline()) {
    impl_->running = false;
    return false;
  }

  impl_->worker = std::thread([this]() { impl_->CaptureLoop(); });
  return true;
}

void RealSenseDeviceHub::Stop()
{
  if (!impl_->running.exchange(false)) {
    return;
  }

  impl_->StopPipeline();
  if (impl_->worker.joinable()) {
    impl_->worker.join();
  }
}

bool RealSenseDeviceHub::IsRunning() const
{
  return impl_->running.load();
}

const std::string & RealSenseDeviceHub::last_error() const
{
  return impl_->last_error;
}

}  // namespace io
}  // namespace autodriver
