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
 * @brief Orchestrates sensor plugins, hotplug, and sample routing.
 */

#ifndef AUTODRIVER_SENSOR_MANAGER_HPP_
#define AUTODRIVER_SENSOR_MANAGER_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include <Eigen/Geometry>

#include "autodriver/config.hpp"
#include "autodriver/common/status.hpp"
#include "autodriver/lidar/motion_compensator.hpp"
#include "autodriver/sample_sink.hpp"
#include "autodriver/sensor_module.hpp"
#include "autodriver/sensor_hub.hpp"
#include "autolink/base/atomic_rw_lock.hpp"
#include "autolink/base/rw_lock_guard.hpp"
#include "autolink/common/macros.hpp"

namespace autolink {
namespace class_loader {
class ClassLoader;
}  // namespace class_loader
}  // namespace autolink

namespace autodriver {

/**
 * @class autodriver::SensorManager
 * @brief Loads one plugin instance per sensor id; routes samples to SensorHub
 *        and an optional SampleSink (Autolink publishing lives in bridge/).
 *
 * Lifecycle: construct → SetSampleSink (optional) → Initialize → Start →
 * AttachSensor / DetachSensor / HandleDeviceEvent → Stop.
 */
class SensorManager {
public:
  /**
   * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
   */
  AUTOLINK_SHARED_PTR_DEFINITIONS(SensorManager)

  /**
   * @brief Disable copy construction and copy assignment.
   */
  DISALLOW_COPY_AND_ASSIGN(SensorManager)

  /**
   * @brief Default-constructs a manager with an empty config.
   */
  SensorManager();

  /**
   * @brief Constructs a manager bound to the given sensor configuration.
   * @param config Process config (sensors, hotplug, alignment, compensator).
   */
  explicit SensorManager(Config config);

  /**
   * @brief Stops the manager and detaches all sensors.
   */
  ~SensorManager();

  /**
   * @brief Registers the downstream consumer for raw or aligned samples.
   * @param sink Non-owning pointer; may be nullptr to clear. Call before Start.
   */
  void SetSampleSink(SampleSink* sink);

  /**
   * @brief Validates config (e.g. duplicate ids) and marks the manager ready.
   * @return false when config is invalid; Start must not be called then.
   */
  bool Initialize();

  /**
   * @brief Attaches autostart sensors, starts the hub, and udev hotplug.
   * @return false if not initialized or attach of an autostart sensor fails.
   */
  bool Start();

  /**
   * @brief Detaches all sensors, stops the hub, and shuts down udev.
   */
  void Stop();

  /**
   * @brief Loads and starts the sensor module for @p id if not already attached.
   * @param id Configured sensor identifier (must exist in config.sensors).
   * @return false if unknown id, already attached, or plugin/driver init fails.
   */
  bool AttachSensor(const SensorId& id);

  /**
   * @brief Stops and unloads the sensor module for @p id (no-op if not attached).
   * @param id Sensor identifier to detach.
   */
  void DetachSensor(const SensorId& id);

  /**
   * @brief Attaches or detaches a sensor in response to a hotplug event.
   * @param added true on device ADD, false on REMOVE.
   * @param device Observed udev identity matched against Config::Sensor::match.
   */
  void HandleDeviceEvent(bool added, const DeviceMatch& device);

  /**
   * @brief Whether Start has been called and Stop has not.
   * @return true while the manager is running.
   */
  bool IsRunning() const;

  /**
   * @brief Number of currently attached sensor modules.
   * @return Count of entries in the internal modules_ map.
   */
  std::size_t AttachedCount() const;

  /**
   * @brief Mutable access to the central sample hub.
   * @return Reference to the owned SensorHub.
   */
  SensorHub& GetHub() { return hub_; }

  /**
   * @brief Const access to the central sample hub.
   * @return Const reference to the owned SensorHub.
   */
  const SensorHub& GetHub() const { return hub_; }

  /**
   * @brief Registers a user callback for aligned snapshots (composed with
   *        config.alignment.publish_aligned sink publishing).
   * @param callback Invoked when a multi-sensor AlignedSnapshot is ready.
   */
  void SetAlignedCallback(SensorHub::AlignedCallback callback);

  /**
   * @brief Forwards per-sample callbacks to the internal hub.
   * @param callback Invoked for each raw sample after time sync.
   */
  void SetRawSampleCallback(SensorHub::RawSampleCallback callback);

  /**
   * @brief Publishes a diagnostic snapshot to the registered sink.
   * @param snapshot Device health / status payload.
   */
  void ReportDiagnostic(diagnostics::DiagnosticSnapshot snapshot);

  /**
   * @brief Feed a world←lidar pose into an attached lidar MotionPoseSink.
   * @param id Attached 3D lidar sensor id.
   * @param time_ns Pose timestamp in nanoseconds.
   * @param pose World ← lidar transform.
   * @return false when id is not attached or driver is not a MotionPoseSink
   *         (e.g. compensator disabled / stub backend).
   */
  bool PushLidarPose(const SensorId& id, std::uint64_t time_ns,
                     const Eigen::Affine3d& pose);

  /**
   * @brief Override PoseLookup for an attached lidar MotionPoseSink.
   * @param id Attached 3D lidar sensor id.
   * @param lookup Callable used by the motion compensator.
   * @return false when id is not attached or driver is not a MotionPoseSink.
   */
  bool SetLidarPoseLookup(const SensorId& id, lidar::PoseLookup lookup);

private:
  /**
   * @brief Looks up a sensor entry by id in the active config.
   * @param id Sensor identifier.
   * @return Pointer into config_.sensors, or nullptr if not found.
   */
  const Config::Sensor* FindSensorConfig(const SensorId& id) const;

  /**
   * @brief Resolves the native shared-library path for a sensor plugin.
   * @param sensor Sensor entry (library basename or absolute path).
   * @return Absolute or search-ready library path string.
   */
  std::string ResolveLibraryPath(const Config::Sensor& sensor) const;

  /**
   * @brief Unloads a plugin library when no attached sensor still references it.
   * @param path Library path key in loaders_.
   */
  void UnloadIfUnused(const std::string& path);

  /**
   * @brief Loads, initializes, and starts a sensor module; caller holds lock.
   * @param id Sensor identifier.
   * @return false on config/plugin/driver failure.
   */
  bool AttachSensorLocked(const SensorId& id);

  /**
   * @brief Stops and removes a sensor module; caller holds lock.
   * @param id Sensor identifier.
   */
  void DetachSensorLocked(const SensorId& id);

  /**
   * @brief Spawns the udev monitor thread when hotplug is enabled.
   */
  void StartUdev();

  /**
   * @brief Joins the udev monitor thread if it was started.
   */
  void StopUdev();

  /**
   * @brief Polls udev for device add/remove events and dispatches matches.
   */
  void RunUdevMonitorLoop();

  /**
   * @brief Routes a sample through alignment and/or the registered sink.
   * @param sample Shared sample produced by a sensor module.
   */
  void DispatchSensorSample(std::shared_ptr<SensorSample> sample);

  /**
   * @brief Installs Hub aligned callback: optional sink publish + user hook.
   */
  void WireAlignedPublishing();

  // Process configuration loaded at construction.
  Config config_;

  // Central router for buffering and time alignment.
  SensorHub hub_;

  // Optional downstream sink; not owned.
  SampleSink* sink_{nullptr};

  // Optional user aligned-snapshot callback (composed with publish_aligned).
  SensorHub::AlignedCallback user_aligned_callback_;

  // Loaded sensor module instances keyed by sensor id.
  std::unordered_map<SensorId, SensorModule::SharedPtr> modules_;

  // Shared class loaders keyed by plugin library path.
  std::unordered_map<std::string,
                     std::unique_ptr<autolink::class_loader::ClassLoader>>
      loaders_;

  // Protects modules_, loaders_, and attach/detach state.
  mutable autolink::base::AtomicRWLock lock_;

  // True after Initialize() completes successfully.
  bool initialized_ = false;

  // True while the manager is started.
  std::atomic<bool> running_{false};

  // Background thread that polls udev when hotplug is enabled.
  std::thread udev_thread_;
};

}  // namespace autodriver

#endif  // AUTODRIVER_SENSOR_MANAGER_HPP_
