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
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include "autodriver/config.hpp"
#include "autodriver/sample_sink.hpp"
#include "autodriver/sensor_module.hpp"
#include "autodriver/sensor_hub.hpp"
#include "autolink/base/atomic_rw_lock.hpp"
#include "autolink/base/rw_lock_guard.hpp"

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
 */
class SensorManager {
public:
    /** @brief Default-construct with an empty configuration. */
    SensorManager();
    /**
     * @brief Construct with a pre-loaded configuration.
     * @param config Process configuration snapshot.
     */
    explicit SensorManager(Config config);
    /** @brief Stop all sensors and join background threads. */
    ~SensorManager();

    /** @brief Copy construction is disabled. */
    SensorManager(const SensorManager&) = delete;
    /** @brief Copy assignment is disabled. */
    SensorManager& operator=(const SensorManager&) = delete;

    /**
     * @brief Register the downstream sink before Start() so Attach can open writers.
     * @param sink Not owned; typically bridge::Publisher.
     */
    void SetSink(SampleSink* sink);

    /**
     * @brief Load plugins and prepare the hub; does not start capture.
     * @return True when initialization succeeds.
     */
    bool Initialize();

    /**
     * @brief Attach autostart sensors and begin udev monitoring when enabled.
     * @return True when startup succeeds.
     */
    bool Start();

    /**
     * @brief Detach all sensors and stop background threads.
     */
    void Stop();

    /**
     * @brief Load and start a sensor by id.
     * @param id Configured sensor identifier to attach.
     * @return False for unknown ids or plugin load failures; the process keeps running.
     */
    bool Attach(const SensorId& id);

    /**
     * @brief Stop and unload a sensor by id.
     * @param id Configured sensor identifier to detach.
     */
    void Detach(const SensorId& id);

    /**
     * @brief Handle a udev add/remove event.
     * @param added True for device arrival, false for removal.
     * @param device Observed device identity from udev.
     */
    void HandleDeviceEvent(bool added, const DeviceMatch& device);

    /**
     * @brief Whether the manager has started and is running.
     * @return True after a successful Start() and before Stop().
     */
    bool IsRunning() const;

    /**
     * @brief Number of currently attached sensor modules.
     * @return Count of loaded and started plugins.
     */
    std::size_t AttachedCount() const;

    /**
     * @brief Mutable access to the central sample hub.
     * @return Reference to the owned SensorHub.
     */
    SensorHub& hub() { return hub_; }

    /**
     * @brief Const access to the central sample hub.
     * @return Const reference to the owned SensorHub.
     */
    const SensorHub& hub() const { return hub_; }

    /**
     * @brief Callback fired when optional multi-sensor alignment produces a snapshot.
     * @param callback Invoked for each aligned snapshot from the hub.
     */
    void SetAlignedCallback(SensorHub::AlignedCallback callback);

    /**
     * @brief Callback fired for every raw sample before alignment.
     * @param callback Invoked for each sample routed through the hub.
     */
    void SetRawSampleCallback(SensorHub::RawSampleCallback callback);

private:
    /**
     * @brief Look up a sensor configuration entry by id.
     * @param id Sensor identifier to find.
     * @return Pointer to the sensor entry, or nullptr when not configured.
     */
    const Config::Sensor* FindSensor(const SensorId& id) const;

    /**
     * @brief Resolve the shared-library path for a sensor plugin.
     * @param sensor Sensor configuration entry.
     * @return Absolute or relative path to the plugin library.
     */
    std::string LibraryPath(const Config::Sensor& sensor) const;

    /**
     * @brief Unload a class loader when no modules reference its library.
     * @param path Plugin library path used as the loader key.
     */
    void UnloadIfUnused(const std::string& path);

    /**
     * @brief Attach a sensor while holding the manager lock.
     * @param id Sensor identifier to attach.
     * @return True when the sensor is loaded and started.
     */
    bool AttachLocked(const SensorId& id);

    /**
     * @brief Detach a sensor while holding the manager lock.
     * @param id Sensor identifier to detach.
     */
    void DetachLocked(const SensorId& id);

    /** @brief Start the udev hotplug monitoring thread. */
    void StartUdev();

    /** @brief Stop the udev hotplug monitoring thread. */
    void StopUdev();

    /** @brief Main loop for processing udev device events. */
    void UdevLoop();

    /**
     * @brief Route a sample to the hub and optional sink.
     * @param sample Shared sample produced by a sensor module.
     */
    void DispatchSample(std::shared_ptr<SensorSample> sample);

    /** @brief Process configuration loaded at construction. */
    Config config_;
    /** @brief Central router for buffering and time alignment. */
    SensorHub hub_;
    /** @brief Optional downstream sink; not owned. */
    SampleSink* sink_ = nullptr;
    /** @brief Loaded sensor module instances keyed by sensor id. */
    std::unordered_map<SensorId, std::shared_ptr<SensorModule>> modules_;
    /** @brief Shared class loaders keyed by plugin library path. */
    std::unordered_map<std::string,
                       std::unique_ptr<autolink::class_loader::ClassLoader>>
        loaders_;
    /** @brief Protects modules_, loaders_, and attach/detach state. */
    mutable autolink::base::AtomicRWLock lock_;
    /** @brief True after Initialize() completes successfully. */
    bool initialized_ = false;
    /** @brief True while the manager is started. */
    std::atomic<bool> running_{false};
    /** @brief Background thread that polls udev when hotplug is enabled. */
    std::thread udev_thread_;
};

}  // namespace autodriver

#endif  // AUTODRIVER_SENSOR_MANAGER_HPP_
