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
 * and an optional SampleSink (Autolink publishing lives in bridge/).
 */
class SensorManager {
public:
    /**
     * @brief Default-constructs a manager with an empty config.
     */
    /**
     * @brief Constructs a manager bound to the given sensor configuration.
     */
    SensorManager();
    explicit SensorManager(Config config);
    /**
     * @brief Stops the manager and detaches all sensors.
     */
    ~SensorManager();

    SensorManager(const SensorManager&) = delete;
    /**
     * @brief Copy assignment is disabled.
     */
    SensorManager& operator=(const SensorManager&) = delete;

    /**
     * @brief Registers the downstream consumer for raw or aligned samples.
     */
    void SetSink(SampleSink* sink);

    /**
     * @brief Validates config and marks the manager as initialized.
     */
    bool Initialize();

    /**
     * @brief Attaches autostart sensors, starts alignment, and udev hotplug.
     */
    bool Start();

    /**
     * @brief Detaches all sensors, stops the hub, and shuts down udev.
     */
    void Stop();

    /**
     * @brief Loads and starts the sensor module for id if not already attached.
     */
    bool Attach(const SensorId& id);

    /**
     * @brief Stops and unloads the sensor module for id.
     */
    void Detach(const SensorId& id);

    /**
     * @brief Attaches or detaches a sensor in response to a hotplug event.
     */
    void HandleDeviceEvent(bool added, const DeviceMatch& device);

    /**
     * @brief Returns true while Start has been called and Stop has not.
     */
    bool IsRunning() const;

    /**
     * @brief Returns the number of currently attached sensor modules.
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
     * @brief Forwards aligned snapshot callbacks to the internal hub.
     */
    void SetAlignedCallback(SensorHub::AlignedCallback callback);

    /**
     * @brief Forwards per-sample callbacks to the internal hub.
     */
    void SetRawSampleCallback(SensorHub::RawSampleCallback callback);

private:
    /**
     * @brief Looks up a sensor entry by id in the active config.
     */
    const Config::Sensor* FindSensor(const SensorId& id) const;

    /**
     * @brief Resolves the native shared-library path for a sensor plugin.
     */
    std::string LibraryPath(const Config::Sensor& sensor) const;

    /**
     * @brief Unloads a plugin library when no attached sensor still references it.
     */
    void UnloadIfUnused(const std::string& path);

    /**
     * @brief Loads, initializes, and starts a sensor module; caller holds lock.
     */
    bool AttachLocked(const SensorId& id);

    /**
     * @brief Stops and removes a sensor module; caller holds lock.
     */
    void DetachLocked(const SensorId& id);

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
    void UdevLoop();

    /**
     * @brief Routes a sample through alignment and/or the registered sink.
     */
    void DispatchSample(std::shared_ptr<SensorSample> sample);

    // Process configuration loaded at construction.
    Config config_;

    // Central router for buffering and time alignment.
    SensorHub hub_;

    // Optional downstream sink; not owned.
    SampleSink* sink_ = nullptr;

    // Loaded sensor module instances keyed by sensor id.
    std::unordered_map<SensorId, std::shared_ptr<SensorModule>> modules_;
    /**
     * @brief Shared class loaders keyed by plugin library path.
     */
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
