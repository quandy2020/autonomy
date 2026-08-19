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

#ifndef AUTODRIVER_SENSOR_MANAGER_HPP_
#define AUTODRIVER_SENSOR_MANAGER_HPP_

#include <atomic>
#include <cstddef>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include "autodriver/config.hpp"
#include "autodriver/sensor_module.hpp"
#include "autodriver/sensor_hub.hpp"
#include "autolink/base/atomic_rw_lock.hpp"
#include "autolink/base/rw_lock_guard.hpp"
#include "autolink/node/node.hpp"

namespace autolink {
namespace class_loader {
class ClassLoader;
}  // namespace class_loader
}  // namespace autolink

namespace autodriver {

// One Node; one plugin instance per id (N of each modality).
// udev add/remove maps onto Attach/Detach. Alignment is optional.
class SensorManager {
public:
    SensorManager();
    explicit SensorManager(Config config);
    ~SensorManager();

    SensorManager(const SensorManager&) = delete;
    SensorManager& operator=(const SensorManager&) = delete;

    bool Initialize();
    bool Start();
    void Stop();

    // Idempotent. Unknown ids and load failures return false; the process
    // keeps running.
    bool Attach(const SensorId& id);
    void Detach(const SensorId& id);

    void HandleDeviceEvent(bool added, const DeviceMatch& device);

    bool IsRunning() const;
    std::size_t AttachedCount() const;
    SensorHub& hub() { return hub_; }
    const SensorHub& hub() const { return hub_; }
    autolink::Node* node() { return node_.get(); }

    void SetAlignedCallback(SensorHub::AlignedCallback callback);
    void SetRawSampleCallback(SensorHub::RawSampleCallback callback);

private:
    const Config::Sensor* FindSensor(const SensorId& id) const;
    std::string LibraryPath(const Config::Sensor& sensor) const;
    void UnloadIfUnused(const std::string& path);
    bool AttachLocked(const SensorId& id);
    void DetachLocked(const SensorId& id);
    void StartUdev();
    void StopUdev();
    void UdevLoop();

    Config config_;
    SensorHub hub_;
    std::shared_ptr<autolink::Node> node_;
    std::unordered_map<SensorId, std::shared_ptr<SensorModule>> modules_;
    std::unordered_map<std::string,
                       std::unique_ptr<autolink::class_loader::ClassLoader>>
        loaders_;
    mutable autolink::base::AtomicRWLock lock_;
    bool initialized_ = false;
    std::atomic<bool> running_{false};
    std::thread udev_thread_;
};

}  // namespace autodriver

#endif  // AUTODRIVER_SENSOR_MANAGER_HPP_
