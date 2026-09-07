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

#include "autodriver/sensor_manager.hpp"

#include <string>
#include <utility>
#include <vector>

#include "autodriver/lidar/motion_pose_sink.hpp"
#include "autolink/class_loader/class_loader.hpp"
#include "autolink/class_loader/class_loader_manager.hpp"
#include "autolink/common/log.hpp"

#ifdef AUTODRIVER_HAVE_UDEV
#include <libudev.h>
#include <poll.h>
#endif

namespace autodriver {
namespace {

using AtomicRWLock = autolink::base::AtomicRWLock;
using ReadLock = autolink::base::ReadLockGuard<AtomicRWLock>;
using WriteLock = autolink::base::WriteLockGuard<AtomicRWLock>;

#ifndef AUTODRIVER_PLUGIN_DIR
#define AUTODRIVER_PLUGIN_DIR ""
#endif

#ifndef AUTODRIVER_SHARED_SUFFIX
#define AUTODRIVER_SHARED_SUFFIX ".so"
#endif

/**
 * @brief Normalizes plugin library names to the platform shared suffix.
 */
std::string NativeLibraryName(std::string name) {
    // Default shared-library suffix on Unix.
    constexpr char kSo[] = ".so";
    const std::string suffix = AUTODRIVER_SHARED_SUFFIX;
    if (suffix != kSo && name.size() >= 3 &&
        name.compare(name.size() - 3, 3, kSo) == 0) {
        name.replace(name.size() - 3, 3, suffix);
    }
    return name;
}

}  // namespace

SensorManager::SensorManager() : SensorManager(Config{}) {}

SensorManager::SensorManager(Config config)
    : config_(std::move(config)), hub_(config_.alignment.options) {}

SensorManager::~SensorManager() { Stop(); }

bool SensorManager::Initialize() {
    WriteLock lock(lock_);
    if (initialized_) {
        return true;
    }
    if (config_.HasDuplicateId()) {
        AERROR << "duplicate id in Config";
        return false;
    }
    initialized_ = true;
    return true;
}

void SensorManager::SetSink(SampleSink* sink) {
    WriteLock lock(lock_);
    sink_ = sink;
}

bool SensorManager::Start() {
    if (!initialized_ && !Initialize()) {
        return false;
    }
    {
        WriteLock lock(lock_);
        running_ = true;
        if (config_.alignment.enable) {
            hub_.Start();
        }
        for (const Config::Sensor& sensor : config_.sensors) {
            if (sensor.autostart) {
                AttachLocked(sensor.id);
            }
        }
    }
    StartUdev();
    return true;
}

void SensorManager::Stop() {
    running_ = false;
    StopUdev();
    WriteLock lock(lock_);

    // Detached sensor ids collected before releasing modules.
    std::vector<SensorId> ids;
    ids.reserve(modules_.size());
    for (const auto& entry : modules_) {
        ids.push_back(entry.first);
    }
    for (const SensorId& id : ids) {
        DetachLocked(id);
    }
    hub_.Stop();
}

bool SensorManager::Attach(const SensorId& id) {
    if (!initialized_ && !Initialize()) {
        return false;
    }
    WriteLock lock(lock_);
    return AttachLocked(id);
}

void SensorManager::Detach(const SensorId& id) {
    WriteLock lock(lock_);
    DetachLocked(id);
}

void SensorManager::HandleDeviceEvent(bool added, const DeviceMatch& device) {
    const SensorId id = config_.FindId(device);
    if (id.empty()) {
        return;
    }
    if (added) {
        Attach(id);
    } else {
        Detach(id);
    }
}

bool SensorManager::IsRunning() const { return running_.load(); }

std::size_t SensorManager::AttachedCount() const {
    ReadLock lock(lock_);
    return modules_.size();
}

void SensorManager::SetAlignedCallback(SensorHub::AlignedCallback callback) {
    hub_.SetAlignedCallback(std::move(callback));
}

void SensorManager::SetRawSampleCallback(SensorHub::RawSampleCallback callback) {
    hub_.SetRawSampleCallback(std::move(callback));
}

void SensorManager::ReportDiagnostic(
    diagnostics::DiagnosticSnapshot snapshot) {
    if (sink_ != nullptr) {
        sink_->OnDiagnostic(snapshot);
    }
}

bool SensorManager::PushLidarPose(const SensorId& id, std::uint64_t time_ns,
                                  const Eigen::Affine3d& pose) {
    ReadLock lock(lock_);
    const auto it = modules_.find(id);
    if (it == modules_.end() || !it->second) {
        return false;
    }
    auto driver = it->second->GetDriver();
    auto* sink = dynamic_cast<lidar::MotionPoseSink*>(driver.get());
    if (sink == nullptr) {
        return false;
    }
    sink->PushPose(time_ns, pose);
    return true;
}

bool SensorManager::SetLidarPoseLookup(const SensorId& id,
                                       lidar::PoseLookup lookup) {
    ReadLock lock(lock_);
    const auto it = modules_.find(id);
    if (it == modules_.end() || !it->second) {
        return false;
    }
    auto driver = it->second->GetDriver();
    auto* sink = dynamic_cast<lidar::MotionPoseSink*>(driver.get());
    if (sink == nullptr) {
        return false;
    }
    sink->SetPoseLookup(std::move(lookup));
    return true;
}

void SensorManager::DispatchSample(std::shared_ptr<SensorSample> sample) {
    if (!sample) {
        return;
    }
    if (config_.alignment.enable) {
        hub_.PushSample(sample);
    }
    if (sink_ != nullptr) {
        sink_->OnSample(std::move(sample));
    }
}

const Config::Sensor* SensorManager::FindSensor(const SensorId& id) const {
    for (const Config::Sensor& sensor : config_.sensors) {
        if (sensor.id == id) {
            return &sensor;
        }
    }
    return nullptr;
}

std::string SensorManager::LibraryPath(const Config::Sensor& sensor) const {
    const std::string name = NativeLibraryName(sensor.library);
    const std::string dir =
        config_.plugins.empty() ? AUTODRIVER_PLUGIN_DIR : config_.plugins;
    if (dir.empty()) {
        return name;
    }
    return dir + "/" + name;
}

void SensorManager::UnloadIfUnused(const std::string& path) {
    if (path.empty()) {
        return;
    }
    for (const auto& entry : modules_) {
        const Config::Sensor* sensor = FindSensor(entry.first);
        if (sensor != nullptr && !sensor->library.empty() &&
            LibraryPath(*sensor) == path) {
            return;
        }
    }
    loaders_.erase(path);
}

bool SensorManager::AttachLocked(const SensorId& id) {
    if (modules_.count(id) != 0) {
        return true;
    }
    const Config::Sensor* sensor = FindSensor(id);
    if (sensor == nullptr) {
        AERROR << "unknown id: " << id;
        return false;
    }

    std::shared_ptr<SensorModule> instance;
    std::string path;
    if (sensor->library.empty()) {
        autolink::class_loader::ClassLoaderManager manager;
        instance = manager.CreateClassObj<SensorModule>(sensor->module);
    } else {
        path = LibraryPath(*sensor);
        auto& loader = loaders_[path];
        if (!loader) {
            loader = std::make_unique<autolink::class_loader::ClassLoader>(path);
        }
        instance = loader->CreateClassObj<SensorModule>(sensor->module);
    }
    if (!instance) {
        AERROR << "CreateClassObj failed: " << sensor->module;
        UnloadIfUnused(path);
        ReportDiagnostic({id, diagnostics::DeviceStatus::kError,
                          "CreateClassObj failed: " + sensor->module, 0});
        return false;
    }

    SensorModule::Context context;
    context.sensor = *sensor;
    context.hook = [this](std::shared_ptr<SensorSample> sample) {
        DispatchSample(std::move(sample));
    };
    if (!instance->Init(context)) {
        AERROR << "module Init failed: " << id;
        instance.reset();
        UnloadIfUnused(path);
        ReportDiagnostic(
            {id, diagnostics::DeviceStatus::kError, "Init failed", 0});
        return false;
    }
    if (sink_ != nullptr && !sink_->OnAttach(*sensor, instance->GetType())) {
        AERROR << "sink OnAttach failed: " << id;
        instance->Stop();
        instance.reset();
        UnloadIfUnused(path);
        ReportDiagnostic(
            {id, diagnostics::DeviceStatus::kError, "OnAttach failed", 0});
        return false;
    }
    if (!instance->Start()) {
        AERROR << "module Start failed: " << id;
        if (sink_ != nullptr) {
            sink_->OnDetach(id);
        }
        instance->Stop();
        instance.reset();
        UnloadIfUnused(path);
        ReportDiagnostic(
            {id, diagnostics::DeviceStatus::kError, "Start failed", 0});
        return false;
    }
    modules_[id] = std::move(instance);
    AINFO << "attached " << id;
    ReportDiagnostic({id, diagnostics::DeviceStatus::kOk, "attached", 0});
    return true;
}

void SensorManager::DetachLocked(const SensorId& id) {
    auto it = modules_.find(id);
    if (it == modules_.end()) {
        return;
    }
    const Config::Sensor* sensor = FindSensor(id);
    it->second->Stop();
    it->second.reset();
    modules_.erase(it);
    if (sink_ != nullptr) {
        sink_->OnDetach(id);
    }
    hub_.DropBuffer(id);
    if (sensor != nullptr && !sensor->library.empty()) {
        UnloadIfUnused(LibraryPath(*sensor));
    }
    AINFO << "detached " << id;
    ReportDiagnostic({id, diagnostics::DeviceStatus::kDisconnected,
                      "detached", 0});
}

void SensorManager::StartUdev() {
#ifdef AUTODRIVER_HAVE_UDEV
    if (!config_.hotplug.udev || udev_thread_.joinable()) {
        return;
    }
    udev_thread_ = std::thread([this] { UdevLoop(); });
#endif
}

void SensorManager::StopUdev() {
#ifdef AUTODRIVER_HAVE_UDEV
    if (udev_thread_.joinable()) {
        udev_thread_.join();
    }
#endif
}

void SensorManager::UdevLoop() {
#ifdef AUTODRIVER_HAVE_UDEV
    udev* udev = udev_new();
    if (udev == nullptr) {
        AERROR << "udev_new failed";
        return;
    }
    udev_monitor* monitor = udev_monitor_new_from_netlink(udev, "udev");
    if (monitor == nullptr) {
        udev_unref(udev);
        return;
    }
    udev_monitor_enable_receiving(monitor);
    const int fd = udev_monitor_get_fd(monitor);
    while (running_.load()) {
        pollfd poll_fd = {fd, POLLIN, 0};
        if (poll(&poll_fd, 1, 200) <= 0 || !(poll_fd.revents & POLLIN)) {
            continue;
        }
        udev_device* udev_dev = udev_monitor_receive_device(monitor);
        if (udev_dev == nullptr) {
            continue;
        }
        DeviceMatch device;
        if (const char* value = udev_device_get_subsystem(udev_dev)) {
            device.subsystem = value;
        }
        if (const char* value = udev_device_get_devnode(udev_dev)) {
            device.device = value;
        }
        if (const char* value =
                udev_device_get_sysattr_value(udev_dev, "idVendor")) {
            device.vendor = value;
        }
        if (const char* value =
                udev_device_get_sysattr_value(udev_dev, "idProduct")) {
            device.product = value;
        }
        if (const char* value =
                udev_device_get_sysattr_value(udev_dev, "serial")) {
            device.serial = value;
        }
        const char* action = udev_device_get_action(udev_dev);
        const std::string event = action != nullptr ? action : "";
        udev_device_unref(udev_dev);
        if (event == "add") {
            HandleDeviceEvent(true, device);
        } else if (event == "remove") {
            HandleDeviceEvent(false, device);
        }
    }
    udev_monitor_unref(monitor);
    udev_unref(udev);
#endif
}

}  // namespace autodriver
