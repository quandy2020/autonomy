/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autonomy/driver/sensor/camera/camera_manager.hpp"

#include <algorithm>

#include "autolink/common/log.hpp"

namespace autonomy {
namespace driver {
namespace sensor {
namespace camera {

CameraManager::CameraManager() {}

CameraManager::~CameraManager() {
    Cleanup();
}

bool CameraManager::LoadDriverPlugin(const std::string& library_path) {
    std::lock_guard<std::mutex> lock(mutex_);

    // 检查是否已经加载
    if (plugin_libraries_.find(library_path) != plugin_libraries_.end()) {
        AWARN << "Plugin library already loaded: " << library_path;
        return true;
    }

    // 创建 ClassLoader
    auto class_loader =
        std::make_shared<::autolink::class_loader::ClassLoader>(library_path);

    if (!class_loader->LoadLibrary()) {
        AERROR << "Failed to load plugin library: " << library_path;
        return false;
    }

    // 获取所有可用的驱动类名
    std::vector<std::string> driver_classes =
        class_loader->GetValidClassNames<CameraBase>();

    if (driver_classes.empty()) {
        AWARN << "No camera driver classes found in library: " << library_path;
        class_loader->UnloadLibrary();
        return false;
    }

    // 保存插件库信息
    PluginLibrary plugin_lib;
    plugin_lib.library_path = library_path;
    plugin_lib.class_loader = class_loader;
    plugin_lib.driver_classes = driver_classes;

    plugin_libraries_[library_path] = plugin_lib;

    AINFO << "Loaded camera driver plugin: " << library_path << " with "
          << driver_classes.size() << " driver classes";
    for (const auto& class_name : driver_classes) {
        AINFO << "  - " << class_name;
    }

    return true;
}

bool CameraManager::UnloadDriverPlugin(const std::string& library_path) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = plugin_libraries_.find(library_path);
    if (it == plugin_libraries_.end()) {
        AWARN << "Plugin library not loaded: " << library_path;
        return false;
    }

    // 检查是否有驱动实例正在使用此插件
    // 注意：由于 ClassLoader 管理生命周期，卸载库前需要确保没有驱动实例在使用

    // 卸载库
    if (it->second.class_loader) {
        it->second.class_loader->UnloadLibrary();
    }

    plugin_libraries_.erase(it);

    AINFO << "Unloaded camera driver plugin: " << library_path;
    return true;
}

std::vector<std::string> CameraManager::GetAvailableDriverClasses() const {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<std::string> all_classes;
    for (const auto& plugin_pair : plugin_libraries_) {
        for (const auto& class_name : plugin_pair.second.driver_classes) {
            all_classes.push_back(class_name);
        }
    }

    return all_classes;
}

CameraBase::SharedPtr CameraManager::CreateDriver(
    const std::string& driver_class_name, const std::string& driver_name) {
    std::lock_guard<std::mutex> lock(mutex_);

    // 在所有已加载的插件库中查找驱动类
    for (auto& plugin_pair : plugin_libraries_) {
        auto& plugin_lib = plugin_pair.second;

        // 检查此插件库是否包含该驱动类
        auto it = std::find(plugin_lib.driver_classes.begin(),
                            plugin_lib.driver_classes.end(), driver_class_name);
        if (it != plugin_lib.driver_classes.end()) {
            // 从此插件库创建驱动实例
            auto driver = plugin_lib.class_loader->CreateClassObj<CameraBase>(
                driver_class_name);
            if (driver != nullptr) {
                AINFO << "Created camera driver instance: " << driver_name
                      << " (class: " << driver_class_name
                      << ", library: " << plugin_lib.library_path << ")";
                return driver;
            } else {
                AERROR << "Failed to create driver instance: "
                       << driver_class_name
                       << " from library: " << plugin_lib.library_path;
            }
        }
    }

    AERROR << "Driver class not found in any loaded plugin: "
           << driver_class_name;
    return nullptr;
}

CameraBase::SharedPtr CameraManager::CreateDriverFromPlugin(
    const std::string& library_path, const std::string& driver_class_name) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = plugin_libraries_.find(library_path);
    if (it == plugin_libraries_.end()) {
        AERROR << "Plugin library not loaded: " << library_path;
        return nullptr;
    }

    auto driver =
        it->second.class_loader->CreateClassObj<CameraBase>(driver_class_name);
    if (driver != nullptr) {
        AINFO << "Created camera driver from plugin: " << driver_class_name
              << " (library: " << library_path << ")";
    }

    return driver;
}

bool CameraManager::RegisterDriver(const std::string& driver_name,
                                   CameraBase::SharedPtr driver) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (driver == nullptr) {
        AERROR << "Cannot register null driver: " << driver_name;
        return false;
    }

    if (drivers_.find(driver_name) != drivers_.end()) {
        AWARN << "Driver already registered: " << driver_name;
        return false;
    }

    drivers_[driver_name] = driver;
    AINFO << "Registered camera driver: " << driver_name;
    return true;
}

void CameraManager::UnregisterDriver(const std::string& driver_name) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = drivers_.find(driver_name);
    if (it != drivers_.end()) {
        // 停止驱动
        if (it->second) {
            it->second->Stop();
            it->second->Cleanup();
        }
        drivers_.erase(it);
        AINFO << "Unregistered camera driver: " << driver_name;
    }
}

CameraBase::SharedPtr CameraManager::GetDriver(
    const std::string& driver_name) const {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = drivers_.find(driver_name);
    if (it != drivers_.end()) {
        return it->second;
    }

    return nullptr;
}

std::vector<std::string> CameraManager::GetRegisteredDriverNames() const {
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<std::string> names;
    for (const auto& pair : drivers_) {
        names.push_back(pair.first);
    }

    return names;
}

bool CameraManager::ConfigureFromOptions(const proto::DriverOptions& options) {
    std::lock_guard<std::mutex> lock(mutex_);

    // 根据配置中的相机选项创建和配置驱动
    // 注意：这里假设配置中包含了驱动类名和插件库路径信息
    // 实际实现中可能需要扩展 proto 定义来包含这些信息

    for (const auto& camera_option : options.cameras()) {
        if (!camera_option.enabled()) {
            continue;
        }

        // TODO: 从配置中获取驱动类名和插件库路径
        // 这里需要扩展 CameraOptions 来包含这些信息
        // 或者从其他配置源获取

        // 示例：假设配置中有 driver_class 和 plugin_library 字段
        // std::string driver_class = camera_option.driver_class();
        // std::string plugin_library = camera_option.plugin_library();

        // 临时实现：使用默认值
        std::string driver_name = "camera_driver_" + camera_option.sensor_id();

        // 检查驱动是否已存在
        if (drivers_.find(driver_name) == drivers_.end()) {
            AWARN << "Camera driver configuration found but driver creation "
                     "not yet "
                     "implemented. "
                  << "Sensor ID: " << camera_option.sensor_id();
            // 需要扩展配置来支持驱动类名和插件库路径
        }
    }

    return true;
}

void CameraManager::StartAllDrivers() {
    std::lock_guard<std::mutex> lock(mutex_);

    for (auto& pair : drivers_) {
        if (pair.second) {
            if (!pair.second->IsConnected()) {
                AWARN << "Camera driver not connected: " << pair.first;
                continue;
            }

            if (!pair.second->Initialize()) {
                AERROR << "Failed to initialize camera driver: " << pair.first;
                continue;
            }

            pair.second->Start();
            AINFO << "Started camera driver: " << pair.first;
        }
    }
}

void CameraManager::StopAllDrivers() {
    std::lock_guard<std::mutex> lock(mutex_);

    for (auto& pair : drivers_) {
        if (pair.second) {
            pair.second->Stop();
            AINFO << "Stopped camera driver: " << pair.first;
        }
    }
}

void CameraManager::Cleanup() {
    std::lock_guard<std::mutex> lock(mutex_);

    // 停止并清理所有驱动
    StopAllDrivers();

    for (auto& pair : drivers_) {
        if (pair.second) {
            pair.second->Cleanup();
        }
    }
    drivers_.clear();

    // 卸载所有插件库
    for (auto& plugin_pair : plugin_libraries_) {
        if (plugin_pair.second.class_loader) {
            plugin_pair.second.class_loader->UnloadLibrary();
        }
    }
    plugin_libraries_.clear();

    AINFO << "CameraManager cleaned up";
}

}  // namespace camera
}  // namespace sensor
}  // namespace driver
}  // namespace autonomy
