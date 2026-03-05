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

#pragma once

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <vector>

#include "autolink/class_loader/class_loader.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/driver/common/driver_interface.hpp"
#include "autonomy/driver/proto/driver_options.pb.h"

namespace autonomy {
namespace driver {

/**
 * @class PluginsManager
 * @brief 统一的驱动插件管理器，支持所有类型的传感器驱动
 *
 * PluginsManager 提供：
 * 1. 动态加载各种类型的驱动插件（.so 文件）
 * 2. 根据配置创建和配置驱动实例
 * 3. 管理驱动的生命周期（初始化、启动、停止）
 * 4. 支持热插拔（动态加载/卸载驱动）
 * 5. 驱动实例的注册和查找（类型安全）
 *
 * 所有传感器驱动基类（CameraBase, ImuBase, RangeBase, LidarBase, GpsBase）
 * 都继承自 DriverInterface，因此可以使用统一的接口进行管理。
 */
class PluginsManager {
 public:
  /**
   * Define PluginsManager::SharedPtr type
   */
  AUTONOMY_SMART_PTR_DEFINITIONS(PluginsManager)

  /**
   * @brief 构造函数
   */
  PluginsManager();

  /**
   * @brief 析构函数
   */
  ~PluginsManager();

  PluginsManager(const PluginsManager&) = delete;
  PluginsManager& operator=(const PluginsManager&) = delete;

  /**
   * @brief 加载驱动插件库
   * @param library_path 动态库路径（.so 文件）
   * @return true 成功，false 失败
   */
  bool LoadDriverPlugin(const std::string& library_path);

  /**
   * @brief 卸载驱动插件库
   * @param library_path 动态库路径
   * @return true 成功，false 失败
   */
  bool UnloadDriverPlugin(const std::string& library_path);

  /**
   * @brief 获取所有已加载的驱动类名
   * @return 驱动类名列表
   */
  std::vector<std::string> GetAvailableDriverClasses() const;

  /**
   * @brief 创建驱动实例（通用接口）
   * @param driver_class_name 驱动类名
   * @param driver_name 驱动实例名称
   * @return 驱动实例指针（DriverInterface），失败返回 nullptr
   */
  common::DriverInterface::SharedPtr CreateDriver(const std::string& driver_class_name, const std::string& driver_name);

  /**
   * @brief 创建驱动实例（类型安全版本）
   * @tparam DriverType 驱动类型（如 CameraBase, ImuBase 等）
   * @param driver_class_name 驱动类名
   * @param driver_name 驱动实例名称
   * @return 驱动实例指针，失败返回 nullptr
   */
  template <typename DriverType>
  std::shared_ptr<DriverType> CreateDriverTyped(const std::string& driver_class_name, const std::string& driver_name) {
    static_assert(std::is_base_of<common::DriverInterface, DriverType>::value,
                  "DriverType must inherit from DriverInterface");

    auto driver = CreateDriver(driver_class_name, driver_name);
    if (driver == nullptr) {
      return nullptr;
    }

    // 尝试转换为具体类型
    auto typed_driver = std::dynamic_pointer_cast<DriverType>(driver);
    if (typed_driver == nullptr) {
      return nullptr;
    }

    return typed_driver;
  }

  /**
   * @brief 注册驱动实例
   * @param driver_name 驱动实例名称
   * @param driver 驱动实例指针
   * @return true 成功，false 失败
   */
  bool RegisterDriver(const std::string& driver_name, common::DriverInterface::SharedPtr driver);

  /**
   * @brief 取消注册驱动实例
   * @param driver_name 驱动实例名称
   */
  void UnregisterDriver(const std::string& driver_name);

  /**
   * @brief 获取驱动实例（通用接口）
   * @param driver_name 驱动实例名称
   * @return 驱动实例指针，不存在返回 nullptr
   */
  common::DriverInterface::SharedPtr GetDriver(const std::string& driver_name) const;

  /**
   * @brief 获取驱动实例（类型安全版本）
   * @tparam DriverType 驱动类型（如 CameraBase, ImuBase 等）
   * @param driver_name 驱动实例名称
   * @return 驱动实例指针，不存在或类型不匹配返回 nullptr
   */
  template <typename DriverType>
  std::shared_ptr<DriverType> GetDriverTyped(const std::string& driver_name) const {
    static_assert(std::is_base_of<common::DriverInterface, DriverType>::value,
                  "DriverType must inherit from DriverInterface");

    auto driver = GetDriver(driver_name);
    if (driver == nullptr) {
      return nullptr;
    }

    // 尝试转换为具体类型
    return std::dynamic_pointer_cast<DriverType>(driver);
  }

  /**
   * @brief 获取所有已注册的驱动名称
   * @return 驱动名称列表
   */
  std::vector<std::string> GetRegisteredDriverNames() const;

  /**
   * @brief 根据配置自动创建和配置驱动
   * @param options 驱动配置选项
   * @return true 成功，false 失败
   */
  bool ConfigureFromOptions(const proto::DriverOptions& options);

  /**
   * @brief 启动所有已注册的驱动
   */
  void StartAllDrivers();

  /**
   * @brief 停止所有已注册的驱动
   */
  void StopAllDrivers();

  /**
   * @brief 清理所有驱动和插件
   */
  void Cleanup();

  /**
   * @brief 获取指定类型的驱动列表
   * @tparam DriverType 驱动类型
   * @return 驱动实例列表
   */
  template <typename DriverType>
  std::vector<std::shared_ptr<DriverType>> GetDriversByType() const {
    static_assert(std::is_base_of<common::DriverInterface, DriverType>::value,
                  "DriverType must inherit from DriverInterface");

    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::shared_ptr<DriverType>> typed_drivers;

    for (const auto& pair : drivers_) {
      auto typed_driver = std::dynamic_pointer_cast<DriverType>(pair.second);
      if (typed_driver != nullptr) {
        typed_drivers.push_back(typed_driver);
      }
    }

    return typed_drivers;
  }

 private:
  /**
   * @brief 插件库信息
   */
  struct PluginLibrary {
    std::string library_path;
    std::shared_ptr<::autolink::class_loader::ClassLoader> class_loader;
    std::vector<std::string> driver_classes;
  };

  /**
   * @brief 从插件库创建驱动实例
   * @param library_path 插件库路径
   * @param driver_class_name 驱动类名
   * @return 驱动实例指针
   */
  common::DriverInterface::SharedPtr CreateDriverFromPlugin(const std::string& library_path,
                                                            const std::string& driver_class_name);

  // 插件库映射表（library_path -> PluginLibrary）
  std::map<std::string, PluginLibrary> plugin_libraries_;

  // 驱动实例映射表（driver_name -> DriverInterface::SharedPtr）
  std::map<std::string, common::DriverInterface::SharedPtr> drivers_;

  // 互斥锁
  mutable std::mutex mutex_;
};

}  // namespace driver
}  // namespace autonomy
