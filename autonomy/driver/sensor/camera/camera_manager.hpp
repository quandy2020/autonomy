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
#include <vector>

#include "autolink/class_loader/class_loader.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/driver/proto/driver_options.pb.h"
#include "autonomy/driver/sensor/camera/camera_base.hpp"

namespace autonomy {
namespace driver {
namespace sensor {
namespace camera {

/**
 * @class CameraManager
 * @brief 相机驱动管理器，支持动态库插件加载、配置化和热插拔
 *
 * CameraManager 提供：
 * 1. 动态加载相机驱动插件（.so 文件）
 * 2. 根据配置创建和配置驱动实例
 * 3. 管理驱动的生命周期（初始化、启动、停止）
 * 4. 支持热插拔（动态加载/卸载驱动）
 * 5. 驱动实例的注册和查找
 */
class CameraManager
{
public:
    /**
     * Define CameraManager::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(CameraManager)

    /**
     * @brief 构造函数
     */
    CameraManager();

    /**
     * @brief 析构函数
     */
    ~CameraManager();

    CameraManager(const CameraManager&) = delete;
    CameraManager& operator=(const CameraManager&) = delete;

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
     * @brief 创建驱动实例
     * @param driver_class_name 驱动类名（如 "RealSenseD435iDriver",
     * "UsbCameraDriver"）
     * @param driver_name 驱动实例名称
     * @return 驱动实例指针，失败返回 nullptr
     */
    CameraBase::SharedPtr CreateDriver(const std::string& driver_class_name,
                                       const std::string& driver_name);

    /**
     * @brief 注册驱动实例
     * @param driver_name 驱动实例名称
     * @param driver 驱动实例指针
     * @return true 成功，false 失败
     */
    bool RegisterDriver(const std::string& driver_name,
                        CameraBase::SharedPtr driver);

    /**
     * @brief 取消注册驱动实例
     * @param driver_name 驱动实例名称
     */
    void UnregisterDriver(const std::string& driver_name);

    /**
     * @brief 获取驱动实例
     * @param driver_name 驱动实例名称
     * @return 驱动实例指针，不存在返回 nullptr
     */
    CameraBase::SharedPtr GetDriver(const std::string& driver_name) const;

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
    CameraBase::SharedPtr CreateDriverFromPlugin(
        const std::string& library_path, const std::string& driver_class_name);

    // 插件库映射表（library_path -> PluginLibrary）
    std::map<std::string, PluginLibrary> plugin_libraries_;

    // 驱动实例映射表（driver_name -> CameraBase::SharedPtr）
    std::map<std::string, CameraBase::SharedPtr> drivers_;

    // 互斥锁
    mutable std::mutex mutex_;
};

}  // namespace camera
}  // namespace sensor
}  // namespace driver
}  // namespace autonomy
