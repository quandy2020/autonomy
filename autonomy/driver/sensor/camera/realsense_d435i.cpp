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

#include "autonomy/driver/sensor/camera/realsense_d435i.hpp"

#include <algorithm>
#include <cstring>

#include "autolink/class_loader/class_loader_register_macro.hpp"
#include "autolink/common/log.hpp"
#include "autonomy/common/time.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"

// RealSense SDK 前向声明（如果未安装 SDK，可以使用模拟实现）
// 注意：实际使用时需要链接 librealsense2 库
#ifdef HAVE_LIBREALSENSE2
#include <librealsense2/rs.hpp>
#else
// 模拟 RealSense 类型（用于编译，实际运行时需要安装 SDK）
namespace rs2 {
class pipeline {};
class frameset {};
class frame {};
class config {};
}  // namespace rs2
#endif

namespace autonomy {
namespace driver {
namespace sensor {
namespace camera {

RealSenseD435iDriver::RealSenseD435iDriver() : hardware_initialized_(false), pipeline_ptr_(nullptr) {}

RealSenseD435iDriver::~RealSenseD435iDriver() { Cleanup(); }

std::string RealSenseD435iDriver::GetHardwareModel() const { return "RealSense D435i"; }

std::string RealSenseD435iDriver::GetVersion() const { return "1.0.0"; }

bool RealSenseD435iDriver::IsConnected() const {
  std::lock_guard<std::mutex> lock(hardware_mutex_);

  if (!hardware_initialized_) {
    return false;
  }

#ifdef HAVE_LIBREALSENSE2
  // TODO: 实现 RealSense SDK 的连接检查
  // rs2::context ctx;
  // auto devices = ctx.query_devices();
  // return devices.size() > 0;
  return true;  // 临时返回 true
#else
  // 未安装 SDK 时返回 false
  AWARN << "RealSense SDK not available. Install librealsense2 to use "
           "RealSense D435i driver.";
  return false;
#endif
}

bool RealSenseD435iDriver::Initialize() {
  std::lock_guard<std::mutex> lock(hardware_mutex_);

  // 先调用基类初始化
  if (!CameraBase::Initialize()) {
    return false;
  }

  // 保存 frame_id 映射（从基类的配置中获取）
  // 注意：基类的 camera_configs_ 是 protected，可以直接访问
  for (const auto& pair : camera_configs_) {
    sensor_frame_ids_[pair.first] = pair.second.frame_id();
  }

  // 初始化硬件
  if (!InitializeHardware()) {
    AERROR << "Failed to initialize RealSense D435i hardware";
    return false;
  }

  hardware_initialized_ = true;
  AINFO << "RealSense D435i driver initialized successfully";
  return true;
}

void RealSenseD435iDriver::Cleanup() {
  std::lock_guard<std::mutex> lock(hardware_mutex_);

  // 调用基类清理
  CameraBase::Cleanup();

#ifdef HAVE_LIBREALSENSE2
  // TODO: 实现 RealSense SDK 的资源清理
  // if (pipeline_ptr_) {
  //     auto* pipeline =
  //     static_cast<std::shared_ptr<rs2::pipeline>*>(pipeline_ptr_); if
  //     (pipeline && *pipeline) {
  //         (*pipeline)->stop();
  //     }
  //     delete pipeline;
  //     pipeline_ptr_ = nullptr;
  // }
#else
  if (pipeline_ptr_) {
    // 清理指针（如果使用了其他实现）
    pipeline_ptr_ = nullptr;
  }
#endif

  hardware_initialized_ = false;

  // 清理 frame_id 映射
  sensor_frame_ids_.clear();

  AINFO << "RealSense D435i driver cleaned up";
}

bool RealSenseD435iDriver::InitializeHardware() {
#ifdef HAVE_LIBREALSENSE2
  // TODO: 实现 RealSense SDK 的硬件初始化
  // try {
  //     rs2::context ctx;
  //     auto devices = ctx.query_devices();
  //     if (devices.size() == 0) {
  //         AERROR << "No RealSense devices found";
  //         return false;
  //     }
  //
  //     rs2::config cfg;
  //     cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
  //     cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
  //
  //     pipeline_ = std::make_shared<rs2::pipeline>();
  //     pipeline_->start(cfg);
  //
  //     AINFO << "RealSense D435i hardware initialized successfully";
  //     return true;
  // } catch (const rs2::error& e) {
  //     AERROR << "RealSense error: " << e.what();
  //     return false;
  // }
  AINFO << "RealSense D435i hardware initialization (SDK available, "
           "implementation pending)";
  return true;
#else
  AWARN << "RealSense SDK (librealsense2) not available. "
        << "Install librealsense2 to enable RealSense D435i driver "
           "functionality.";
  // 返回 false 表示硬件未初始化，但不阻止编译
  return false;
#endif
}

std::shared_ptr<commsgs::sensor_msgs::Image> RealSenseD435iDriver::ReadImageData(const std::string& sensor_id) {
  std::lock_guard<std::mutex> lock(hardware_mutex_);

  if (!hardware_initialized_) {
    return nullptr;
  }

#ifdef HAVE_LIBREALSENSE2
  // TODO: 实现 RealSense SDK 的图像读取
  // try {
  //     rs2::frameset frames = pipeline_->wait_for_frames();
  //     rs2::frame color_frame = frames.get_color_frame();
  //
  //     if (!color_frame) {
  //         return nullptr;
  //     }
  //
  //     auto image_msg = std::make_shared<commsgs::sensor_msgs::Image>();
  //
  //     // 设置时间戳
  //     image_msg->header.stamp = commsgs::builtin_interfaces::Time::Now();
  //
  //     // 从配置中获取 frame_id
  //     auto frame_it = sensor_frame_ids_.find(sensor_id);
  //     if (frame_it != sensor_frame_ids_.end()) {
  //         image_msg->header.frame_id = frame_it->second;
  //     } else {
  //         image_msg->header.frame_id = "camera_color_optical_frame";
  //     }
  //
  //     // 获取图像参数
  //     int width = color_frame.get_width();
  //     int height = color_frame.get_height();
  //     int stride = color_frame.get_stride_in_bytes();
  //
  //     image_msg->width = static_cast<uint32_t>(width);
  //     image_msg->height = static_cast<uint32_t>(height);
  //     image_msg->step = static_cast<uint32_t>(stride);
  //     image_msg->encoding = "rgb8";
  //     image_msg->is_bigendian = 0;
  //
  //     // 复制图像数据
  //     const void* data = color_frame.get_data();
  //     size_t data_size = stride * height;
  //     image_msg->data.resize(data_size);
  //     std::memcpy(image_msg->data.data(), data, data_size);
  //
  //     return image_msg;
  // } catch (const rs2::error& e) {
  //     AERROR << "RealSense error reading color frame: " << e.what();
  //     return nullptr;
  // }

  // 临时返回 nullptr（待实现）
  return nullptr;
#else
  // 未安装 SDK 时返回 nullptr
  return nullptr;
#endif
}

std::shared_ptr<commsgs::sensor_msgs::Image> RealSenseD435iDriver::ReadDepthData(const std::string& sensor_id) {
  std::lock_guard<std::mutex> lock(hardware_mutex_);

  if (!hardware_initialized_) {
    return nullptr;
  }

#ifdef HAVE_LIBREALSENSE2
  // TODO: 实现 RealSense SDK 的深度图像读取
  // try {
  //     rs2::frameset frames = pipeline_->wait_for_frames();
  //     rs2::frame depth_frame = frames.get_depth_frame();
  //
  //     if (!depth_frame) {
  //         return nullptr;
  //     }
  //
  //     auto image_msg = std::make_shared<commsgs::sensor_msgs::Image>();
  //
  //     // 设置时间戳
  //     image_msg->header.stamp = commsgs::builtin_interfaces::Time::Now();
  //
  //     // 从配置中获取 frame_id
  //     auto frame_it = sensor_frame_ids_.find(sensor_id);
  //     if (frame_it != sensor_frame_ids_.end()) {
  //         image_msg->header.frame_id = frame_it->second + "_depth";
  //     } else {
  //         image_msg->header.frame_id = "camera_depth_optical_frame";
  //     }
  //
  //     // 获取图像参数
  //     int width = depth_frame.get_width();
  //     int height = depth_frame.get_height();
  //     int stride = depth_frame.get_stride_in_bytes();
  //
  //     image_msg->width = static_cast<uint32_t>(width);
  //     image_msg->height = static_cast<uint32_t>(height);
  //     image_msg->step = static_cast<uint32_t>(stride);
  //     image_msg->encoding = "16UC1";  // 16-bit unsigned, single channel
  //     image_msg->is_bigendian = 0;
  //
  //     // 复制图像数据
  //     const void* data = depth_frame.get_data();
  //     size_t data_size = stride * height;
  //     image_msg->data.resize(data_size);
  //     std::memcpy(image_msg->data.data(), data, data_size);
  //
  //     return image_msg;
  // } catch (const rs2::error& e) {
  //     AERROR << "RealSense error reading depth frame: " << e.what();
  //     return nullptr;
  // }

  // 临时返回 nullptr（待实现）
  return nullptr;
#else
  // 未安装 SDK 时返回 nullptr
  return nullptr;
#endif
}

}  // namespace camera
}  // namespace sensor
}  // namespace driver
}  // namespace autonomy

// 注册驱动类为插件，支持动态库加载
CLASS_LOADER_REGISTER_CLASS(autonomy::driver::sensor::camera::RealSenseD435iDriver,
                            autonomy::driver::sensor::camera::CameraBase)
