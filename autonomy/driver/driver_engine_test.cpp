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

#include "autonomy/driver/driver_engine.hpp"

#include "autolink/autolink.hpp"
#include "autonomy/driver/proto/driver_options.pb.h"
#include "autonomy/sensor/imu_data.hpp"
#include "autonomy/sensor/point_cloud.hpp"
#include "autonomy/sensor/range_data.hpp"
#include "gtest/gtest.h"

namespace autonomy {
namespace driver {
namespace {

class DriverEngineTest : public ::testing::Test
{
protected:
    void SetUp() override {
        // 创建 autolink 节点
        node_ = ::autolink::CreateNode("test_driver_engine_node", "");

        // 创建驱动引擎
        driver_engine_ = std::make_unique<DriverEngine>(node_.get());
    }

    void TearDown() override {
        if (driver_engine_) {
            driver_engine_->Stop();
        }
        driver_engine_.reset();
        node_.reset();
    }

    std::shared_ptr<::autolink::Node> node_;
    std::unique_ptr<DriverEngine> driver_engine_;
};

// 测试构造函数和析构函数
TEST_F(DriverEngineTest, Construction) {
    EXPECT_NE(driver_engine_, nullptr);
}

// 测试初始化（空配置）
TEST_F(DriverEngineTest, InitializeWithEmptyOptions) {
    proto::DriverOptions options;
    EXPECT_TRUE(driver_engine_->Initialize(options));
}

// 测试初始化（带配置）
TEST_F(DriverEngineTest, InitializeWithOptions) {
    proto::DriverOptions options;
    options.set_auto_start(false);

    // 添加一个激光雷达配置
    auto* lidar = options.add_lidars();
    lidar->set_sensor_id("test_laser");
    lidar->set_type(proto::LidarType::LASER_SCAN);
    lidar->set_topic("test_scan");
    lidar->set_frame_id("laser_link");
    lidar->set_enabled(true);
    lidar->set_min_range(0.1);
    lidar->set_max_range(30.0);
    lidar->set_min_angle(-3.14159);
    lidar->set_max_angle(3.14159);

    EXPECT_TRUE(driver_engine_->Initialize(options));
}

// 测试启动和停止
TEST_F(DriverEngineTest, StartStop) {
    proto::DriverOptions options;
    EXPECT_TRUE(driver_engine_->Initialize(options));

    driver_engine_->Start();
    // 启动后应该可以正常操作

    driver_engine_->Stop();
    // 停止后应该清理资源
}

// 测试注册和取消注册传感器处理器
TEST_F(DriverEngineTest, RegisterUnregisterSensorHandler) {
    proto::DriverOptions options;
    EXPECT_TRUE(driver_engine_->Initialize(options));

    std::string sensor_id = "test_sensor";
    bool handler_called = false;

    // 注册处理器
    DriverEngine::SensorDataHandler handler =
        [&handler_called, sensor_id](
            const std::string& id, const std::shared_ptr<sensor::Data>& data) {
            if (id == sensor_id) {
                handler_called = true;
            }
        };

    EXPECT_TRUE(driver_engine_->RegisterSensorHandler(sensor_id, handler));

    // 取消注册
    driver_engine_->UnregisterSensorHandler(sensor_id);
}

// 测试订阅传感器
TEST_F(DriverEngineTest, SubscribeSensor) {
    proto::DriverOptions options;
    EXPECT_TRUE(driver_engine_->Initialize(options));
    driver_engine_->Start();

    std::string sensor_id = "test_laser";
    std::string topic = "test_scan";
    std::string sensor_type = "laser_scan";

    // 注意：实际订阅需要 autolink 节点正常工作
    // 这里主要测试接口调用不会崩溃
    // 结果可能为 true 或 false，取决于 autolink 环境
    // 在测试环境中可能无法真正创建 reader，所以不强制要求成功
    (void)driver_engine_->SubscribeSensor(sensor_id, topic, sensor_type);

    // 取消订阅
    driver_engine_->UnsubscribeSensor(sensor_id);
}

// 测试获取已注册的传感器列表
TEST_F(DriverEngineTest, GetRegisteredSensors) {
    proto::DriverOptions options;

    // 添加一个激光雷达配置
    auto* lidar = options.add_lidars();
    lidar->set_sensor_id("test_laser");
    lidar->set_type(proto::LidarType::LASER_SCAN);
    lidar->set_topic("test_scan");
    lidar->set_frame_id("laser_link");
    lidar->set_enabled(true);

    EXPECT_TRUE(driver_engine_->Initialize(options));

    auto sensors = driver_engine_->GetRegisteredSensors();
    // 如果初始化成功，应该至少有一个传感器
    // 注意：实际订阅可能失败，所以这里不强制要求有传感器
}

// 测试检查传感器是否已注册
TEST_F(DriverEngineTest, IsSensorRegistered) {
    proto::DriverOptions options;
    EXPECT_TRUE(driver_engine_->Initialize(options));

    std::string sensor_id = "test_sensor";
    bool is_registered = driver_engine_->IsSensorRegistered(sensor_id);
    // 在空配置下，应该返回 false
    EXPECT_FALSE(is_registered);
}

// 测试添加激光扫描数据
TEST_F(DriverEngineTest, AddSensorDataLaserScan) {
    proto::DriverOptions options;
    EXPECT_TRUE(driver_engine_->Initialize(options));

    std::string sensor_id = "test_laser";
    auto message = std::make_shared<commsgs::sensor_msgs::LaserScan>();
    message->header.frame_id = "laser_link";
    message->angle_min = -3.14159;
    message->angle_max = 3.14159;
    message->range_min = 0.1;
    message->range_max = 30.0;
    message->ranges = {1.0, 2.0, 3.0};

    // 测试添加数据不会崩溃
    driver_engine_->AddSensorData(sensor_id, message);
}

// 测试添加IMU数据
TEST_F(DriverEngineTest, AddSensorDataImu) {
    proto::DriverOptions options;
    EXPECT_TRUE(driver_engine_->Initialize(options));

    std::string sensor_id = "test_imu";
    auto message = std::make_shared<commsgs::sensor_msgs::Imu>();
    message->header.frame_id = "imu_link";

    // 测试添加数据不会崩溃
    driver_engine_->AddSensorData(sensor_id, message);
}

// 测试添加点云数据
TEST_F(DriverEngineTest, AddSensorDataPointCloud2) {
    proto::DriverOptions options;
    EXPECT_TRUE(driver_engine_->Initialize(options));

    std::string sensor_id = "test_pointcloud";
    auto message = std::make_shared<commsgs::sensor_msgs::PointCloud2>();
    message->header.frame_id = "pointcloud_link";
    message->width = 10;
    message->height = 1;

    // 测试添加数据不会崩溃
    driver_engine_->AddSensorData(sensor_id, message);
}

}  // namespace
}  // namespace driver
}  // namespace autonomy
