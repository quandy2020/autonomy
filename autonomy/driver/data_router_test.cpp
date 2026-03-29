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

#include "autonomy/driver/data_router.hpp"

#include "autolink/autolink.hpp"
#include "autonomy/common/time.hpp"
#include "autonomy/driver/driver_engine.hpp"
#include "autonomy/driver/proto/driver_options.pb.h"
#include "autonomy/map/common/map_interface.hpp"
#include "autonomy/sensor/data.hpp"
#include "gtest/gtest.h"

namespace autonomy {
namespace driver {
namespace {

class DataRouterTest : public ::testing::Test
{
protected:
    void SetUp() override {
        // 创建 autolink 节点
        node_ = ::autolink::CreateNode("test_data_router_node", "");

        // 创建驱动引擎
        driver_engine_ = std::make_unique<DriverEngine>(node_.get());

        // 创建数据路由器
        data_router_ =
            std::make_unique<DataRouter>(node_.get(), driver_engine_.get());
    }

    void TearDown() override {
        if (data_router_) {
            data_router_->Stop();
        }
        if (driver_engine_) {
            driver_engine_->Stop();
        }
        data_router_.reset();
        driver_engine_.reset();
        node_.reset();
    }

    std::shared_ptr<::autolink::Node> node_;
    std::unique_ptr<DriverEngine> driver_engine_;
    std::unique_ptr<DataRouter> data_router_;
};

// 测试构造函数和析构函数
TEST_F(DataRouterTest, Construction) {
    EXPECT_NE(data_router_, nullptr);
}

// 测试初始化（空配置）
TEST_F(DataRouterTest, InitializeWithEmptyOptions) {
    proto::DriverOptions options;
    EXPECT_TRUE(data_router_->Initialize(options));
}

// 测试初始化（带配置）
TEST_F(DataRouterTest, InitializeWithOptions) {
    proto::DriverOptions options;
    options.set_auto_start(false);
    options.add_default_forward_targets("map");
    options.add_default_forward_targets("localization");

    EXPECT_TRUE(data_router_->Initialize(options));
}

// 测试启动和停止
TEST_F(DataRouterTest, StartStop) {
    proto::DriverOptions options;
    EXPECT_TRUE(data_router_->Initialize(options));

    data_router_->Start();
    // 启动后应该可以正常操作

    data_router_->Stop();
    // 停止后应该清理资源
}

// 测试设置和获取数据源类型
TEST_F(DataRouterTest, SetGetDataSource) {
    proto::DriverOptions options;
    EXPECT_TRUE(data_router_->Initialize(options));

    std::string sensor_id = "test_sensor";

    // 设置数据源为 ROS2
    data_router_->SetDataSource(sensor_id, DataRouter::DataSource::ROS2);
    EXPECT_EQ(data_router_->GetDataSource(sensor_id),
              DataRouter::DataSource::ROS2);

    // 设置数据源为 DRIVER
    data_router_->SetDataSource(sensor_id, DataRouter::DataSource::DRIVER);
    EXPECT_EQ(data_router_->GetDataSource(sensor_id),
              DataRouter::DataSource::DRIVER);
}

// 测试从 ROS2 转发数据
TEST_F(DataRouterTest, ForwardFromRos2) {
    proto::DriverOptions options;
    options.add_default_forward_targets("map");
    EXPECT_TRUE(data_router_->Initialize(options));

    std::string sensor_id = "test_sensor";
    data_router_->SetDataSource(sensor_id, DataRouter::DataSource::ROS2);

    // 创建一个简单的测试数据类
    class TestSensorData : public sensor::Data
    {
    public:
        TestSensorData(const std::string& sensor_id) : Data(sensor_id) {}
        sensor::Time GetTime() const override {
            return sensor::Time{};
        }
        void AddToCostmap(map::common::MapInterface* costmap_builder) override {
            (void)costmap_builder;
        }
    };

    auto data = std::make_shared<TestSensorData>(sensor_id);

    // 注册目标处理器
    bool handler_called = false;
    data_router_->RegisterTargetHandler(
        "map",
        [&handler_called, sensor_id](const std::string& id,
                                     const std::shared_ptr<sensor::Data>& d) {
            if (id == sensor_id) {
                handler_called = true;
            }
        });

    // 转发数据
    data_router_->ForwardFromRos2(sensor_id, data);

    // 注意：由于没有启动，处理器可能不会被调用
    // 这里主要测试接口调用不会崩溃
}

// 测试注册和取消注册目标处理器
TEST_F(DataRouterTest, RegisterUnregisterTargetHandler) {
    proto::DriverOptions options;
    EXPECT_TRUE(data_router_->Initialize(options));

    std::string target = "map";
    bool handler_called = false;

    // 注册处理器
    data_router_->RegisterTargetHandler(
        target, [&handler_called](const std::string&,
                                  const std::shared_ptr<sensor::Data>&) {
            handler_called = true;
        });

    // 取消注册
    data_router_->UnregisterTargetHandler(target);
}

// 测试转发数据到目标
TEST_F(DataRouterTest, ForwardToTargets) {
    proto::DriverOptions options;
    options.add_default_forward_targets("map");
    EXPECT_TRUE(data_router_->Initialize(options));

    std::string sensor_id = "test_sensor";

    // 创建一个简单的测试数据类
    class TestSensorData : public sensor::Data
    {
    public:
        TestSensorData(const std::string& sensor_id) : Data(sensor_id) {}
        sensor::Time GetTime() const override {
            return sensor::Time{};
        }
        void AddToCostmap(map::common::MapInterface* costmap_builder) override {
            (void)costmap_builder;
        }
    };

    auto data = std::make_shared<TestSensorData>(sensor_id);

    // 注册目标处理器
    bool handler_called = false;
    data_router_->RegisterTargetHandler(
        "map",
        [&handler_called, sensor_id](const std::string& id,
                                     const std::shared_ptr<sensor::Data>&) {
            if (id == sensor_id) {
                handler_called = true;
            }
        });

    // 转发到默认目标
    data_router_->ForwardToTargets(sensor_id, data);

    // 转发到指定目标
    std::vector<std::string> targets = {"map"};
    data_router_->ForwardToTargets(sensor_id, data, targets);
}

// 测试从驱动读取数据
TEST_F(DataRouterTest, ReadAndSendFromDriver) {
    proto::DriverOptions options;
    EXPECT_TRUE(data_router_->Initialize(options));

    std::string sensor_id = "test_sensor";
    data_router_->SetDataSource(sensor_id, DataRouter::DataSource::DRIVER);

    // 测试读取不会崩溃
    // 注意：实际读取由 DriverEngine 的回调处理
    data_router_->ReadAndSendFromDriver(sensor_id);
}

// 测试数据源检测
TEST_F(DataRouterTest, DetectDataSource) {
    proto::DriverOptions options;

    // 添加一个激光雷达配置
    auto* lidar = options.add_lidars();
    lidar->set_sensor_id("test_laser");
    lidar->set_type(proto::LidarType::LASER_SCAN);
    lidar->set_topic("test_scan");
    lidar->set_frame_id("laser_link");
    lidar->set_enabled(true);

    EXPECT_TRUE(driver_engine_->Initialize(options));
    EXPECT_TRUE(data_router_->Initialize(options));

    // 初始化后，数据源应该被检测
    // 注意：实际检测逻辑可能返回 ROS2 或 DRIVER，取决于实现
}

// 测试空数据转发
TEST_F(DataRouterTest, ForwardNullData) {
    proto::DriverOptions options;
    EXPECT_TRUE(data_router_->Initialize(options));

    std::string sensor_id = "test_sensor";

    // 转发空数据应该不会崩溃
    data_router_->ForwardFromRos2(sensor_id, nullptr);
    data_router_->ForwardToTargets(sensor_id, nullptr);
}

}  // namespace
}  // namespace driver
}  // namespace autonomy
