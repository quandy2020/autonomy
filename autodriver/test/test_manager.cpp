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

#include <gtest/gtest.h>

#include "autodriver/sensor_manager.hpp"
#include "autodriver/sensor_module.hpp"
#include "autolink/class_loader/class_loader_register_macro.hpp"
#include "autolink/init.hpp"

class FakeModule : public autodriver::SensorModule {
public:
    autodriver::SensorType GetType() const override {
        return autodriver::SensorType::kImu;
    }
    const autodriver::SensorId& GetSensorId() const override { return id_; }

    bool Init(const autodriver::SensorModule::Context& context) override {
        id_ = context.sensor.id;
        return true;
    }
    bool Start() override {
        running_ = true;
        return true;
    }
    void Stop() override { running_ = false; }
    bool IsRunning() const override { return running_; }

private:
    autodriver::SensorId id_;
    bool running_ = false;
};

CLASS_LOADER_REGISTER_CLASS(FakeModule, autodriver::SensorModule)

class SensorManagerTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() { autolink::Init("test_manager"); }
};

TEST_F(SensorManagerTest, AttachIsIdempotent) {
    autodriver::Config config;
    autodriver::Config::Sensor sensor;
    sensor.module = "FakeModule";
    sensor.id = "imu/test";
    config.sensors = {sensor};
    autodriver::SensorManager manager(config);
    ASSERT_TRUE(manager.Initialize());
    EXPECT_TRUE(manager.Attach("imu/test"));
    EXPECT_TRUE(manager.Attach("imu/test"));
    EXPECT_EQ(manager.AttachedCount(), 1u);
    manager.Detach("imu/test");
    manager.Detach("imu/test");
    EXPECT_EQ(manager.AttachedCount(), 0u);
}

TEST_F(SensorManagerTest, AttachNImusBySensorId) {
    autodriver::Config config;
    autodriver::Config::Sensor a;
    a.module = "FakeModule";
    a.id = "imu/a";
    autodriver::Config::Sensor b;
    b.module = "FakeModule";
    b.id = "imu/b";
    config.sensors = {a, b};
    autodriver::SensorManager manager(config);
    ASSERT_TRUE(manager.Initialize());
    EXPECT_TRUE(manager.Attach("imu/a"));
    EXPECT_TRUE(manager.Attach("imu/b"));
    EXPECT_EQ(manager.AttachedCount(), 2u);
    manager.Detach("imu/a");
    EXPECT_EQ(manager.AttachedCount(), 1u);
    manager.Detach("imu/b");
    EXPECT_EQ(manager.AttachedCount(), 0u);
}

TEST_F(SensorManagerTest, UnknownIdFails) {
    autodriver::SensorManager manager;
    ASSERT_TRUE(manager.Initialize());
    EXPECT_FALSE(manager.Attach("nope"));
}

TEST_F(SensorManagerTest, DuplicateConfigFailsInitialize) {
    autodriver::Config config;
    autodriver::Config::Sensor a;
    a.id = "x";
    a.module = "FakeModule";
    config.sensors = {a, a};
    autodriver::SensorManager manager(config);
    EXPECT_FALSE(manager.Initialize());
}

TEST_F(SensorManagerTest, UnknownClassFails) {
    autodriver::Config config;
    autodriver::Config::Sensor sensor;
    sensor.module = "NoSuchModule";
    sensor.id = "imu/test";
    config.sensors = {sensor};
    autodriver::SensorManager manager(config);
    ASSERT_TRUE(manager.Initialize());
    EXPECT_FALSE(manager.Attach("imu/test"));
    EXPECT_EQ(manager.AttachedCount(), 0u);
}

TEST_F(SensorManagerTest, UnknownLibraryFails) {
    autodriver::Config config;
    autodriver::Config::Sensor sensor;
    sensor.module = "FakeModule";
    sensor.library = "libdoes_not_exist.so";
    sensor.id = "imu/test";
    config.sensors = {sensor};
    autodriver::SensorManager manager(config);
    ASSERT_TRUE(manager.Initialize());
    EXPECT_FALSE(manager.Attach("imu/test"));
    EXPECT_EQ(manager.AttachedCount(), 0u);
}

TEST_F(SensorManagerTest, DeviceAddAttachesMatchedSensor) {
    autodriver::Config config;
    autodriver::Config::Sensor sensor;
    sensor.module = "FakeModule";
    sensor.id = "imu/serial";
    sensor.match.subsystem = "tty";
    sensor.match.device = "/dev/ttyUSB0";
    config.sensors = {sensor};
    autodriver::SensorManager manager(config);
    ASSERT_TRUE(manager.Initialize());
    autodriver::DeviceMatch device;
    device.subsystem = "tty";
    device.device = "/dev/ttyUSB0";
    manager.HandleDeviceEvent(true, device);
    EXPECT_EQ(manager.AttachedCount(), 1u);
    manager.HandleDeviceEvent(false, device);
    EXPECT_EQ(manager.AttachedCount(), 0u);
}
