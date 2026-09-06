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

#include <memory>

#include "autodriver/lidar/backend_registry.hpp"
#include "autodriver/sensor_driver.hpp"

namespace {

struct DummyDriver : autodriver::SensorDriver {
    autodriver::SensorType GetType() const override {
        return autodriver::SensorType::kLidar3d;
    }
    const autodriver::SensorId& GetSensorId() const override { return id_; }
    bool Start() override { return true; }
    void Stop() override {}
    bool IsRunning() const override { return false; }
    void SetSampleCallback(autodriver::SampleCallback) override {}
    autodriver::SensorId id_{"lidar/dummy"};
};

}  // namespace

TEST(LidarBackendRegistry, RegisterAndCreate) {
    auto& reg = autodriver::lidar::LidarBackendRegistry::Instance();
    reg.Register("fake_lidar_ut", [](const autodriver::SensorId& id,
                                     const autodriver::hardware::DriverParams&) {
        auto driver = std::make_shared<DummyDriver>();
        driver->id_ = id;
        return driver;
    });
    reg.RegisterAlias("fake_alias_ut", "fake_lidar_ut");
    EXPECT_TRUE(reg.Has("fake_lidar_ut"));
    EXPECT_TRUE(reg.Has("fake_alias_ut"));
    autodriver::hardware::DriverParams params;
    auto driver = reg.Create("fake_alias_ut", "lidar/x", params);
    ASSERT_NE(driver, nullptr);
    EXPECT_EQ(driver->GetSensorId(), "lidar/x");
    EXPECT_EQ(reg.Create("no_such_backend_ut", "lidar/y", params), nullptr);
}
