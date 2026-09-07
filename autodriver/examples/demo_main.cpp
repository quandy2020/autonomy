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

#include "autodriver/bridge/publisher.hpp"
#include "autodriver/sensor_manager.hpp"
#include "autolink/common/log.hpp"
#include "autolink/init.hpp"
#include "autolink/time/duration.hpp"

int main(int argc, char** argv) {
    autolink::Init(argv[0]);
    autodriver::Config config;
    autodriver::Config::Sensor lidar2d;
    lidar2d.module = "Lidar2dModule";
    lidar2d.id = "lidar/front";
    lidar2d.autostart = true;
    autodriver::Config::Sensor lidar3d;
    lidar3d.module = "Lidar3dModule";
    lidar3d.id = "lidar/velo";
    lidar3d.autostart = true;
    config.sensors = {lidar2d, lidar3d};
    autodriver::bridge::Publisher publisher(config.node_name);
    if (!publisher.Initialize()) {
        AERROR << "autolink publisher failed";
        autolink::Clear();
        return 1;
    }
    autodriver::SensorManager manager(std::move(config));
    manager.SetSink(&publisher);
    if (!manager.Initialize() || !manager.Start()) {
        AERROR << "SensorManager failed";
        autolink::Clear();
        return 1;
    }
    AINFO << "autodriver_demo running";
    autolink::Duration(300'000'000).Sleep();
    manager.Stop();
    autolink::Clear();
    return 0;
}
