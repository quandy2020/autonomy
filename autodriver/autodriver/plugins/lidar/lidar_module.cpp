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

#include "autodriver/sensor_plugin.hpp"
#include "autolink/class_loader/class_loader_register_macro.hpp"

class Lidar2dModule
    : public autodriver::SensorPlugin<autodriver::SensorType::kLidar2d, false> {
};

class Lidar3dModule
    : public autodriver::SensorPlugin<autodriver::SensorType::kLidar3d, false> {
};

class LidarModule : public Lidar2dModule {};

CLASS_LOADER_REGISTER_CLASS(Lidar2dModule, autodriver::SensorModule)
CLASS_LOADER_REGISTER_CLASS(Lidar3dModule, autodriver::SensorModule)
CLASS_LOADER_REGISTER_CLASS(LidarModule, autodriver::SensorModule)
