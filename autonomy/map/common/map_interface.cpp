/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/map/common/map_interface.hpp"

namespace autonomy {
namespace map {
namespace common {

void MapInterface::AddSensorData(std::unique_ptr<sensor::Data> data) {
    // 默认实现：什么都不做
    // 子类可以重写此方法来实现具体的传感器数据处理逻辑
    (void)data;  // 避免未使用参数警告
}

}  // namespace common
}  // namespace map
}  // namespace autonomy