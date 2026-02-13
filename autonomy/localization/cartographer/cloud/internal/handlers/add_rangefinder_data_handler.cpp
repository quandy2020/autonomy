/*
 * Copyright 2018 The Cartographer Authors
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

#include "autonomy/localization/cartographer/cloud/internal/handlers/add_rangefinder_data_handler.hpp"

#include "absl/memory/memory.h"
#include "autonomy/common/async_grpc/rpc_handler.h"
#include "autonomy/localization/cartographer/cloud/internal/map_builder_context_interface.hpp"
#include "autonomy/localization/cartographer/cloud/proto/map_builder_service.pb.h"
#include "autonomy/localization/cartographer/sensor/internal/dispatchable.hpp"
#include "autonomy/localization/cartographer/sensor/timed_point_cloud_data.hpp"
#include "google/protobuf/empty.pb.h"

namespace cartographer {
namespace cloud {
namespace handlers {

void AddRangefinderDataHandler::OnSensorData(const proto::AddRangefinderDataRequest& request) {
    // The 'BlockingQueue' returned by 'sensor_data_queue()' is already
    // thread-safe. Therefore it suffices to get an unsynchronized reference to
    // the 'MapBuilderContext'.
    this->template GetUnsynchronizedContext<MapBuilderContextInterface>()->EnqueueSensorData(
        request.sensor_metadata().trajectory_id(),
        sensor::MakeDispatchable(request.sensor_metadata().sensor_id(),
                                 sensor::FromProto(request.timed_point_cloud_data())));
}

}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer
