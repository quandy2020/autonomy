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

#include "autonomy/localization/cartographer/cloud/internal/handlers/load_state_from_file_handler.hpp"

#include "absl/memory/memory.h"
#include "autonomy/common/async_grpc/rpc_handler.h"
#include "autonomy/localization/cartographer/cloud/internal/map_builder_context_interface.hpp"
#include "autonomy/localization/cartographer/cloud/internal/mapping/serialization.hpp"
#include "autonomy/localization/cartographer/cloud/proto/map_builder_service.pb.h"

namespace cartographer {
namespace cloud {
namespace handlers {

void LoadStateFromFileHandler::OnRequest(const proto::LoadStateFromFileRequest& request) {
  // TODO(gaschler): This blocks a handler thread, consider working in
  // background.
  auto trajectory_remapping = GetContext<MapBuilderContextInterface>()->map_builder().LoadStateFromFile(
      request.file_path(), request.load_frozen_state());
  for (const auto& entry : trajectory_remapping) {
    GetContext<MapBuilderContextInterface>()->RegisterClientIdForTrajectory(request.client_id(), entry.second);
  }
  auto response = absl::make_unique<proto::LoadStateFromFileResponse>();
  *response->mutable_trajectory_remapping() = ToProto(trajectory_remapping);
  Send(std::move(response));
}

}  // namespace handlers
}  // namespace cloud
}  // namespace cartographer
