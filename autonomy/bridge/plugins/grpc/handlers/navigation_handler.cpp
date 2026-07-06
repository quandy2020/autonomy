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

#include "autonomy/bridge/plugins/grpc/handlers/navigation_handler.hpp"

#include "autonomy/bridge/plugins/grpc/grpc_bridge_context.hpp"
#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace bridge {
namespace plugins {
namespace grpc {
namespace handlers {

void SendNavigationHandler::OnRequest(
    const proto::NavigationCommandRequest& request) {
    auto* context = GetUnsynchronizedContext<GrpcBridgeContextInterface>();
    if (!context) {
        LOG(ERROR) << "SendNavigationHandler: missing GrpcBridgeContextInterface.";
        Finish(::grpc::Status(::grpc::StatusCode::INTERNAL,
                              "bridge execution context unavailable"));
        return;
    }

    const bool accepted = context->navigator().HandleCommand(
        request, [this](const proto::NavigationCommandResponse& response) {
            auto message =
                std::make_unique<proto::NavigationCommandResponse>(response);
            if (response.ack().final()) {
                Send(std::move(message));
                Finish(::grpc::Status::OK);
                return;
            }
            Send(std::move(message));
        });

    if (!accepted) {
        LOG(WARNING) << "SendNavigationHandler: navigation command rejected.";
    }
}

}  // namespace handlers
}  // namespace grpc
}  // namespace plugins
}  // namespace bridge
}  // namespace autonomy
