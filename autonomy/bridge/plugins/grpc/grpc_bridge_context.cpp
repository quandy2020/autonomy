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

#include "autonomy/bridge/plugins/grpc/grpc_bridge_context.hpp"

namespace autonomy {
namespace bridge {
namespace plugins {
namespace grpc {

GrpcBridgeContextInterface::GrpcBridgeContextInterface(
    std::shared_ptr<autolink::Node> node)
    : navigator_stub_(std::make_shared<clients::NavigatorStub>(std::move(node))) {}

}  // namespace grpc
}  // namespace plugins
}  // namespace bridge
}  // namespace autonomy
