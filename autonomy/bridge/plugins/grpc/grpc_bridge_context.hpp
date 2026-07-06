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

#pragma once

#include <memory>

#include "autolink/node/node.hpp"
#include "autonomy/bridge/plugins/grpc/clients/navigator_stub.hpp"
#include "autonomy/common/async_grpc/execution_context.h"
#include "autonomy/common/macros.hpp"

class GrpcBridgeServer;

namespace autonomy {
namespace bridge {
namespace plugins {
namespace grpc {

/** Shared gRPC handler state: navigator action forwarding and future FSM. */
class GrpcBridgeContextInterface
    : public autonomy::common::async_grpc::ExecutionContext
{
public:
    explicit GrpcBridgeContextInterface(std::shared_ptr<autolink::Node> node);
    ~GrpcBridgeContextInterface() override = default;

    GrpcBridgeContextInterface(const GrpcBridgeContextInterface&) = delete;
    GrpcBridgeContextInterface& operator=(const GrpcBridgeContextInterface&) =
        delete;

    clients::NavigatorStub& navigator() { return *navigator_stub_; }
    const clients::NavigatorStub& navigator() const { return *navigator_stub_; }

private:
    clients::NavigatorStub::SharedPtr navigator_stub_;
};

}  // namespace grpc
}  // namespace plugins
}  // namespace bridge
}  // namespace autonomy
