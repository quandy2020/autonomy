/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include <memory>

#include "autolink/node/node.hpp"
#include "autonomy/bridge/grpc/clients/dock_stub.hpp"
#include "autonomy/bridge/grpc/clients/exploration_stub.hpp"
#include "autonomy/bridge/grpc/clients/follow_stub.hpp"
#include "autonomy/bridge/grpc/clients/localization_stub.hpp"
#include "autonomy/bridge/grpc/clients/map_service_stub.hpp"
#include "autonomy/bridge/grpc/clients/map_stub.hpp"
#include "autonomy/bridge/grpc/clients/navigator_stub.hpp"
#include "autonomy/bridge/grpc/clients/relative_teleop_stub.hpp"
#include "autonomy/bridge/grpc/clients/sensor_stub.hpp"
#include "autonomy/bridge/grpc/clients/system_monitor_stub.hpp"
#include "autonomy/bridge/grpc/clients/teleop_stub.hpp"
#include "autonomy/bridge/grpc/clients/voice_stub.hpp"
#include "autonomy/bridge/grpc/state_hub.hpp"
#include "autonomy/bridge/grpc/task_muxer.hpp"
#include "autonomy/common/async_grpc/execution_context.h"
#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {

class GrpcBridgeContextInterface
    : public autonomy::common::async_grpc::ExecutionContext
{
public:
    explicit GrpcBridgeContextInterface(std::shared_ptr<autolink::Node> node);
    ~GrpcBridgeContextInterface() override = default;

    GrpcBridgeContextInterface(const GrpcBridgeContextInterface&) = delete;
    GrpcBridgeContextInterface& operator=(const GrpcBridgeContextInterface&) =
        delete;

    TaskMuxer& muxer() { return *muxer_; }
    StateHub& state_hub() { return *state_hub_; }

    clients::NavigatorStub& navigator() { return *navigator_stub_; }
    clients::TeleopStub& teleop() { return *teleop_stub_; }
    clients::RelativeTeleopStub& relative_teleop() {
        return *relative_teleop_stub_;
    }
    clients::FollowStub& follow() { return *follow_stub_; }
    clients::DockStub& dock() { return *dock_stub_; }
    clients::MapStub& map() { return *map_stub_; }
    clients::MapServiceStub& map_service() { return *map_service_stub_; }
    clients::LocalizationStub& localization() { return *localization_stub_; }
    clients::ExplorationStub& exploration() { return *exploration_stub_; }
    clients::VoiceStub& voice() { return *voice_stub_; }
    clients::SensorStub& sensor() { return *sensor_stub_; }
    clients::SystemMonitorStub& system_monitor() {
        return *system_monitor_stub_;
    }

    void CancelAllTasks();
    void EmergencyStop(bool engage);

private:
    std::shared_ptr<TaskMuxer> muxer_;
    std::shared_ptr<StateHub> state_hub_;
    clients::NavigatorStub::SharedPtr navigator_stub_;
    clients::TeleopStub::SharedPtr teleop_stub_;
    clients::RelativeTeleopStub::SharedPtr relative_teleop_stub_;
    clients::FollowStub::SharedPtr follow_stub_;
    clients::DockStub::SharedPtr dock_stub_;
    clients::MapStub::SharedPtr map_stub_;
    clients::MapServiceStub::SharedPtr map_service_stub_;
    clients::LocalizationStub::SharedPtr localization_stub_;
    clients::ExplorationStub::SharedPtr exploration_stub_;
    clients::VoiceStub::SharedPtr voice_stub_;
    clients::SensorStub::SharedPtr sensor_stub_;
    clients::SystemMonitorStub::SharedPtr system_monitor_stub_;
};

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
