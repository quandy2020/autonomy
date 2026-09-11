/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/bridge/grpc/grpc_bridge_context.hpp"

#include "autonomy/bridge/proto/external_command_service.pb.h"

namespace autonomy {
namespace bridge {
namespace grpc {

GrpcBridgeContextInterface::GrpcBridgeContextInterface(
    std::shared_ptr<autolink::Node> node)
    : muxer_(std::make_shared<TaskMuxer>()),
      state_hub_(std::make_shared<StateHub>(node, muxer_)),
      navigator_stub_(std::make_shared<clients::NavigatorStub>(node, muxer_)),
      teleop_stub_(std::make_shared<clients::TeleopStub>(node, muxer_)),
      relative_teleop_stub_(
          std::make_shared<clients::RelativeTeleopStub>(node, muxer_)),
      follow_stub_(std::make_shared<clients::FollowStub>(node, muxer_)),
      dock_stub_(std::make_shared<clients::DockStub>(node, muxer_)),
      map_stub_(std::make_shared<clients::MapStub>(node, muxer_)),
      map_service_stub_(
          std::make_shared<clients::MapServiceStub>(node, map_stub_)),
      localization_stub_(
          std::make_shared<clients::LocalizationStub>(node, map_stub_)),
      exploration_stub_(
          std::make_shared<clients::ExplorationStub>(node, muxer_)),
      voice_stub_(std::make_shared<clients::VoiceStub>(
          muxer_, navigator_stub_.get(), follow_stub_.get(), dock_stub_.get(),
          exploration_stub_.get())),
      sensor_stub_(std::make_shared<clients::SensorStub>(node)),
      system_monitor_stub_(
          std::make_shared<clients::SystemMonitorStub>(node, muxer_)) {}

void GrpcBridgeContextInterface::CancelAllTasks() {
    if (navigator_stub_->CheckNavigating()) {
        proto::NavigationCommandRequest cancel;
        cancel.set_command(proto::NAV_CMD_CANCEL);
        navigator_stub_->HandleCommand(cancel, [](const auto&) {});
    }
    teleop_stub_->ResetSession();
    relative_teleop_stub_->Cancel("");
    follow_stub_->CancelActiveSession();
    dock_stub_->CancelActiveSession();
    exploration_stub_->CancelActiveSession();
    muxer_->Clear();
}

void GrpcBridgeContextInterface::EmergencyStop(const bool engage) {
    muxer_->SetEstop(engage);
    if (engage) {
        CancelAllTasks();
        muxer_->SetEstop(true);
    }
}

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
