/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/clients/map_service_stub.hpp"

#include "autonomy/bridge/constants.hpp"
#include "autonomy/bridge/grpc/rpc_convert.hpp"
#include "autonomy/bridge/proto/external_command_service.pb.h"
#include "autonomy/common/logging.hpp"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {
namespace {

using StatusCode = ::automsgs::msgs::status_msgs::StatusCode;
namespace mapping_rpc = ::automsgs::rpcs::mapping;

}  // namespace

MapServiceStub::MapServiceStub(std::shared_ptr<autolink::Node> node,
                               std::shared_ptr<MapStub> map_stub)
    : map_stub_(std::move(map_stub)) {
    live_map_cache_.BindReader(node, kMapChannel);
}

::automsgs::rpcs::common::Status MapServiceStub::StartMapping(
    const mapping_rpc::StartMappingRequest& request) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (session_state_ == mapping_rpc::MAPPING_STATE_MAPPING) {
        return MakeRpcStatus(StatusCode::MAPPING_BUSY, "mapping already active");
    }
    session_state_ = mapping_rpc::MAPPING_STATE_MAPPING;
    session_goal_id_ = request.goal_id();
    session_map_name_ = request.map_name();
    if (map_stub_) {
        proto::MapCommandRequest bridge;
        bridge.set_command(proto::MAP_CMD_LOAD);
        if (!request.map_name().empty()) {
            bridge.set_map_name(request.map_name());
        }
        map_stub_->HandleCommand(bridge, [](const auto&) {});
    }
    return MakeOkStatus("mapping started");
}

mapping_rpc::FinishMappingResponse MapServiceStub::FinishMapping(
    const mapping_rpc::FinishMappingRequest& request) {
    mapping_rpc::FinishMappingResponse response;
    std::lock_guard<std::mutex> lock(mutex_);
    if (session_state_ != mapping_rpc::MAPPING_STATE_MAPPING) {
        *response.mutable_status() =
            MakeRpcStatus(StatusCode::INVALID_ARGUMENT, "no active mapping");
        return response;
    }
    const std::string map_identifier = session_map_name_.empty()
                               ? (request.goal_id().empty() ? "map"
                                                            : request.goal_id())
                               : session_map_name_;
    if (request.persist()) {
        if (auto live = live_map_cache_.GetLatest()) {
            catalog_[map_identifier] = std::move(*live);
            current_map_id_ = map_identifier;
        }
    }
    session_state_ = mapping_rpc::MAPPING_STATE_IDLE;
    response.set_map_identifier(map_identifier);
    *response.mutable_status() = MakeOkStatus("mapping finished");
    return response;
}

::automsgs::rpcs::common::Status MapServiceStub::CancelMapping(
    const mapping_rpc::CancelMappingRequest&) {
    std::lock_guard<std::mutex> lock(mutex_);
    session_state_ = mapping_rpc::MAPPING_STATE_IDLE;
    return MakeOkStatus("mapping cancelled");
}

mapping_rpc::MappingStatus MapServiceStub::GetMappingStatus() const {
    mapping_rpc::MappingStatus status;
    std::lock_guard<std::mutex> lock(mutex_);
    *status.mutable_status() = MakeOkStatus();
    status.set_state(session_state_);
    status.set_goal_id(session_goal_id_);
    status.set_map_name(session_map_name_);
    status.set_map_identifier(current_map_id_);
    return status;
}

mapping_rpc::ListMapsResponse MapServiceStub::ListMaps() const {
    mapping_rpc::ListMapsResponse response;
    std::lock_guard<std::mutex> lock(mutex_);
    *response.mutable_status() = MakeOkStatus();
    for (const auto& entry : catalog_) {
        auto* summary = response.add_maps();
        summary->set_map_identifier(entry.first);
        summary->set_name(entry.first);
        summary->set_is_current(entry.first == current_map_id_);
        if (entry.second.has_info()) {
            summary->set_resolution(entry.second.info().resolution());
            summary->set_width(entry.second.info().width());
            summary->set_height(entry.second.info().height());
        }
    }
    return response;
}

mapping_rpc::GetMapResponse MapServiceStub::GetMap(
    const mapping_rpc::GetMapRequest& request) const {
    mapping_rpc::GetMapResponse response;
    std::lock_guard<std::mutex> lock(mutex_);
    const std::string map_key = !request.map_identifier().empty()
                                ? request.map_identifier()
                                : request.map_name();
    if (!map_key.empty()) {
        const auto catalog_iterator = catalog_.find(map_key);
        if (catalog_iterator != catalog_.end()) {
            *response.mutable_map() = catalog_iterator->second;
            *response.mutable_status() = MakeOkStatus();
            return response;
        }
    }
    if (auto live = live_map_cache_.GetLatest()) {
        *response.mutable_map() = *live;
        *response.mutable_status() = MakeOkStatus("live map");
        return response;
    }
    *response.mutable_status() =
        MakeRpcStatus(StatusCode::NOT_FOUND, "map not found");
    return response;
}

mapping_rpc::GetMapMetadataResponse MapServiceStub::GetMapMetadata(
    const mapping_rpc::GetMapMetadataRequest& request) const {
    mapping_rpc::GetMapMetadataResponse response;
    mapping_rpc::GetMapRequest get_map_request;
    get_map_request.set_map_identifier(request.map_identifier());
    get_map_request.set_map_name(request.map_name());
    const auto map_response = GetMap(get_map_request);
    *response.mutable_status() = map_response.status();
    if (map_response.has_map() && map_response.map().has_info()) {
        *response.mutable_metadata() = map_response.map().info();
        response.set_map_identifier(!request.map_identifier().empty()
                                        ? request.map_identifier()
                                        : request.map_name());
    }
    return response;
}

mapping_rpc::SaveMapResponse MapServiceStub::SaveMap(
    const mapping_rpc::SaveMapRequest& request) {
    mapping_rpc::SaveMapResponse response;
    std::lock_guard<std::mutex> lock(mutex_);
    const std::string map_identifier = !request.map_identifier().empty()
                               ? request.map_identifier()
                               : (request.map_name().empty() ? "saved_map"
                                                             : request.map_name());
    if (request.has_map()) {
        catalog_[map_identifier] = request.map();
    } else if (auto live = live_map_cache_.GetLatest()) {
        catalog_[map_identifier] = std::move(*live);
    } else {
        *response.mutable_status() =
            MakeRpcStatus(StatusCode::INVALID_ARGUMENT, "no map buffer");
        return response;
    }
    current_map_id_ = map_identifier;
    response.set_map_identifier(map_identifier);
    *response.mutable_status() = MakeOkStatus("saved");
    if (map_stub_) {
        proto::MapCommandRequest bridge;
        bridge.set_command(proto::MAP_CMD_SWITCH);
        bridge.set_map_name(map_identifier);
        map_stub_->HandleCommand(bridge, [](const auto&) {});
    }
    return response;
}

::automsgs::rpcs::common::Status MapServiceStub::DeleteMap(
    const mapping_rpc::DeleteMapRequest& request) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (request.map_identifier().empty() ||
        catalog_.erase(request.map_identifier()) == 0) {
        return MakeRpcStatus(StatusCode::NOT_FOUND, "map not found");
    }
    if (current_map_id_ == request.map_identifier()) {
        current_map_id_.clear();
    }
    return MakeOkStatus("deleted");
}

::automsgs::rpcs::common::Status MapServiceStub::SetCurrentMap(
    const mapping_rpc::SetCurrentMapRequest& request) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (catalog_.find(request.map_identifier()) == catalog_.end()) {
        return MakeRpcStatus(StatusCode::NOT_FOUND, "map not found");
    }
    current_map_id_ = request.map_identifier();
    if (map_stub_) {
        proto::MapCommandRequest bridge;
        bridge.set_command(proto::MAP_CMD_SWITCH);
        bridge.set_map_name(current_map_id_);
        map_stub_->HandleCommand(bridge, [](const auto&) {});
    }
    return MakeOkStatus("current map set");
}

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
