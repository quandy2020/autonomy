/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/clients/map_service_stub.hpp"

#include "autonomy/bridge/constants.hpp"
#include "autonomy/bridge/grpc/clients/mapping_stub.hpp"
#include "autonomy/bridge/grpc/rpc_status.hpp"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {
namespace {

using StatusCode = ::automsgs::msgs::status_msgs::StatusCode;
namespace mapping_rpc = ::automsgs::rpcs::mapping;

std::string ResolveMapKey(const std::string& map_identifier,
                          const std::string& map_name,
                          const std::string& fallback = {}) {
    if (!map_identifier.empty()) {
        return map_identifier;
    }
    if (!map_name.empty()) {
        return map_name;
    }
    return fallback;
}

}  // namespace

MapServiceStub::MapServiceStub(std::shared_ptr<autolink::Node> node,
                               MappingStub* mapping_stub)
    : mapping_stub_(mapping_stub) {
    live_map_cache_.BindReader(std::move(node), kMapChannel);
}

::automsgs::rpcs::common::Status MapServiceStub::StartMapping(
    const mapping_rpc::StartMappingRequest& request) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (session_state_ == mapping_rpc::MAPPING_STATE_MAPPING) {
        return ErrorStatus(StatusCode::MAPPING_BUSY, "mapping already active");
    }
    session_state_ = mapping_rpc::MAPPING_STATE_MAPPING;
    session_goal_id_ = request.goal_id();
    session_map_name_ = request.map_name();
    if (mapping_stub_) {
        mapping_stub_->HandleStart(request);
    }
    return OkStatus("mapping started");
}

mapping_rpc::FinishMappingResponse MapServiceStub::FinishMapping(
    const mapping_rpc::FinishMappingRequest& request) {
    mapping_rpc::FinishMappingResponse response;
    std::lock_guard<std::mutex> lock(mutex_);
    if (session_state_ != mapping_rpc::MAPPING_STATE_MAPPING) {
        *response.mutable_status() =
            ErrorStatus(StatusCode::INVALID_ARGUMENT, "no active mapping");
        return response;
    }
    const std::string map_identifier = ResolveMapKey(
        session_map_name_, request.goal_id(), "map");
    if (request.persist()) {
        if (auto live = live_map_cache_.GetLatestMessage()) {
            catalog_[map_identifier] = std::move(*live);
            current_map_id_ = map_identifier;
        }
    }
    session_state_ = mapping_rpc::MAPPING_STATE_IDLE;
    response.set_map_identifier(map_identifier);
    *response.mutable_status() = OkStatus("mapping finished");
    return response;
}

::automsgs::rpcs::common::Status MapServiceStub::CancelMapping(
    const mapping_rpc::CancelMappingRequest&) {
    std::lock_guard<std::mutex> lock(mutex_);
    session_state_ = mapping_rpc::MAPPING_STATE_IDLE;
    return OkStatus("mapping cancelled");
}

mapping_rpc::MappingStatus MapServiceStub::GetMappingStatus() const {
    mapping_rpc::MappingStatus status;
    std::lock_guard<std::mutex> lock(mutex_);
    *status.mutable_status() = OkStatus();
    status.set_state(session_state_);
    status.set_goal_id(session_goal_id_);
    status.set_map_name(session_map_name_);
    status.set_map_identifier(current_map_id_);
    return status;
}

mapping_rpc::ListMapsResponse MapServiceStub::ListMaps() const {
    mapping_rpc::ListMapsResponse response;
    std::lock_guard<std::mutex> lock(mutex_);
    *response.mutable_status() = OkStatus();
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
    const std::string map_key =
        ResolveMapKey(request.map_identifier(), request.map_name());
    if (!map_key.empty()) {
        const auto it = catalog_.find(map_key);
        if (it != catalog_.end()) {
            *response.mutable_map() = it->second;
            *response.mutable_status() = OkStatus();
            return response;
        }
    }
    if (auto live = live_map_cache_.GetLatestMessage()) {
        *response.mutable_map() = *live;
        *response.mutable_status() = OkStatus("live map");
        return response;
    }
    *response.mutable_status() =
        ErrorStatus(StatusCode::NOT_FOUND, "map not found");
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
        response.set_map_identifier(
            ResolveMapKey(request.map_identifier(), request.map_name()));
    }
    return response;
}

mapping_rpc::SaveMapResponse MapServiceStub::SaveMap(
    const mapping_rpc::SaveMapRequest& request) {
    mapping_rpc::SaveMapResponse response;
    std::lock_guard<std::mutex> lock(mutex_);
    const std::string map_identifier = ResolveMapKey(
        request.map_identifier(), request.map_name(), "saved_map");
    if (request.has_map()) {
        catalog_[map_identifier] = request.map();
    } else if (auto live = live_map_cache_.GetLatestMessage()) {
        catalog_[map_identifier] = std::move(*live);
    } else {
        *response.mutable_status() =
            ErrorStatus(StatusCode::INVALID_ARGUMENT, "no map buffer");
        return response;
    }
    current_map_id_ = map_identifier;
    response.set_map_identifier(map_identifier);
    *response.mutable_status() = OkStatus("saved");
    if (mapping_stub_) {
        mapping_rpc::StartMappingRequest start;
        start.set_map_name(map_identifier);
        mapping_stub_->HandleStart(start);
    }
    return response;
}

::automsgs::rpcs::common::Status MapServiceStub::DeleteMap(
    const mapping_rpc::DeleteMapRequest& request) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (request.map_identifier().empty() ||
        catalog_.erase(request.map_identifier()) == 0) {
        return ErrorStatus(StatusCode::NOT_FOUND, "map not found");
    }
    if (current_map_id_ == request.map_identifier()) {
        current_map_id_.clear();
    }
    return OkStatus("deleted");
}

::automsgs::rpcs::common::Status MapServiceStub::SetCurrentMap(
    const mapping_rpc::SetCurrentMapRequest& request) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (catalog_.find(request.map_identifier()) == catalog_.end()) {
        return ErrorStatus(StatusCode::NOT_FOUND, "map not found");
    }
    current_map_id_ = request.map_identifier();
    if (mapping_stub_) {
        mapping_rpc::StartMappingRequest start;
        start.set_map_name(current_map_id_);
        mapping_stub_->HandleStart(start);
    }
    return OkStatus("current map set");
}

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
