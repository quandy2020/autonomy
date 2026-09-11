/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/clients/map_stub.hpp"

#include "autonomy/bridge/grpc/clients/stub_util.hpp"
#include "autonomy/bridge/grpc/rpc_convert.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {
namespace {

namespace task_proto = ::autonomy::task::proto;

bool CheckMapTerminalStatus(task_proto::MapStatus status) {
    return status == task_proto::MAP_STATUS_SUCCEEDED ||
           status == task_proto::MAP_STATUS_FAILED;
}

}  // namespace

std::function<std::string()> MapTraits::map_name_fn;

MapTraits::Goal MapTraits::ConvertToGoal(const Request& request) {
    return ToTaskMappingGoal(request);
}

MapTraits::Response MapTraits::ConvertFromFeedback(const Feedback& feedback,
                                                   const Request& last) {
    Response response;
    response.set_status(static_cast<proto::MapStatus>(feedback.status()));
    response.set_current_map_name(feedback.current_map_name());
    FillCommandAck(response, kTaskType, last,
                   feedback.status() != task_proto::MAP_STATUS_FAILED,
                   CheckMapTerminalStatus(feedback.status()),
                   feedback.status() == task_proto::MAP_STATUS_SUCCEEDED
                       ? proto::TASK_STATUS_SUCCEEDED
                       : (feedback.status() == task_proto::MAP_STATUS_FAILED
                              ? proto::TASK_STATUS_FAILED
                              : proto::TASK_STATUS_RUNNING));
    return response;
}

MapTraits::Response MapTraits::MakeResponse(const Request& request,
                                            bool success, bool final,
                                            const std::string& message) {
    Response response;
    response.set_status(success ? (final ? proto::MAP_STATUS_SUCCEEDED
                                         : proto::MAP_STATUS_LOADING)
                                : proto::MAP_STATUS_FAILED);
    if (map_name_fn) {
        response.set_current_map_name(map_name_fn());
    }
    FillCommandAck(response, kTaskType, request, success, final,
                   success ? (final ? proto::TASK_STATUS_SUCCEEDED
                                    : proto::TASK_STATUS_RUNNING)
                           : proto::TASK_STATUS_FAILED,
                   message);
    return response;
}

bool MapTraits::CheckTerminalStatus(const Feedback& feedback) {
    return CheckMapTerminalStatus(feedback.status());
}

MapStub::MapStub(std::shared_ptr<autolink::Node> node,
                 std::shared_ptr<TaskMuxer> muxer)
    : channel_(std::move(node), std::move(muxer)) {
    MapTraits::map_name_fn = [this]() { return GetCurrentMapName(); };
    channel_.SetFeedbackHook([this](const task_proto::MappingFeedback& feedback) {
        std::lock_guard<std::mutex> lock(map_name_mutex_);
        current_map_name_ = feedback.current_map_name();
    });
}

MapStub::~MapStub() {
    if (MapTraits::map_name_fn) {
        MapTraits::map_name_fn = nullptr;
    }
}

std::string MapStub::GetCurrentMapName() const {
    std::lock_guard<std::mutex> lock(map_name_mutex_);
    return current_map_name_;
}

bool MapStub::HandleCommand(const proto::MapCommandRequest& request,
                            StreamCallback stream_callback) {
    if (!stream_callback) {
        return false;
    }
    if (request.has_map_name()) {
        std::lock_guard<std::mutex> lock(map_name_mutex_);
        current_map_name_ = request.map_name();
    }
    if (!channel_.CheckWriterReady()) {
        stream_callback(MapTraits::MakeResponse(
            request, false, true, "map goal writer unavailable"));
        return false;
    }
    if (channel_.CheckEstopActive()) {
        stream_callback(MapTraits::MakeResponse(request, false, true,
                                                "emergency stop active"));
        return false;
    }

    channel_.BindStream(request, stream_callback);
    if (!channel_.WriteGoal(MapTraits::ConvertToGoal(request))) {
        stream_callback(MapTraits::MakeResponse(request, false, true,
                                                "failed to publish map goal"));
        return false;
    }
    channel_.SetSessionActive(true);
    stream_callback(MapTraits::MakeResponse(request, true, false, ""));
    return true;
}

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
