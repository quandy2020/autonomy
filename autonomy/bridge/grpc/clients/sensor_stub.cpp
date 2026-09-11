/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/grpc/clients/sensor_stub.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <filesystem>

#include "autonomy/bridge/grpc/rpc_convert.hpp"
#include "autonomy/common/logging.hpp"
#include "nlohmann/json.hpp"
#include <automsgs/msgs/status_msgs/status_msgs.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {
namespace {

namespace filesystem = std::filesystem;
namespace sensor = ::automsgs::rpcs::sensor;
using StatusCode = ::automsgs::msgs::status_msgs::StatusCode;

sensor::SensorInfo MakeInfo(const std::string& id, const std::string& frame,
                            sensor::SensorType type, const std::string& topic,
                            std::initializer_list<const char*> params) {
    sensor::SensorInfo info;
    info.set_sensor_id(id);
    info.set_frame_id(frame);
    info.set_type(type);
    info.set_topic(topic);
    for (const char* p : params) {
        info.add_parameter_names(p);
    }
    return info;
}

std::unordered_map<std::string, std::string> DefaultParams() {
    return {{"enabled", "true"},
            {"frame_rate_hz", "10"},
            {"exposure", "auto"},
            {"range_max", "30"}};
}

}  // namespace

SensorStub::SensorStub(std::shared_ptr<autolink::Node> node)
    : node_(std::move(node)) {
    InitCatalogue();
    SubscribeSamples();
    record_status_.set_state(sensor::RECORD_STATE_IDLE);
    record_status_.set_active(false);
    record_status_.set_final(false);
    *record_status_.mutable_status() = MakeOkStatus();
}

SensorStub::~SensorStub() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        record_cancel_ = true;
    }
    if (record_thread_.joinable()) {
        record_thread_.join();
    }
}

void SensorStub::InitCatalogue() {
    auto add = [this](sensor::SensorInfo info) {
        SensorEntry entry;
        entry.info = std::move(info);
        entry.parameters = DefaultParams();
        sensors_.push_back(std::move(entry));
    };
    add(MakeInfo("camera_front", "camera_front_optical",
                 sensor::SENSOR_TYPE_CAMERA, "/camera/front/image_raw",
                 {"enabled", "frame_rate_hz", "exposure"}));
    add(MakeInfo("camera_front_compressed", "camera_front_optical",
                 sensor::SENSOR_TYPE_CAMERA, "/camera/front/image_raw/compressed",
                 {"enabled", "frame_rate_hz"}));
    add(MakeInfo("depth_front", "camera_front_optical",
                 sensor::SENSOR_TYPE_DEPTH_CAMERA,
                 "/camera/front/depth/image_raw",
                 {"enabled", "frame_rate_hz", "range_max"}));
    add(MakeInfo("lidar_top", "laser", sensor::SENSOR_TYPE_LASER_SCAN, "/scan",
                 {"enabled", "frame_rate_hz", "range_max"}));
    add(MakeInfo("lidar_points", "laser", sensor::SENSOR_TYPE_POINT_CLOUD,
                 "/points", {"enabled", "frame_rate_hz", "range_max"}));
    add(MakeInfo("imu", "imu_link", sensor::SENSOR_TYPE_IMU, "/imu",
                 {"enabled", "frame_rate_hz"}));
}

void SensorStub::SubscribeSamples() {
    if (!node_) {
        return;
    }
    for (const auto& entry : sensors_) {
        const std::string id = entry.info.sensor_id();
        const std::string topic = entry.info.topic();
        switch (entry.info.type()) {
            case sensor::SENSOR_TYPE_CAMERA:
            case sensor::SENSOR_TYPE_DEPTH_CAMERA:
                if (topic.find("compressed") != std::string::npos) {
                    SubscribeSample<::automsgs::msgs::sensor_msgs::CompressedImage>(
                        id, topic);
                } else {
                    SubscribeSample<::automsgs::msgs::sensor_msgs::Image>(id,
                                                                         topic);
                }
                break;
            case sensor::SENSOR_TYPE_LASER_SCAN:
                SubscribeSample<::automsgs::msgs::sensor_msgs::LaserScan>(id,
                                                                          topic);
                break;
            case sensor::SENSOR_TYPE_POINT_CLOUD:
                SubscribeSample<::automsgs::msgs::sensor_msgs::PointCloud2>(
                    id, topic);
                break;
            case sensor::SENSOR_TYPE_IMU:
                SubscribeSample<::automsgs::msgs::sensor_msgs::Imu>(id, topic);
                break;
            default:
                break;
        }
    }
}

SensorStub::SensorEntry* SensorStub::FindMutable(const std::string& sensor_id) {
    for (auto& entry : sensors_) {
        if (entry.info.sensor_id() == sensor_id) {
            return &entry;
        }
    }
    return nullptr;
}

const SensorStub::SensorEntry* SensorStub::Find(
    const std::string& sensor_id) const {
    for (const auto& entry : sensors_) {
        if (entry.info.sensor_id() == sensor_id) {
            return &entry;
        }
    }
    return nullptr;
}

std::string SensorStub::ParamsPath() const {
    return "/tmp/autonomy_bridge/sensor_params.json";
}

sensor::ListSensorsResponse SensorStub::ListSensors() const {
    std::lock_guard<std::mutex> lock(mutex_);
    sensor::ListSensorsResponse response;
    *response.mutable_status() = MakeOkStatus();
    for (const auto& entry : sensors_) {
        *response.add_sensors() = entry.info;
    }
    return response;
}

sensor::GetSampleResponse SensorStub::GetSample(
    const sensor::GetSampleRequest& request) const {
    std::lock_guard<std::mutex> lock(mutex_);
    sensor::GetSampleResponse response;
    const auto* entry = Find(request.sensor_id());
    if (!entry) {
        *response.mutable_status() =
            MakeRpcStatus(StatusCode::SENSOR_NOT_FOUND, "unknown sensor_id");
        return response;
    }
    response.set_sensor_id(entry->info.sensor_id());
    response.set_type(entry->info.type());
    const auto sample_iterator = samples_.find(request.sensor_id());
    if (sample_iterator == samples_.end()) {
        *response.mutable_status() =
            MakeRpcStatus(StatusCode::SENSOR_UNAVAILABLE, "no sample yet");
        return response;
    }
    const auto& cache = sample_iterator->second;
    bool sample_found = false;
    switch (entry->info.type()) {
        case sensor::SENSOR_TYPE_CAMERA:
        case sensor::SENSOR_TYPE_DEPTH_CAMERA:
            if (request.prefer_compressed() && cache.has_compressed_image) {
                *response.mutable_compressed_image() = cache.compressed_image;
                sample_found = true;
            } else if (cache.has_image) {
                *response.mutable_image() = cache.image;
                sample_found = true;
            } else if (cache.has_compressed_image) {
                *response.mutable_compressed_image() = cache.compressed_image;
                sample_found = true;
            }
            break;
        case sensor::SENSOR_TYPE_LASER_SCAN:
            if (cache.has_laser_scan) {
                *response.mutable_laser_scan() = cache.laser_scan;
                sample_found = true;
            }
            break;
        case sensor::SENSOR_TYPE_POINT_CLOUD:
            if (cache.has_point_cloud) {
                *response.mutable_point_cloud() = cache.point_cloud;
                sample_found = true;
            }
            break;
        case sensor::SENSOR_TYPE_IMU:
            if (cache.has_imu) {
                *response.mutable_imu() = cache.imu;
                sample_found = true;
            }
            break;
        default:
            break;
    }
    *response.mutable_status() =
        sample_found ? MakeOkStatus()
           : MakeRpcStatus(StatusCode::SENSOR_UNAVAILABLE, "no sample yet");
    return response;
}

sensor::GetParametersResponse SensorStub::GetParameters(
    const sensor::GetParametersRequest& request) const {
    std::lock_guard<std::mutex> lock(mutex_);
    sensor::GetParametersResponse response;
    const auto* entry = Find(request.sensor_id());
    if (!entry) {
        *response.mutable_status() =
            MakeRpcStatus(StatusCode::SENSOR_NOT_FOUND, "unknown sensor_id");
        return response;
    }
    response.set_sensor_id(entry->info.sensor_id());
    *response.mutable_status() = MakeOkStatus();
    if (request.names().empty()) {
        for (const auto& [name, value] : entry->parameters) {
            auto* p = response.add_parameters();
            p->set_name(name);
            p->set_value(value);
        }
    } else {
        for (const auto& name : request.names()) {
            auto it = entry->parameters.find(name);
            if (it == entry->parameters.end()) {
                continue;
            }
            auto* p = response.add_parameters();
            p->set_name(it->first);
            p->set_value(it->second);
        }
    }
    return response;
}

sensor::SetParametersResponse SensorStub::SetParameters(
    const sensor::SetParametersRequest& request) {
    std::lock_guard<std::mutex> lock(mutex_);
    sensor::SetParametersResponse response;
    auto* entry = FindMutable(request.sensor_id());
    if (!entry) {
        *response.mutable_status() =
            MakeRpcStatus(StatusCode::SENSOR_NOT_FOUND, "unknown sensor_id");
        return response;
    }
    for (const auto& p : request.parameters()) {
        entry->parameters[p.name()] = p.value();
        *response.add_parameters() = p;
    }
    *response.mutable_status() = MakeOkStatus("applied");
    return response;
}

::automsgs::rpcs::common::Status SensorStub::SaveParameters(
    const sensor::SaveParametersRequest& request) {
    std::lock_guard<std::mutex> lock(mutex_);
    nlohmann::json root = nlohmann::json::object();
    for (const auto& entry : sensors_) {
        if (!request.sensor_id().empty() &&
            entry.info.sensor_id() != request.sensor_id()) {
            continue;
        }
        nlohmann::json obj = nlohmann::json::object();
        for (const auto& [k, v] : entry.parameters) {
            obj[k] = v;
        }
        root[entry.info.sensor_id()] = obj;
    }
    if (!request.sensor_id().empty() && !root.contains(request.sensor_id())) {
        return MakeRpcStatus(StatusCode::SENSOR_NOT_FOUND, "unknown sensor_id");
    }
    try {
        const auto path = ParamsPath();
        filesystem::create_directories(filesystem::path(path).parent_path());
        std::ofstream ofs(path);
        ofs << root.dump(2);
        if (!ofs) {
            return MakeRpcStatus(StatusCode::SENSOR_SAVE_FAILED, "write failed");
        }
    } catch (const std::exception& e) {
        return MakeRpcStatus(StatusCode::SENSOR_SAVE_FAILED, e.what());
    }
    return MakeOkStatus("saved");
}

::automsgs::rpcs::common::Status SensorStub::LoadParameters(
    const sensor::LoadParametersRequest& request) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto path = ParamsPath();
    if (!filesystem::exists(path)) {
        return MakeRpcStatus(StatusCode::SENSOR_NOT_FOUND, "no saved parameters");
    }
    try {
        std::ifstream ifs(path);
        nlohmann::json root;
        ifs >> root;
        for (auto& entry : sensors_) {
            if (!request.sensor_id().empty() &&
                entry.info.sensor_id() != request.sensor_id()) {
                continue;
            }
            if (!root.contains(entry.info.sensor_id())) {
                continue;
            }
            for (auto it = root[entry.info.sensor_id()].begin();
                 it != root[entry.info.sensor_id()].end(); ++it) {
                if (it.value().is_string()) {
                    entry.parameters[it.key()] = it.value().get<std::string>();
                } else {
                    entry.parameters[it.key()] = it.value().dump();
                }
            }
        }
    } catch (const std::exception& e) {
        return MakeRpcStatus(StatusCode::SENSOR_LOAD_FAILED, e.what());
    }
    return MakeOkStatus("loaded");
}

void SensorStub::EmitRecord(const sensor::RecordResponse& response) {
    RecordStreamCallback callback;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        record_status_ = response;
        callback = record_callback_;
    }
    if (callback) {
        callback(response);
    }
}

void SensorStub::FinishRecord(sensor::RecordState state,
                              const std::string& message) {
    sensor::RecordResponse response;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        response = record_status_;
        response.set_state(state);
        response.set_active(false);
        response.set_final(true);
        *response.mutable_status() =
            (state == sensor::RECORD_STATE_SUCCEEDED)
                ? MakeOkStatus(message)
                : MakeRpcStatus(state == sensor::RECORD_STATE_CANCELLED
                                    ? StatusCode::SENSOR_CANCELLED
                                    : StatusCode::SENSOR_RECORD_FAILED,
                                message);
        if (record_writer_) {
            record_writer_->Close();
            record_writer_.reset();
        }
        record_status_ = response;
    }
    EmitRecord(response);
}

bool SensorStub::StartRecord(const sensor::RecordRequest& request,
                             RecordStreamCallback callback) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (record_status_.active()) {
            sensor::RecordResponse busy;
            busy.set_state(sensor::RECORD_STATE_FAILED);
            busy.set_active(false);
            busy.set_final(true);
            busy.set_record_id(request.record_id());
            *busy.mutable_status() =
                MakeRpcStatus(StatusCode::SENSOR_BUSY, "record busy");
            if (callback) {
                callback(busy);
            }
            return false;
        }
        if (record_thread_.joinable()) {
            record_thread_.join();
        }
        record_callback_ = std::move(callback);
        record_cancel_ = false;
        record_status_.Clear();
        record_status_.set_state(sensor::RECORD_STATE_RECORDING);
        record_status_.set_active(true);
        record_status_.set_final(false);
        record_status_.set_record_id(
            request.record_id().empty() ? "rec" : request.record_id());
        record_status_.set_uri(
            request.uri().empty() ? "/tmp/autonomy_bridge/record.record"
                                 : request.uri());
        *record_status_.mutable_status() = MakeOkStatus("recording");
    }
    EmitRecord(record_status_);
    record_thread_ =
        std::thread([this, request]() { RecordLoop(request); });
    return true;
}

void SensorStub::RecordLoop(sensor::RecordRequest request) {
    const std::string uri =
        request.uri().empty() ? "/tmp/autonomy_bridge/record.record"
                              : request.uri();
    try {
        filesystem::create_directories(filesystem::path(uri).parent_path());
    } catch (...) {
    }

    auto writer = std::make_unique<::autolink::record::RecordWriter>();
    if (!writer->Open(uri)) {
        FinishRecord(sensor::RECORD_STATE_FAILED, "open record failed");
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        record_writer_ = std::move(writer);
    }

    std::vector<std::string> sensor_ids(request.sensor_ids().begin(),
                                        request.sensor_ids().end());
    if (sensor_ids.empty()) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& entry : sensors_) {
            sensor_ids.push_back(entry.info.sensor_id());
        }
    }

    const auto start = std::chrono::steady_clock::now();
    const float duration_limit =
        request.has_duration_seconds() ? request.duration_seconds() : 0.f;
    const uint64_t bytes_limit =
        request.has_maximum_bytes() ? request.maximum_bytes() : 0ULL;
    uint64_t bytes = 0;
    uint64_t seq = 0;

    while (!record_cancel_.load()) {
        if (duration_limit > 0.f) {
            const float elapsed =
                std::chrono::duration<float>(
                    std::chrono::steady_clock::now() - start)
                    .count();
            if (elapsed >= duration_limit) {
                break;
            }
        }
        if (bytes_limit > 0 && bytes >= bytes_limit) {
            break;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!record_writer_) {
                break;
            }
            for (const auto& id : sensor_ids) {
                const auto* entry = Find(id);
                auto sit = samples_.find(id);
                if (!entry || sit == samples_.end()) {
                    continue;
                }
                const auto& cache = sit->second;
                const std::string& topic = entry->info.topic();
                const auto now_ns =
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count();
                bool wrote = false;
                switch (entry->info.type()) {
                    case sensor::SENSOR_TYPE_CAMERA:
                    case sensor::SENSOR_TYPE_DEPTH_CAMERA:
                        if (cache.has_compressed_image) {
                            wrote = record_writer_->WriteMessage(
                                topic, cache.compressed_image,
                                static_cast<uint64_t>(now_ns));
                            bytes +=
                                static_cast<uint64_t>(cache.compressed_image.ByteSizeLong());
                        } else if (cache.has_image) {
                            wrote = record_writer_->WriteMessage(
                                topic, cache.image,
                                static_cast<uint64_t>(now_ns));
                            bytes +=
                                static_cast<uint64_t>(cache.image.ByteSizeLong());
                        }
                        break;
                    case sensor::SENSOR_TYPE_LASER_SCAN:
                        if (cache.has_laser_scan) {
                            wrote = record_writer_->WriteMessage(
                                topic, cache.laser_scan,
                                static_cast<uint64_t>(now_ns));
                            bytes +=
                                static_cast<uint64_t>(cache.laser_scan.ByteSizeLong());
                        }
                        break;
                    case sensor::SENSOR_TYPE_POINT_CLOUD:
                        if (cache.has_point_cloud) {
                            wrote = record_writer_->WriteMessage(
                                topic, cache.point_cloud,
                                static_cast<uint64_t>(now_ns));
                            bytes +=
                                static_cast<uint64_t>(cache.point_cloud.ByteSizeLong());
                        }
                        break;
                    case sensor::SENSOR_TYPE_IMU:
                        if (cache.has_imu) {
                            wrote = record_writer_->WriteMessage(
                                topic, cache.imu,
                                static_cast<uint64_t>(now_ns));
                            bytes +=
                                static_cast<uint64_t>(cache.imu.ByteSizeLong());
                        }
                        break;
                    default:
                        break;
                }
                (void)wrote;
            }
        }

        sensor::RecordResponse progress;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            progress = record_status_;
        }
        progress.set_state(sensor::RECORD_STATE_RECORDING);
        progress.set_active(true);
        progress.set_final(false);
        progress.set_bytes_transferred(bytes);
        if (bytes_limit > 0) {
            progress.set_progress(
                std::min(1.f, static_cast<float>(bytes) /
                                  static_cast<float>(bytes_limit)));
        } else if (duration_limit > 0.f) {
            const float elapsed =
                std::chrono::duration<float>(
                    std::chrono::steady_clock::now() - start)
                    .count();
            progress.set_progress(std::min(1.f, elapsed / duration_limit));
        }
        *progress.mutable_status() = MakeOkStatus("recording");
        EmitRecord(progress);

        ++seq;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (seq > 0 && duration_limit <= 0.f && bytes_limit == 0 &&
            seq >= 50) {
            // Safety: ~10s default when no limits set.
            break;
        }
    }

    if (record_cancel_.load()) {
        FinishRecord(sensor::RECORD_STATE_CANCELLED, "cancelled");
    } else {
        FinishRecord(sensor::RECORD_STATE_SUCCEEDED, "done");
    }
}

::automsgs::rpcs::common::Status SensorStub::CancelRecord(
    const sensor::CancelRecordRequest& request) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!record_status_.active()) {
        return MakeOkStatus("idle");
    }
    if (!request.record_id().empty() &&
        request.record_id() != record_status_.record_id()) {
        return MakeRpcStatus(StatusCode::SENSOR_NOT_FOUND, "record_id mismatch");
    }
    record_cancel_ = true;
    return MakeOkStatus("cancel requested");
}

sensor::GetRecordStatusResponse SensorStub::GetRecordStatus() const {
    std::lock_guard<std::mutex> lock(mutex_);
    sensor::GetRecordStatusResponse response;
    *response.mutable_record() = record_status_;
    return response;
}

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
