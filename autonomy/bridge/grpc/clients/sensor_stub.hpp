/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "autolink/node/node.hpp"
#include "autolink/record/record_writer.hpp"
#include "autonomy/bridge/grpc/clients/sensor_sample_traits.hpp"
#include "autonomy/common/macros.hpp"
#include <automsgs/rpcs/sensor.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief Sensor catalogue, sample cache, parameter KV, and record session.
 */
class SensorStub
{
public:
    using RecordStreamCallback = std::function<void(
        const ::automsgs::rpcs::sensor::RecordResponse& response)>;

    AUTONOMY_SMART_PTR_DEFINITIONS(SensorStub)

    explicit SensorStub(std::shared_ptr<autolink::Node> node);
    ~SensorStub();

    ::automsgs::rpcs::sensor::ListSensorsResponse ListSensors() const;
    ::automsgs::rpcs::sensor::GetSampleResponse GetSample(
        const ::automsgs::rpcs::sensor::GetSampleRequest& request) const;

    ::automsgs::rpcs::sensor::GetParametersResponse GetParameters(
        const ::automsgs::rpcs::sensor::GetParametersRequest& request) const;
    ::automsgs::rpcs::sensor::SetParametersResponse SetParameters(
        const ::automsgs::rpcs::sensor::SetParametersRequest& request);
    ::automsgs::rpcs::common::Status SaveParameters(
        const ::automsgs::rpcs::sensor::SaveParametersRequest& request);
    ::automsgs::rpcs::common::Status LoadParameters(
        const ::automsgs::rpcs::sensor::LoadParametersRequest& request);

    bool StartRecord(const ::automsgs::rpcs::sensor::RecordRequest& request,
                     RecordStreamCallback callback);
    ::automsgs::rpcs::common::Status CancelRecord(
        const ::automsgs::rpcs::sensor::CancelRecordRequest& request);
    ::automsgs::rpcs::sensor::GetRecordStatusResponse GetRecordStatus() const;

private:
    struct SensorEntry {
        ::automsgs::rpcs::sensor::SensorInfo info;
        std::unordered_map<std::string, std::string> parameters;
    };

    void InitCatalogue();
    void SubscribeSamples();

    /**
     * @brief Subscribe @p topic and store samples via @ref SampleFieldTraits.
     * @tparam MessageT Sensor message type.
     * @param[in] sensor_id Catalogue id used as cache key.
     * @param[in] topic Autolink channel name.
     */
    template <typename MessageT>
    void SubscribeSample(const std::string& sensor_id,
                         const std::string& topic) {
        if (!node_) {
            return;
        }
        readers_.push_back(node_->CreateReader<MessageT>(
            topic, [this, sensor_id](const std::shared_ptr<MessageT>& message) {
                if (!message) {
                    return;
                }
                std::lock_guard<std::mutex> lock(mutex_);
                SampleFieldTraits<MessageT>::Store(samples_[sensor_id], *message);
            }));
    }

    SensorEntry* FindMutable(const std::string& sensor_id);
    const SensorEntry* Find(const std::string& sensor_id) const;
    std::string ParamsPath() const;
    void EmitRecord(const ::automsgs::rpcs::sensor::RecordResponse& response);
    void FinishRecord(::automsgs::rpcs::sensor::RecordState state,
                      const std::string& message);
    void RecordLoop(::automsgs::rpcs::sensor::RecordRequest request);

    std::shared_ptr<autolink::Node> node_;
    mutable std::mutex mutex_;
    std::vector<SensorEntry> sensors_;
    std::unordered_map<std::string, SensorSampleCache> samples_;

    std::vector<std::shared_ptr<void>> readers_;

    std::atomic<bool> record_cancel_{false};
    std::thread record_thread_;
    RecordStreamCallback record_callback_;
    ::automsgs::rpcs::sensor::RecordResponse record_status_;
    std::unique_ptr<::autolink::record::RecordWriter> record_writer_;
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
