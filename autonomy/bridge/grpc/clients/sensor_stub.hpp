/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file sensor_stub.hpp
 * @brief SensorStub: catalogue / sample cache / params / record.
 *
 * @details
 * Not a GoalChannel client. Maintains an in-memory sensor catalogue,
 * per-sensor GenericSampleCache slots fed by Autolink readers, parameter
 * get/set/save/load helpers, and a WorkScheduler-backed record loop that
 * may borrow RecordResponse frames from ProtoPool.
 *
 * @par Ownership
 * UniquePtr owned by DomainBundle. node_ is shared; scheduler_ is
 * non-owning (Server / Context pool). readers_ keep subscriptions alive
 * for the stub lifetime. record_writer_ is unique-owned during recording.
 *
 * @par Threading
 * Catalogue / samples / params share mutex_. Record loop runs on
 * WorkScheduler; cancel via record_cancel_. CancelRecord is safe vs the
 * record loop; StartRecord must not be called concurrently without
 * external serialization.
 *
 * @par Invariants
 * - Catalogue / samples / params share mutex_.
 * - Record loop runs on WorkScheduler; cancel via record_cancel_.
 * - RecordResponse frames may be borrowed from ProtoPool.
 *
 * @see SampleFieldTraits
 * @see ProtoPool
 * @see rpc_sensor_handlers.hpp
 */

#pragma once

#include <atomic>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "autolink/node/node.hpp"
#include "autolink/record/record_writer.hpp"
#include "autonomy/bridge/grpc/clients/proto_pool.hpp"
#include "autonomy/bridge/grpc/clients/sample_cache.hpp"
#include "autonomy/bridge/grpc/work_scheduler.hpp"
#include "autonomy/common/macros.hpp"
#include <automsgs/rpcs/sensor.pb.h>

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief SensorService facade (catalogue · samples · params · record).
 *
 * @details
 * Handlers call ListSensors / GetSample / parameter RPCs / StartRecord
 * without constructing readers themselves. Sample subscriptions are
 * established in the constructor via SubscribeSamples.
 *
 * @par Ownership
 * node_ shared; scheduler_ non-owning; readers_ keep subscriptions alive
 * for the stub lifetime. UniquePtr of this stub held by DomainBundle.
 *
 * @par Threading
 * Public catalogue/sample/param APIs take mutex_. Record loop uses
 * record_cancel_ / record_future_ with EmitRecord under lock where needed.
 *
 * @warning StartRecord must not be called concurrently without external
 * serialization; CancelRecord is thread-safe vs the record loop.
 *
 * @note Not registered as a CancelRegistry domain hook by default; teardown
 * cancels record in the destructor.
 *
 * @see rpc_sensor_handlers.hpp
 */
class SensorStub
{
public:
    /**
     * @brief Stream sink for RecordResponse progress / terminal frames.
     */
    using RecordStreamCallback = std::function<void(
        const ::automsgs::rpcs::sensor::RecordResponse& response)>;

    /**
     * @brief Shared / weak / unique pointer aliases for SensorStub.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(SensorStub)

    /**
     * @brief Construct catalogue subscribers and bind the work scheduler.
     *
     * @param[in] node      Autolink node for readers / params.
     * @param[in] scheduler Work pool for the record loop (non-null).
     */
    SensorStub(std::shared_ptr<autolink::Node> node, WorkScheduler* scheduler);

    /**
     * @brief Cancel in-flight record and tear down readers.
     */
    ~SensorStub();

    /**
     * @brief List known sensors from the in-memory catalogue.
     *
     * @return ListSensorsResponse snapshot.
     */
    ::automsgs::rpcs::sensor::ListSensorsResponse ListSensors() const;

    /**
     * @brief Fetch the latest cached sample for a sensor / message kind.
     *
     * @param[in] request Sensor id + sample selector.
     * @return            GetSampleResponse with payload or error status.
     */
    ::automsgs::rpcs::sensor::GetSampleResponse GetSample(
        const ::automsgs::rpcs::sensor::GetSampleRequest& request) const;

    /**
     * @brief Read parameters for a sensor id.
     *
     * @param[in] request Sensor id (+ optional keys).
     * @return            GetParametersResponse.
     */
    ::automsgs::rpcs::sensor::GetParametersResponse GetParameters(
        const ::automsgs::rpcs::sensor::GetParametersRequest& request) const;

    /**
     * @brief Update in-memory parameters for a sensor id.
     *
     * @param[in] request Sensor id + key/value updates.
     * @return            SetParametersResponse.
     */
    ::automsgs::rpcs::sensor::SetParametersResponse SetParameters(
        const ::automsgs::rpcs::sensor::SetParametersRequest& request);

    /**
     * @brief Persist parameters to disk.
     *
     * @param[in] request Sensor id scope for save.
     * @return            Status.
     */
    ::automsgs::rpcs::common::Status SaveParameters(
        const ::automsgs::rpcs::sensor::SaveParametersRequest& request);

    /**
     * @brief Load parameters from disk into the catalogue.
     *
     * @param[in] request Sensor id scope for load.
     * @return            Status.
     */
    ::automsgs::rpcs::common::Status LoadParameters(
        const ::automsgs::rpcs::sensor::LoadParametersRequest& request);

    /**
     * @brief Start a record session streaming RecordResponse frames.
     *
     * @param[in] request  Record configuration (path, topics, …).
     * @param[in] callback Stream sink for progress / terminal frames.
     * @return             false if rejected (already recording / invalid).
     */
    bool StartRecord(const ::automsgs::rpcs::sensor::RecordRequest& request,
                     RecordStreamCallback callback);

    /**
     * @brief Request cancellation of the active record loop.
     *
     * @param[in] request Cancel request (goal / session id).
     * @return            Status.
     */
    ::automsgs::rpcs::common::Status CancelRecord(
        const ::automsgs::rpcs::sensor::CancelRecordRequest& request);

    /**
     * @brief Snapshot of the last record status frame.
     *
     * @return GetRecordStatusResponse.
     */
    ::automsgs::rpcs::sensor::GetRecordStatusResponse GetRecordStatus() const;

private:
    /**
     * @brief One catalogue entry: SensorInfo + mutable parameter map.
     */
    struct SensorEntry {
        /**
         * @brief Shared / weak / unique pointer aliases for SensorEntry.
         */
        AUTONOMY_SMART_PTR_DEFINITIONS(SensorEntry)

        /**
         * @brief Static / advertised sensor metadata (id, topics, kinds).
         */
        ::automsgs::rpcs::sensor::SensorInfo info;

        /**
         * @brief Mutable string key/value parameters for this sensor.
         */
        std::unordered_map<std::string, std::string> parameters;
    };

    /**
     * @brief Populate sensors_ with the default onboard catalogue.
     */
    void InitCatalogue();

    /**
     * @brief Bind Autolink readers for every catalogue sample topic.
     */
    void SubscribeSamples();

    /**
     * @brief Subscribe MessageT on @p topic into samples_[sensor_id].
     *
     * @tparam MessageT  Sensor protobuf message type.
     * @param[in] sensor_id Catalogue key for the sample cache.
     * @param[in] topic     Topic name to read.
     */
    template <typename MessageT>
    void SubscribeSample(const std::string& sensor_id,
                         const std::string& topic);

    /**
     * @brief Mutable catalogue lookup by sensor id.
     *
     * @param[in] sensor_id Catalogue key.
     * @return              Pointer into sensors_, or nullptr.
     */
    SensorEntry* FindMutable(const std::string& sensor_id);

    /**
     * @brief Const catalogue lookup by sensor id.
     *
     * @param[in] sensor_id Catalogue key.
     * @return              Pointer into sensors_, or nullptr.
     */
    const SensorEntry* Find(const std::string& sensor_id) const;

    /**
     * @brief Filesystem path used by SaveParameters / LoadParameters.
     *
     * @return Absolute or configured params path string.
     */
    std::string GetParamsPath() const;

    /**
     * @brief Deliver a RecordResponse to record_callback_ (if bound).
     *
     * @param[in] response Frame to emit (also updates record_status_).
     */
    void EmitRecord(const ::automsgs::rpcs::sensor::RecordResponse& response);

    /**
     * @brief Terminal helper: set state, emit, clear writer / callback.
     *
     * @param[in] state   Terminal RecordState.
     * @param[in] message Detail / error text.
     */
    void FinishRecord(::automsgs::rpcs::sensor::RecordState state,
                      const std::string& message);

    /**
     * @brief WorkScheduler body: open writer, pump topics until cancel.
     *
     * @param[in] request RecordRequest copied for the worker thread.
     */
    void RunRecordLoop(::automsgs::rpcs::sensor::RecordRequest request);

    /**
     * @brief Autolink node for readers / parameter I/O.
     */
    std::shared_ptr<autolink::Node> node_{nullptr};

    /**
     * @brief Non-owning WorkScheduler for the record loop (Server-owned).
     */
    WorkScheduler* scheduler_{nullptr};

    /**
     * @brief Protects sensors_ / samples_ / params / record_status_ / callback.
     */
    mutable std::mutex mutex_;

    /**
     * @brief In-memory sensor catalogue entries.
     */
    std::vector<SensorEntry> sensors_;

    /**
     * @brief Per-sensor typed latest-sample caches keyed by sensor id.
     */
    std::unordered_map<std::string, SensorSampleCache> samples_;

    /**
     * @brief Opaque shared Readers keeping sample subscriptions alive.
     *
     * @details Stored as shared_ptr<void> to type-erase heterogeneous
     * MessageT readers created by SubscribeSample.
     */
    std::vector<std::shared_ptr<void>> readers_;

    /**
     * @brief Cooperative cancel flag polled by RunRecordLoop.
     */
    std::atomic<bool> record_cancel_{false};

    /**
     * @brief Future for the scheduled record worker (joinable on cancel).
     */
    std::future<void> record_future_;

    /**
     * @brief Active Record stream sink (cleared on FinishRecord).
     */
    RecordStreamCallback record_callback_;

    /**
     * @brief Last RecordResponse for GetRecordStatus.
     */
    ::automsgs::rpcs::sensor::RecordResponse record_status_;

    /**
     * @brief Active bag / record writer while recording (sole owner).
     */
    std::unique_ptr<::autolink::record::RecordWriter> record_writer_{nullptr};

    /**
     * @brief ProtoPool for borrowed RecordResponse frames during emit.
     */
    ProtoPool<::automsgs::rpcs::sensor::RecordResponse> record_pool_{16};
};

/**
 * @brief Subscribe MessageT on @p topic into samples_[sensor_id].
 *
 * @tparam MessageT  Sensor protobuf message type.
 * @param[in] sensor_id Catalogue key for the sample cache.
 * @param[in] topic     Topic name to read.
 */
template <typename MessageT>
void SensorStub::SubscribeSample(const std::string& sensor_id,
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
            SampleFieldTraits<MessageT>::StoreSample(samples_[sensor_id],
                                                     *message);
        }));
}

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
