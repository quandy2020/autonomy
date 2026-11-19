/* Copyright 2026 The Openbot Authors. */
/** @file native_observation_source_test.cpp
 * @brief Actual Autolink Reader/Writer lifecycle tests for the production source.
 * @details Uses native nodes, scheduler callbacks and generated automsgs bytes.
 * Localization is an attested test dependency, not a substitute SLAM verdict.
 * Compile and run against actual Autonomy dependencies in SpaceHero; no mocked
 * Reader or local helper pass is accepted as evidence for this executable.
 */
#include <atomic>
#include <chrono>
#include <cstring>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include "autolink/autolink.hpp"
#include "autolink/croutine/croutine.hpp"
#include "autolink/init.hpp"
#include "autonomy/perception/observation/native_observation_source.hpp"
#include "autonomy/perception/observation/native_sensor_messages.hpp"

namespace {
namespace p = autonomy::perception;
namespace m = automsgs::msgs::sensor_msgs;
using namespace std::chrono_literals;

/** @brief Fail on an actual unmet lifecycle or delivery invariant. */
void Require(bool value, const char* reason) {
    if (!value) throw std::runtime_error(reason);
}

/** @brief Observe transport discovery/result progress under a finite deadline. */
template <typename Predicate>
void Await(Predicate predicate, const char* reason) {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!predicate()) {
        Require(std::chrono::steady_clock::now() < deadline, reason);
        std::this_thread::sleep_for(1ms);
    }
}

/** @brief Fixed unique channels, calibration and explicit acquisition policy. */
p::NativeObservationSourceOptions Options(p::SensorMode mode, const std::string& suffix) {
    p::NativeObservationSourceOptions options;
    options.mode = mode;
    const auto prefix = "/autonomy_test/observation/" + suffix;
    options.camera_info_topic = prefix + "/camera_info";
    options.depth_topic = prefix + "/depth";
    options.lidar_topic = prefix + "/points";
    options.depth_calibration_revision = 7;
    options.depth_rectified = true;
    options.decoding.instantaneous_cloud = true;
    return options;
}

/** @brief Give each actual acquisition a matching independent pose proof. */
p::ObservationIngress::LocalizationLookup Localization() {
    return [](p::SensorKind sensor, const std::string& frame, double stamp) {
        p::AcquisitionLocalization proof;
        proof.world_frame = "map"; proof.sensor_frame = frame;
        proof.map_epoch = 4; proof.calibration_revision = 7;
        proof.support = p::Mask(sensor); proof.healthy = true;
        proof.valid_from = stamp - 1; proof.valid_until = stamp + 1;
        return std::optional<p::AcquisitionLocalization>{proof};
    };
}

/** @brief Construct the real source with owned test dependencies. */
std::shared_ptr<p::NativeObservationSource> Source(
    const std::shared_ptr<autolink::Node>& node, p::NativeObservationSourceOptions options,
    p::ObservationIngress::Sink sink = [](auto, double) { return true; },
    p::NativeObservationSource::LossSink loss = [](auto, auto) {}) {
    return std::make_shared<p::NativeObservationSource>(node, std::move(options),
        Localization(), std::move(sink), [] { return 3.4; }, std::move(loss));
}

/** @brief Actual stamped PointCloud2 payload with one valid sensor-frame hit. */
m::PointCloud2 Cloud() {
    m::PointCloud2 cloud;
    cloud.mutable_header()->set_frame_id("lidar");
    cloud.mutable_header()->mutable_stamp()->set_sec(3);
    cloud.mutable_header()->mutable_stamp()->set_nanosec(250000000);
    cloud.set_width(1); cloud.set_height(1); cloud.set_point_step(12); cloud.set_row_step(12);
    unsigned offset = 0;
    for (const char* name : {"x", "y", "z"}) {
        auto* field = cloud.add_fields(); field->set_name(name); field->set_offset(offset);
        field->set_count(1); field->set_datatype(m::PointField::FLOAT32); offset += 4;
    }
    std::string bytes(12, '\0');
    const float x = 1; std::uint32_t bits; std::memcpy(&bits, &x, sizeof(bits));
    for (unsigned i = 0; i < 4; ++i) bytes[i] = static_cast<char>((bits >> (8 * i)) & 255);
    cloud.set_data(bytes);
    return cloud;
}

/** @brief Publish only after the actual transport discovers the native Reader. */
void Publish(const std::shared_ptr<autolink::Writer<m::PointCloud2>>& writer) {
    Require(writer != nullptr, "actual Writer created");
    Await([&] { return writer->HasReader(); }, "native Writer discovered source Reader");
    Require(writer->Write(Cloud()), "native Writer accepted real Protobuf bytes");
}

/** @brief Verify the exact registration and configured pending queue. */
void CheckRegistered(const std::shared_ptr<autolink::Node>& node,
                     const p::NativeObservationSourceOptions& options, bool registered) {
    if (p::Mask(options.mode) & p::Mask(p::SensorKind::kRgbd)) {
        const auto camera = node->GetReader<m::CameraInfo>(options.camera_info_topic);
        const auto depth = node->GetReader<m::Image>(options.depth_topic);
        Require(static_cast<bool>(camera) == registered && static_cast<bool>(depth) == registered,
                "RGB-D Node registrations match real source lifecycle");
        if (registered) Require(camera->PendingQueueSize() == options.pending_queue_size &&
            depth->PendingQueueSize() == options.pending_queue_size, "explicit RGB-D callback queues applied");
    }
    if (p::Mask(options.mode) & p::Mask(p::SensorKind::kLidar)) {
        const auto cloud = node->GetReader<m::PointCloud2>(options.lidar_topic);
        Require(static_cast<bool>(cloud) == registered, "LiDAR Node registration matches real source lifecycle");
        if (registered) Require(cloud->PendingQueueSize() == options.pending_queue_size,
                                "explicit LiDAR callback queue applied");
    }
}

/** @brief All three modes drain and rebuild on the same actual Node/channels. */
void RebuildAllModes() {
    int mode_id = 0;
    for (auto mode : {p::SensorMode::kLidarOnly, p::SensorMode::kRgbdOnly, p::SensorMode::kLidarRgbd}) {
        const auto suffix = "rebuild_" + std::to_string(mode_id++);
        auto node = autolink::CreateNode("observation_" + suffix);
        Require(node != nullptr, "actual Node created");
        auto options = Options(mode, suffix); options.pending_queue_size = 17;
        for (int run = 0; run < 2; ++run) {
            auto source = Source(node, options);
            Require(source->Start(), "native source starts on clean channels");
            CheckRegistered(node, options, true);
            source->Shutdown();
            Require(source->WaitForShutdown(5s), "real Reader cleanup finished");
            CheckRegistered(node, options, false);
            Require(!source->Start(), "retired instance cannot reopen admission");
        }
    }
}

/** @brief Partial startup removes its camera Reader but preserves a preexisting depth Reader. */
void PartialStartupOwnership() {
    auto node = autolink::CreateNode("observation_partial_start");
    Require(node != nullptr, "actual Node created");
    const auto options = Options(p::SensorMode::kRgbdOnly, "partial_start");
    auto existing = node->CreateReader<m::Image>(options.depth_topic, [](const auto&) {});
    Require(existing != nullptr, "foreign depth registration exists before Source");
    auto source = Source(node, options);
    Require(!source->Start(), "partial source startup fails on occupied channel");
    Require(source->WaitForShutdown(5s), "partial startup cleanup drains owned resources");
    Require(!node->GetReader<m::CameraInfo>(options.camera_info_topic), "owned camera registration removed");
    Require(node->GetReader<m::Image>(options.depth_topic) == existing, "foreign registration preserved");
    existing->Shutdown(); Require(node->DeleteReader(options.depth_topic), "test owner removes foreign Reader");
}

/** @brief Shared callback evidence owns resources even if a test exits early. */
struct CallbackEvidence {
    std::weak_ptr<p::NativeObservationSource> source;  ///< Ordinary callback never owns the Source.
    std::shared_ptr<p::NativeObservationSource> final_owner;  ///< Explicit final-release case only.
    std::mutex owner_mutex;  ///< Protects final release against early test-failure cleanup.
    std::promise<void> entered;  ///< Publication that the native sink executed.
    std::future<void> entry = entered.get_future();  ///< External rendezvous with actual scheduler callback.
    std::atomic<bool> native_coroutine{false};  ///< Actual CRoutine, not a manual callback invocation.
    std::atomic<bool> callback_wait_rejected{false};  ///< Coroutine wait guard was exercised.
    std::atomic<bool> packet_valid{false};  ///< Generated wire input passed real decoding/localization.
    /** @brief Release the last owned source outside the evidence lock on any exit path. */
    void ReleaseSource() {
        std::shared_ptr<p::NativeObservationSource> source_to_release;
        { std::lock_guard<std::mutex> lock(owner_mutex); source_to_release.swap(final_owner); }
        source_to_release.reset();
    }
};

/** @brief Break the deliberate final-owner cycle if transport/assertions fail early. */
struct ReleaseSourceOnExit {
    std::shared_ptr<CallbackEvidence> evidence;  ///< Keeps synchronized cleanup evidence alive.
    ~ReleaseSourceOnExit() { evidence->ReleaseSource(); }
};

/** @brief Shutdown invoked from the actual source sink returns before Reader retirement. */
void ShutdownFromCallback() {
    auto node = autolink::CreateNode("observation_callback_shutdown");
    Require(node != nullptr, "actual Node created");
    const auto options = Options(p::SensorMode::kLidarOnly, "callback_shutdown");
    auto evidence = std::make_shared<CallbackEvidence>();
    auto source = Source(node, options, [evidence](auto packet, double now) {
        evidence->packet_valid = packet.Valid(now);
        evidence->native_coroutine = autolink::croutine::CRoutine::GetCurrentRoutine() != nullptr;
        const auto owner = evidence->source.lock();
        if (!owner) return false;
        owner->Shutdown();
        evidence->callback_wait_rejected = !owner->WaitForShutdown(0ms);
        evidence->entered.set_value();
        return true;
    });
    evidence->source = source;
    Require(source->Start(), "callback source started");
    auto writer = node->CreateWriter<m::PointCloud2>(options.lidar_topic); Publish(writer);
    Require(evidence->entry.wait_for(5s) == std::future_status::ready, "native sink completed shutdown request");
    Require(source->WaitForShutdown(5s), "external owner drains after native callback returns");
    Require(evidence->native_coroutine && evidence->packet_valid && evidence->callback_wait_rejected,
            "valid wire packet reaches real coroutine and coroutine wait is refused");
    CheckRegistered(node, options, false);
}

/** @brief Last Source reference can die in a native callback without self-removal. */
void FinalReferenceOnCallback() {
    auto node = autolink::CreateNode("observation_callback_last_reference");
    Require(node != nullptr, "actual Node created");
    const auto options = Options(p::SensorMode::kLidarOnly, "last_reference");
    auto evidence = std::make_shared<CallbackEvidence>();
    ReleaseSourceOnExit release{evidence};
    auto source = Source(node, options, [evidence](auto packet, double now) {
        evidence->packet_valid = packet.Valid(now);
        evidence->native_coroutine = autolink::croutine::CRoutine::GetCurrentRoutine() != nullptr;
        evidence->ReleaseSource();
        evidence->entered.set_value();
        return true;
    });
    const auto completion = source->ShutdownCompletion();
    const std::weak_ptr<p::NativeObservationSource> weak = source;
    { std::lock_guard<std::mutex> lock(evidence->owner_mutex); evidence->final_owner = source; }
    Require(source->Start(), "final-reference source started");
    source.reset();
    auto writer = node->CreateWriter<m::PointCloud2>(options.lidar_topic); Publish(writer);
    Require(evidence->entry.wait_for(5s) == std::future_status::ready, "real callback released last Source reference");
    Require(completion.wait_for(5s) == std::future_status::ready && completion.get(),
            "stable future confirms retirement after callback-thread destruction");
    Require(weak.expired() && evidence->native_coroutine && evidence->packet_valid,
            "last Source destruction occurred inside actual valid-input callback");
    CheckRegistered(node, options, false);
}

/** @brief A fatal loss dependency triggers retirement without an external Shutdown call. */
void FatalLossRetiresReaders() {
    auto node = autolink::CreateNode("observation_callback_loss_failure");
    Require(node != nullptr, "actual Node created");
    const auto options = Options(p::SensorMode::kLidarOnly, "loss_failure");
    auto source = Source(node, options, [](auto, double) { return false; },
        [](auto, auto) { throw std::runtime_error("controlled loss dependency failure"); });
    Require(source->Start(), "fatal-loss source started");
    auto writer = node->CreateWriter<m::PointCloud2>(options.lidar_topic); Publish(writer);
    const auto completion = source->ShutdownCompletion();
    Require(completion.wait_for(5s) == std::future_status::ready && completion.get(),
            "fatal callback schedules and finishes actual Reader retirement");
    Require(!source->Snapshot().running && source->Snapshot().rejected[0] == 1,
            "fatal rejected packet closes input and preserves rejection evidence");
    CheckRegistered(node, options, false);
}

/** @brief Actual rectified calibration for the deferred depth source. */
m::CameraInfo Camera() {
    m::CameraInfo info;
    info.mutable_header()->set_frame_id("depth_optical");
    info.set_width(2); info.set_height(2);
    for (double value : {100., 0., .5, 0., 110., .5, 0., 0., 1.}) info.add_k(value);
    for (double value : {100., 0., .5, 0., 0., 110., .5, 0., 0., 0., 1., 0.}) info.add_p(value);
    return info;
}

/** @brief Four real encoded depth samples at the requested original acquisition stamp. */
m::Image Image(unsigned nanoseconds = 250000000) {
    m::Image image;
    image.mutable_header()->set_frame_id("depth_optical");
    image.mutable_header()->mutable_stamp()->set_sec(3);
    image.mutable_header()->mutable_stamp()->set_nanosec(nanoseconds);
    image.set_width(2); image.set_height(2); image.set_step(4);
    image.set_encoding("16UC1"); image.set_is_bigendian(false);
    std::string bytes;
    for (unsigned i = 0; i < 4; ++i) { bytes.push_back(static_cast<char>(232)); bytes.push_back(3); }
    image.set_data(bytes);
    return image;
}

/** @brief Shared test authority; native transport remains real and localization is explicitly controlled. */
struct DelayedEvidence {
    std::array<std::atomic<bool>, 2> available{{false, false}};  ///< Independent authority readiness per source.
    std::array<std::atomic<unsigned>, 2> calls{{0, 0}};  ///< Actual acquisition lookups made by the Source.
    std::atomic<double> now{3.4};  ///< Source clock used to exercise evidence expiry independently from wall waits.
    std::mutex mutex;  ///< Protects packet and loss evidence from native/association threads.
    std::vector<p::ObservationPacket> packets;  ///< Actually admitted packets, with original stamps and ordinals.
    std::vector<std::pair<p::SensorKind, std::uint64_t>> losses;  ///< Rejected ordinals reported by the actual source.
};

/** @brief Build a real native source whose independently supplied authority may finish later. */
std::shared_ptr<p::NativeObservationSource> DelayedSource(const std::shared_ptr<autolink::Node>& node,
    const p::NativeObservationSourceOptions& options, const std::shared_ptr<DelayedEvidence>& evidence) {
    auto authority = Localization();
    return std::make_shared<p::NativeObservationSource>(node, options,
        [evidence, authority](p::SensorKind sensor, const std::string& frame, double stamp)
            -> std::optional<p::AcquisitionLocalization> {
            const unsigned index = sensor == p::SensorKind::kLidar ? 0 : 1;
            ++evidence->calls[index];
            if (!evidence->available[index]) return {};
            return authority(sensor, frame, stamp);
        },
        [evidence](p::ObservationPacket packet, double now) {
            Require(packet.Valid(now), "deferred packet retains independently valid acquisition evidence");
            std::lock_guard<std::mutex> lock(evidence->mutex);
            evidence->packets.push_back(std::move(packet)); return true;
        }, [evidence] { return evidence->now.load(); },
        [evidence](p::SensorKind sensor, std::uint64_t sequence) {
            std::lock_guard<std::mutex> lock(evidence->mutex); evidence->losses.emplace_back(sensor, sequence);
        });
}

/** @brief Publish actual CameraInfo and wait for the source's own calibration latch. */
void Calibrate(const std::shared_ptr<autolink::Node>& node, const p::NativeObservationSourceOptions& options,
    const std::shared_ptr<p::NativeObservationSource>& source) {
    auto writer = node->CreateWriter<m::CameraInfo>(options.camera_info_topic);
    Require(writer != nullptr, "actual calibration Writer created");
    Await([&] { return writer->HasReader(); }, "calibration Reader discovered");
    Require(writer->Write(Camera()), "actual CameraInfo submitted");
    Await([&] { return source->Snapshot().depth_calibration_ready; }, "source calibrated from actual CameraInfo");
}

/** @brief Delayed RGB-D proofs admit the original two timestamps in FIFO order. */
void DelayedRgbdAcquisition() {
    auto node = autolink::CreateNode("observation_delayed_rgbd");
    auto options = Options(p::SensorMode::kRgbdOnly, "delayed_rgbd");
    options.localization_wait_timeout = 1500ms;
    auto evidence = std::make_shared<DelayedEvidence>();
    auto source = DelayedSource(node, options, evidence);
    Require(source->Start(), "deferred RGB-D source starts"); Calibrate(node, options, source);
    auto writer = node->CreateWriter<m::Image>(options.depth_topic);
    Await([&] { return writer && writer->HasReader(); }, "actual depth Reader discovered");
    Require(writer->Write(Image()), "first delayed depth written");
    Await([&] { return evidence->calls[1] > 0; }, "first original acquisition lookup attempted");
    Require(writer->Write(Image(350000000)), "second delayed depth written");
    Await([&] { return source->Snapshot().awaiting_localization[1] == 2; }, "both wire inputs retained");
    Require(source->Snapshot().accepted[1] == 0, "missing authority cannot become an admitted packet");
    evidence->available[1] = true;
    Await([&] { return source->Snapshot().accepted[1] == 2; }, "delayed original acquisitions admitted");
    {
        std::lock_guard<std::mutex> lock(evidence->mutex);
        Require(evidence->packets.size() == 2 && evidence->losses.empty(), "two real admissions without fabricated loss");
        Require(evidence->packets[0].observation.sequence == 1 && evidence->packets[1].observation.sequence == 2 &&
            evidence->packets[0].observation.stamp == 3.25 && evidence->packets[1].observation.stamp == 3.35,
            "original stamps and receiver ordinals survive deferred admission");
    }
    source->Shutdown(); Require(source->WaitForShutdown(5s), "deferred RGB-D workers and readers retire");
    CheckRegistered(node, options, false);
}

/** @brief A valid first depth acquisition waits for its actual CameraInfo callback instead of becoming source loss. */
void DelayedInitialCalibration() {
    auto node = autolink::CreateNode("observation_initial_calibration");
    auto options = Options(p::SensorMode::kRgbdOnly,"initial_calibration");
    options.localization_wait_timeout = 1500ms;
    auto evidence = std::make_shared<DelayedEvidence>(); evidence->available[1] = true;
    auto source = DelayedSource(node,options,evidence);
    Require(source->Start(),"actual source starts before CameraInfo publication");
    auto writer = node->CreateWriter<m::Image>(options.depth_topic);
    Await([&] { return writer && writer->HasReader(); },"actual first-depth Reader discovered");
    Require(writer->Write(Image()),"first real depth arrives before calibration callback");
    Await([&] { return source->Snapshot().awaiting_localization[1] == 1; },"first depth retained while actual calibration is pending");
    Require(!source->Snapshot().depth_calibration_ready && source->Snapshot().rejected[1] == 0,
        "missing callback does not invent calibration or permanently discard the valid frame");
    Calibrate(node,options,source);
    Await([&] { return source->Snapshot().accepted[1] == 1; },"actual CameraInfo releases original first acquisition");
    Require(source->Snapshot().rejected[1] == 0,"startup ordering introduces no synthetic source gap");
    source->Shutdown(); Require(source->WaitForShutdown(5s),"first-calibration association and native Readers retire");
}
/** @brief Overflow reports the newly rejected ordinal instead of the most recently received one. */
void DelayedOverflowIdentity() {
    auto node = autolink::CreateNode("observation_delayed_overflow");
    auto options = Options(p::SensorMode::kLidarOnly, "delayed_overflow");
    options.localization_wait_timeout = 1500ms; options.localization_queue_size = 1;
    auto evidence = std::make_shared<DelayedEvidence>();
    auto source = DelayedSource(node, options, evidence);
    Require(source->Start(), "bounded deferred source starts");
    auto writer = node->CreateWriter<m::PointCloud2>(options.lidar_topic); Publish(writer);
    Await([&] { return evidence->calls[0] > 0; }, "first cloud waiting for its own proof");
    Publish(writer);
    Await([&] { return source->Snapshot().localization_overflows[0] == 1; }, "new cloud rejected on retained-head overflow");
    evidence->available[0] = true;
    Await([&] { return source->Snapshot().accepted[0] == 1; }, "retained first cloud survives overflow");
    {
        std::lock_guard<std::mutex> lock(evidence->mutex);
        Require(evidence->packets.size() == 1 && evidence->packets.front().observation.sequence == 1 &&
            evidence->losses.size() == 1 && evidence->losses.front().second == 2,
            "deferred first success and second loss keep distinct original ordinals");
    }
    source->Shutdown(); Require(source->WaitForShutdown(5s), "overflow case retires cleanly");
}

/** @brief An available authority cannot extend actual sensor evidence past its expiry. */
void DelayedEvidenceExpiry() {
    auto node = autolink::CreateNode("observation_delayed_expiry");
    auto options = Options(p::SensorMode::kRgbdOnly, "delayed_expiry");
    options.localization_wait_timeout = 80ms;
    auto evidence = std::make_shared<DelayedEvidence>();
    evidence->available[1] = true; evidence->now = 3.9;
    auto source = DelayedSource(node, options, evidence);
    Require(source->Start(), "expiry source starts"); Calibrate(node, options, source);
    auto writer = node->CreateWriter<m::Image>(options.depth_topic);
    Await([&] { return writer && writer->HasReader(); }, "expiry depth Reader discovered");
    Require(writer->Write(Image()), "expired sensor input actually published");
    Await([&] { return source->Snapshot().rejected[1] == 1; }, "expired measured depth rejected");
    Require(source->Snapshot().accepted[1] == 0, "waiting never extends sensor evidence expiry");
    source->Shutdown(); Require(source->WaitForShutdown(5s), "expiry source retires");
}

/** @brief Independent workers prevent delayed RGB-D localization from blocking healthy LiDAR. */
void DelayedMixedSensors() {
    auto node = autolink::CreateNode("observation_delayed_mixed");
    auto options = Options(p::SensorMode::kLidarRgbd, "delayed_mixed");
    options.localization_wait_timeout = 1500ms;
    auto evidence = std::make_shared<DelayedEvidence>(); evidence->available[0] = true;
    auto source = DelayedSource(node, options, evidence);
    Require(source->Start(), "mixed deferred source starts"); Calibrate(node, options, source);
    auto depth = node->CreateWriter<m::Image>(options.depth_topic);
    Await([&] { return depth && depth->HasReader(); }, "mixed depth Reader discovered");
    Require(depth->Write(Image()), "mixed delayed depth published");
    Await([&] { return evidence->calls[1] > 0; }, "RGB-D authority still pending");
    auto cloud = node->CreateWriter<m::PointCloud2>(options.lidar_topic); Publish(cloud);
    Await([&] { return source->Snapshot().accepted[0] == 1; }, "healthy LiDAR admitted while depth waits");
    Require(source->Snapshot().accepted[1] == 0, "LiDAR cannot attest another source's acquisition pose");
    evidence->available[1] = true;
    Await([&] { return source->Snapshot().accepted[1] == 1; }, "depth admitted by its own later authority");
    source->Shutdown(); Require(source->WaitForShutdown(5s), "both sensor workers retire");
}

/** @brief Wall-time deadline, byte budget and cancellation all remain finite without a pose. */
void DelayedBoundsAndCancellation() {
    for (unsigned run = 0; run < 3; ++run) {
        const auto suffix = "delayed_bounds_" + std::to_string(run);
        auto node = autolink::CreateNode("observation_" + suffix);
        auto options = Options(p::SensorMode::kLidarOnly, suffix);
        options.localization_wait_timeout = run == 0 ? 60ms : 5s;
        if (run == 1) options.localization_queue_bytes = 1;
        auto evidence = std::make_shared<DelayedEvidence>();
        auto source = DelayedSource(node, options, evidence);
        Require(source->Start(), "bounded source starts");
        auto writer = node->CreateWriter<m::PointCloud2>(options.lidar_topic); Publish(writer);
        if (run != 2) {
            Await([&] { return source->Snapshot().rejected[0] == 1; }, "bounded unavailable input rejected");
            const auto snapshot = source->Snapshot();
            Require(snapshot.accepted[0] == 0 && (run == 0 ? snapshot.localization_timeouts[0] == 1 :
                snapshot.localization_overflows[0] == 1), "deadline and byte bounds report their actual cause");
        } else Await([&] { return evidence->calls[0] > 0; }, "cancel case has a retained acquisition");
        source->Shutdown(); Require(source->WaitForShutdown(1s), "shutdown interrupts a five-second association wait");
        if (run == 2) Require(source->Snapshot().localization_canceled[0] == 1,
            "retired pending acquisition counted separately from admitted or fused input");
        CheckRegistered(node, options, false);
    }
}

/** @brief Callback-triggered destruction cannot wait for its own association worker. */
void FinalReferenceOnAssociationWorker() {
    auto node = autolink::CreateNode("observation_association_last_reference");
    auto options = Options(p::SensorMode::kLidarOnly, "association_last_reference");
    options.localization_wait_timeout = 1500ms;
    auto evidence = std::make_shared<CallbackEvidence>(); ReleaseSourceOnExit release{evidence};
    auto source = std::make_shared<p::NativeObservationSource>(node, options, Localization(),
        [evidence](auto packet, double now) {
            evidence->packet_valid = packet.Valid(now);
            evidence->native_coroutine = autolink::croutine::CRoutine::GetCurrentRoutine() != nullptr;
            const auto owner = evidence->source.lock();
            if (!owner) return false;
            owner->Shutdown();
            evidence->callback_wait_rejected = !owner->WaitForShutdown(0ms);
            evidence->ReleaseSource();
            evidence->entered.set_value(); return true;
        }, [] { return 3.4; }, [](auto, auto) {});
    evidence->source = source;
    const auto completion = source->ShutdownCompletion();
    const std::weak_ptr<p::NativeObservationSource> weak = source;
    { std::lock_guard<std::mutex> lock(evidence->owner_mutex); evidence->final_owner = source; }
    Require(source->Start(), "association final-reference source started"); source.reset();
    auto writer = node->CreateWriter<m::PointCloud2>(options.lidar_topic); Publish(writer);
    Require(evidence->entry.wait_for(5s) == std::future_status::ready, "association worker released final Source owner");
    Require(completion.wait_for(5s) == std::future_status::ready && completion.get(),
        "cleanup joins association worker after its callback exits");
    Require(weak.expired() && !evidence->native_coroutine && evidence->packet_valid && evidence->callback_wait_rejected,
        "real native wire input uses off-coroutine association with a refused self-wait");
    CheckRegistered(node, options, false);
}
}  // namespace

/** @brief Initialize actual Autolink and run independent native lifecycle cases. */
int main(int argc, char** argv) {
    if (argc < 1 || !autolink::Init(argv[0])) return 2;
    const std::vector<std::pair<const char*, void(*)()>> cases = {
        {"rebuild_all_modes", RebuildAllModes}, {"partial_startup_ownership", PartialStartupOwnership},
        {"shutdown_from_callback", ShutdownFromCallback}, {"final_reference_on_callback", FinalReferenceOnCallback},
        {"fatal_loss_retirement", FatalLossRetiresReaders},
        {"delayed_rgbd_acquisition", DelayedRgbdAcquisition}, {"delayed_overflow_identity", DelayedOverflowIdentity},
        {"delayed_initial_calibration", DelayedInitialCalibration},
        {"delayed_evidence_expiry", DelayedEvidenceExpiry}, {"delayed_mixed_sensors", DelayedMixedSensors},
        {"delayed_bounds_and_cancellation", DelayedBoundsAndCancellation},
        {"final_reference_on_association_worker", FinalReferenceOnAssociationWorker}};
    int failures = 0;
    for (const auto& test : cases) {
        try { test.second(); std::cout << "PASS " << test.first << std::endl; }
        catch (const std::exception& error) { ++failures; std::cerr << "FAIL " << test.first << ": " << error.what() << std::endl; }
    }
    autolink::Clear();
    return failures ? 1 : 0;
}
