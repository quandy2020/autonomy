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
}  // namespace

/** @brief Initialize actual Autolink and run independent native lifecycle cases. */
int main(int argc, char** argv) {
    if (argc < 1 || !autolink::Init(argv[0])) return 2;
    const std::vector<std::pair<const char*, void(*)()>> cases = {
        {"rebuild_all_modes", RebuildAllModes}, {"partial_startup_ownership", PartialStartupOwnership},
        {"shutdown_from_callback", ShutdownFromCallback}, {"final_reference_on_callback", FinalReferenceOnCallback},
        {"fatal_loss_retirement", FatalLossRetiresReaders}};
    int failures = 0;
    for (const auto& test : cases) {
        try { test.second(); std::cout << "PASS " << test.first << std::endl; }
        catch (const std::exception& error) { ++failures; std::cerr << "FAIL " << test.first << ": " << error.what() << std::endl; }
    }
    autolink::Clear();
    return failures ? 1 : 0;
}
