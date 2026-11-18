/* Copyright 2026 The Openbot Authors. */
/** @file exploration_observation_source_test.cpp
 * @brief Actual task-owner and Autolink input-generation lifecycle regression.
 * @details Uses generated sensor messages and real Node/Reader/Writer callbacks.
 * Acquisition localization is controlled; this does not validate a SLAM backend.
 * No map seed is fabricated, so the session factory must remain uncalled here.
 */
#include <atomic>
#include <chrono>
#include <cstring>
#include <future>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "autolink/autolink.hpp"
#include "autolink/croutine/croutine.hpp"
#include "autonomy/perception/observation/native_sensor_messages.hpp"
#include "autonomy/task/exploration/exploration.hpp"

namespace {
namespace t = autonomy::task;
namespace p = autonomy::perception;
namespace m = automsgs::msgs::sensor_msgs;
using namespace std::chrono_literals;
std::atomic<bool> cleanup_failed{false};  ///< Test exit cannot hide unfinished or failed native cleanup.

/** @brief Fail a real state or transport invariant. */
void Require(bool value, const char* reason) {
    if (!value) throw std::runtime_error(reason);
}
/** @brief Await a specific observable transition under a finite wall-clock bound. */
template <typename Predicate>
void Await(Predicate predicate, const char* reason) {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!predicate()) {
        Require(std::chrono::steady_clock::now() < deadline, reason);
        std::this_thread::sleep_for(1ms);
    }
}

/** @brief Hold one actual acquisition callback across accepted task replacement. */
struct AcquisitionGate {
    std::atomic<bool> armed{false};  ///< Only the next lookup enters the controlled gate.
    std::atomic<bool> released{false};  ///< Release is idempotent even during test unwinding.
    std::promise<void> entered_promise, release_promise;  ///< Real callback/owner synchronization.
    std::shared_future<void> entered = entered_promise.get_future().share();  ///< Callback entry evidence.
    std::shared_future<void> release = release_promise.get_future().share();  ///< Owned lookup exit permission.
    /** @brief Block only a deliberately armed real Reader callback. */
    void Enter() {
        if (!armed.exchange(false)) return;
        entered_promise.set_value();
        release.wait();
    }
    /** @brief Always release before task shutdown, including failed assertions. */
    void Release() {
        if (!released.exchange(true)) release_promise.set_value();
    }
};

/** @brief Healthy pose proof matching the actual message stamp and requested frame. */
p::ObservationIngress::LocalizationLookup Localization(std::shared_ptr<AcquisitionGate> gate) {
    return [gate = std::move(gate)](p::SensorKind sensor, const std::string& frame, double stamp) {
        gate->Enter();
        p::AcquisitionLocalization proof;
        proof.world_frame = "map"; proof.sensor_frame = frame;
        proof.map_epoch = 4; proof.calibration_revision = 7;
        proof.support = p::Mask(sensor); proof.healthy = true;
        proof.valid_from = stamp - 1; proof.valid_until = stamp + 1;
        return std::optional<p::AcquisitionLocalization>{proof};
    };
}

/** @brief Encode a real little-endian float field, independently of the decoder. */
void Float(std::string& bytes, unsigned offset, float value) {
    std::uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    for (unsigned i = 0; i < 4; ++i) bytes.at(offset + i) = static_cast<char>((bits >> (8 * i)) & 255);
}
/** @brief One stamped actual sensor-frame point with explicit instantaneous timing. */
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
    std::string bytes(12, '\0'); Float(bytes, 0, 1); cloud.set_data(bytes);
    return cloud;
}
/** @brief Explicit rectified depth calibration with a real optical frame. */
m::CameraInfo Calibration() {
    m::CameraInfo info;
    info.mutable_header()->set_frame_id("depth_optical"); info.set_width(2); info.set_height(2);
    for (double value : {100., 0., .5, 0., 110., .5, 0., 0., 1.}) info.add_k(value);
    for (double value : {100., 0., .5, 0., 0., 110., .5, 0., 0., 0., 1., 0.}) info.add_p(value);
    return info;
}
/** @brief Four valid metric depths in a stamped native Image. */
m::Image Depth() {
    m::Image image;
    image.mutable_header()->set_frame_id("depth_optical");
    image.mutable_header()->mutable_stamp()->set_sec(3);
    image.mutable_header()->mutable_stamp()->set_nanosec(250000000);
    image.set_width(2); image.set_height(2); image.set_step(8); image.set_encoding("32FC1");
    std::string bytes(16, '\0');
    for (unsigned i = 0; i < 16; i += 4) Float(bytes, i, 1);
    image.set_data(bytes); return image;
}
/** @brief Distinct native task commands preserve original accepted identities. */
t::proto::ExplorationGoal Goal(const std::string& id) {
    t::proto::ExplorationGoal goal;
    goal.set_command(t::proto::EXPLORATION_CMD_START);
    goal.mutable_header()->set_task_id(id);
    return goal;
}

/** @brief Own the actual task and Node; release controlled acquisition before joining. */
struct Harness {
    std::shared_ptr<autolink::Node> node;  ///< Real registration owner.
    std::shared_ptr<t::ExplorationTask> task = std::make_shared<t::ExplorationTask>();  ///< Production task owner.
    std::shared_ptr<AcquisitionGate> gate = std::make_shared<AcquisitionGate>();  ///< Controlled pose dependency.
    std::shared_ptr<std::atomic<unsigned>> factory_calls = std::make_shared<std::atomic<unsigned>>(0);  ///< No fabricated map readiness.
    p::NativeObservationSourceOptions options;  ///< Unique native channels and explicit decoding.
    /** @brief Configure real input dependencies without substituting a map/localization implementation. */
    Harness(p::SensorMode mode, const std::string& name, t::ExplorationPacketProcessor processor = {}) {
        node = autolink::CreateNode("exploration_input_" + name);
        Require(node != nullptr, "actual Node created"); task->SetNode(node);
        const auto prefix = "/autonomy_test/exploration_input/" + name;
        options.mode = mode; options.depth_topic = prefix + "/depth";
        options.camera_info_topic = prefix + "/info"; options.lidar_topic = prefix + "/points";
        options.depth_calibration_revision = 7; options.depth_rectified = true;
        options.decoding.instantaneous_cloud = true;
        t::ExplorationTaskBindings bindings;
        bindings.clock = [] { return 3.4; }; bindings.options.mode = mode;
        bindings.process_sensor_packet = std::move(processor);
        bindings.options.maximum_sensor_wait = 10;
        bindings.create_session = [calls = factory_calls](const auto&, const auto&) {
            ++*calls;
            return std::shared_ptr<t::exploration::ExplorationSession>{};
        };
        Require(task->ConfigureSession(std::move(bindings)), "real session dependencies configured");
        t::proto::TaskServerOptions scheduler;
        scheduler.mutable_scheduler()->set_feedback_period_ms(2);
        Require(task->Initialize(scheduler), "actual task owner initialized");
    }
    /** @brief Retire real Readers without blocking an acquisition callback's test release. */
    ~Harness() {
        gate->Release(); task->Shutdown();
        const auto completion = task->ObservationSourceShutdownCompletion();
        if (completion.valid() &&
            (completion.wait_for(5s) != std::future_status::ready || !completion.get()))
            cleanup_failed = true;
    }
    /** @brief Attach the production owner-driven source, using the current exact mode. */
    bool Attach(std::chrono::milliseconds timeout = 5s) {
        return t::ExplorationTask::ConfigureObservationSource(task, options, Localization(gate), timeout);
    }
    /** @brief Await a running Reader bound to exactly this accepted task generation. */
    void Running(std::uint64_t generation) {
        Await([&] {
            const auto s = task->GetObservationSourceSnapshot();
            return !s.faulted && !s.retiring && s.source.running && s.identity &&
                   s.identity->generation == generation;
        }, "real Reader starts for the latest fixed generation");
    }
    /** @brief Check every mode's actual Node registration. */
    void Registered(bool present) {
        if (p::Mask(options.mode) & p::Mask(p::SensorKind::kLidar))
            Require(bool(node->GetReader<m::PointCloud2>(options.lidar_topic)) == present, "actual LiDAR registration");
        if (p::Mask(options.mode) & p::Mask(p::SensorKind::kRgbd)) {
            Require(bool(node->GetReader<m::Image>(options.depth_topic)) == present, "actual depth registration");
            Require(bool(node->GetReader<m::CameraInfo>(options.camera_info_topic)) == present, "actual calibration registration");
        }
    }
    /** @brief Publish one real cloud after transport discovery, optionally waiting for sink admission. */
    void PublishCloud(bool admitted = true) {
        auto writer = node->CreateWriter<m::PointCloud2>(options.lidar_topic);
        Require(writer != nullptr, "real cloud Writer created");
        Await([&] { return writer->HasReader(); }, "cloud Reader discovery");
        const auto before = task->GetObservationSourceSnapshot().source.accepted[0];
        Require(writer->Write(Cloud()), "actual cloud payload published");
        if (admitted) Await([&] {
            return task->GetObservationSourceSnapshot().source.accepted[0] > before;
        }, "cloud admitted by actual task sink");
    }
    /** @brief Wait for actual calibration consumption by observing accepted depth packets. */
    void PublishDepth() {
        auto info = node->CreateWriter<m::CameraInfo>(options.camera_info_topic);
        auto depth = node->CreateWriter<m::Image>(options.depth_topic);
        Require(info && depth, "real RGB-D Writers created");
        Await([&] { return info->HasReader() && depth->HasReader(); }, "RGB-D Reader discovery");
        Require(info->Write(Calibration()), "actual camera calibration published");
        const auto before = task->GetObservationSourceSnapshot().source.accepted[1];
        Await([&] {
            if (task->GetObservationSourceSnapshot().source.accepted[1] > before) return true;
            Require(depth->Write(Depth()), "actual depth payload published");
            return false;
        }, "depth admitted after actual calibration callback");
    }
};

/** @brief All modes seed, rebind, retain pause sequences and close terminal input. */
void ModesAndLifecycle() {
    unsigned mode_id = 0;
    for (auto mode : {p::SensorMode::kLidarOnly, p::SensorMode::kRgbdOnly, p::SensorMode::kLidarRgbd}) {
        Harness h(mode, "mode_" + std::to_string(mode_id++));
        Require(h.Attach(), "production Source configured"); h.Running(0); h.Registered(true);
        if (p::Mask(mode) & p::Mask(p::SensorKind::kLidar)) h.PublishCloud();
        if (p::Mask(mode) & p::Mask(p::SensorKind::kRgbd)) h.PublishDepth();
        Require(!h.factory_calls->load(), "pre-START observations do not fabricate a map or start a Session");
        Require(h.task->SubmitGoal(Goal("first")), "first START accepted"); h.Running(1);
        Require(h.task->Pause(), "pause accepted");
        const auto lidar = h.node->GetReader<m::PointCloud2>(h.options.lidar_topic);
        const auto depth = h.node->GetReader<m::Image>(h.options.depth_topic);
        if (p::Mask(mode) & p::Mask(p::SensorKind::kLidar)) h.PublishCloud();
        if (p::Mask(mode) & p::Mask(p::SensorKind::kRgbd)) h.PublishDepth();
        Require(h.task->Resume(), "resume accepted"); h.Running(1);
        Require(lidar == h.node->GetReader<m::PointCloud2>(h.options.lidar_topic) &&
                depth == h.node->GetReader<m::Image>(h.options.depth_topic), "pause/resume keeps real Reader and ordinals");
        Require(h.task->Cancel(), "terminal cancel accepted");
        Await([&] { const auto s = h.task->GetObservationSourceSnapshot();
                    return !s.identity && s.source.retirement.finished && s.source.retirement.succeeded; },
              "terminal input actually drained");
        h.Registered(false);
        Require(h.task->SubmitGoal(Goal("second")), "new START after terminal accepted"); h.Running(2);
        Require(!h.factory_calls->load(), "missing actual map remains a real readiness requirement");
        h.task->Shutdown(); const auto cleanup = h.task->ObservationSourceShutdownCompletion();
        Require(cleanup.valid() && cleanup.wait_for(5s) == std::future_status::ready && cleanup.get(),
                "shutdown future confirms actual Reader cleanup"); h.Registered(false);
    }
}

/** @brief Pending teardown preserves old registration until successful cleanup, then selects the latest START. */
void LatestGenerationAfterRetirement() {
    Harness h(p::SensorMode::kLidarOnly, "replacement");
    Require(h.Attach(), "native source attached"); h.Running(0);
    const auto old_reader = h.node->GetReader<m::PointCloud2>(h.options.lidar_topic);
    h.gate->armed = true; h.PublishCloud(false);
    Require(h.gate->entered.wait_for(5s) == std::future_status::ready, "actual old acquisition entered lookup");
    Require(h.task->SubmitGoal(Goal("superseded")), "replacement START accepted");
    Await([&] { return h.task->GetObservationSourceSnapshot().retiring; }, "old source enters real retirement");
    Require(h.task->SubmitGoal(Goal("latest")), "latest START replaces waiting generation");
    Require(h.node->GetReader<m::PointCloud2>(h.options.lidar_topic) == old_reader,
            "old registration reserved while actual callback is unfinished");
    h.gate->Release(); h.Running(2);
    Require(h.node->GetReader<m::PointCloud2>(h.options.lidar_topic) != old_reader,
            "latest source is a newly registered Reader after real cleanup");
    Require(h.task->GetObservationSourceSnapshot().source.accepted[0] == 0,
            "late old callback never transfers observations to the latest source");
    h.PublishCloud();
    Require(h.task->GetObservationSourceSnapshot().source.accepted[0] == 1,
            "latest generation starts its own actual acquisition sequence");
}

/** @brief Retirement timeout is observable and permanently blocks automatic channel reuse. */
void RetirementTimeout() {
    Harness h(p::SensorMode::kLidarOnly, "timeout");
    Require(h.Attach(40ms), "short explicit retirement deadline installed"); h.Running(0);
    const auto old_reader = h.node->GetReader<m::PointCloud2>(h.options.lidar_topic);
    h.gate->armed = true; h.PublishCloud(false);
    Require(h.gate->entered.wait_for(5s) == std::future_status::ready, "blocked actual callback entered");
    Require(h.task->SubmitGoal(Goal("timed_out")), "new START accepted before cleanup timeout");
    Await([&] { return h.task->GetObservationSourceSnapshot().faulted; }, "real cleanup timeout reported");
    const auto failed = h.task->GetObservationSourceSnapshot();
    Require(failed.retiring && !failed.source.retirement.finished && !failed.error.empty(),
            "timeout does not claim real cleanup completed");
    Require(h.node->GetReader<m::PointCloud2>(h.options.lidar_topic) == old_reader,
            "timed-out registration is not replaced");
    Require(!h.task->SubmitGoal(Goal("unsafe_retry")), "latched cleanup fault rejects new START");
    h.gate->Release();
    Await([&] { const auto s = h.task->GetObservationSourceSnapshot();
                return s.source.retirement.finished && s.source.retirement.succeeded && !s.identity; },
          "later actual cleanup remains separately observable");
    Require(h.task->GetObservationSourceSnapshot().faulted && !h.Attach(),
            "eventual cleanup success does not silently clear input failure"); h.Registered(false);
}

/** @brief Startup collision preserves the foreign Reader and reports a real input fault. */
void StartupCollisionAndModeCheck() {
    Harness h(p::SensorMode::kLidarOnly, "collision");
    auto different = h.options; different.mode = p::SensorMode::kRgbdOnly;
    Require(!t::ExplorationTask::ConfigureObservationSource(h.task, different, Localization(h.gate)),
            "source/session mode mismatch rejected before subscription");
    auto foreign = h.node->CreateReader<m::PointCloud2>(h.options.lidar_topic, [](const auto&) {});
    Require(foreign != nullptr && h.Attach(), "foreign registration and configured real input exist");
    Await([&] { return h.task->GetObservationSourceSnapshot().faulted; }, "actual duplicate Reader startup fails");
    Require(h.node->GetReader<m::PointCloud2>(h.options.lidar_topic) == foreign,
            "startup failure preserves another owner's native registration");
    Require(!h.task->SubmitGoal(Goal("collision_retry")), "startup fault cannot accept navigation intent");
    foreign->Shutdown(); Require(h.node->DeleteReader(h.options.lidar_topic), "test owner removes its own Reader");
}
/** @brief Native mapping callbacks run on the task owner and retain the native input revision.
 * The controlled map below checks callback ownership only; its synchronization
 * flag stays false and it is never claimed as real fusion evidence.
 */
void NativeProcessorOwnership() {
    auto calls = std::make_shared<std::atomic<unsigned>>(0);
    auto affinity = std::make_shared<std::atomic<bool>>(false);
    const auto caller = std::this_thread::get_id();
    t::ExplorationPacketProcessor processor = [calls,affinity,caller](std::uint64_t generation,
        const p::ObservationPacket& packet,double now) -> std::optional<t::exploration::MapReadView> {
        *affinity = generation == 0 && packet.Valid(now) && std::this_thread::get_id() != caller &&
            autolink::croutine::CRoutine::GetCurrentRoutine() == nullptr;
        t::exploration::MapReadView map;
        map.grid = std::make_shared<opt::navigation::GridMap>(8,8,1,-2,-2,"map");
        map.map_epoch = packet.localization.map_epoch; map.geometry_revision = 1;
        map.frame_id = packet.localization.world_frame; map.source_stamp = packet.observation.stamp;
        ++*calls; return map;
    };
    Harness h(p::SensorMode::kLidarOnly,"processor_owner",processor);
    Require(h.Attach(),"real source installed with its immutable mapping callback"); h.Running(0);
    h.PublishCloud(); Await([&] { return calls->load() == 1; },"actual native packet reached the mapping owner");
    Require(affinity->load() && !h.factory_calls->load(),"bootstrap mapper runs on owner without starting a Session");
    const auto identity = h.task->SensorInputIdentity(); Require(identity.has_value(),"actual source identity available");
    Require(h.Attach(),"new native source revision accepted while idle"); h.Running(0);
    const auto message = Cloud(); const auto cloud = p::NativePointCloud(message); Require(cloud.has_value(),"real old-source bytes decoded");
    p::ObservationPacket packet;
    p::ObservationIngress ingress(p::SensorMode::kLidarOnly,h.options.decoding,Localization(h.gate),
        [&](auto value,double) { packet = std::move(value); return true; },[] { return 3.4; });
    Require(ingress.PushCloud(*cloud,1) == p::ObservationIngressResult::kAccepted,"old native packet retains valid actual acquisition evidence");
    Require(!h.task->PushNativeSensorPacket(*identity,std::move(packet),3.4,processor,1),
        "valid old-revision input cannot invoke a replacement platform processor");
    Require(calls->load() == 1,"old revision produced no mapper operation");
}
}  // namespace

/** @brief Run against the actual native scheduler and generated message targets. */
int main(int argc, char** argv) {
    if (!autolink::Init(argv[0])) return 2;
    try {
        ModesAndLifecycle(); std::cout << "PASS all_modes_task_input_lifecycle\n";
        LatestGenerationAfterRetirement(); std::cout << "PASS latest_generation_after_actual_retirement\n";
        RetirementTimeout(); std::cout << "PASS retirement_timeout_preserves_fault_and_channels\n";
        StartupCollisionAndModeCheck(); std::cout << "PASS startup_collision_and_exact_mode\n";
        NativeProcessorOwnership(); std::cout << "PASS native_processor_owner_and_revision\n";
        Require(!cleanup_failed.load(), "every fixture completed actual Reader shutdown before native runtime cleanup");
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n'; autolink::Clear(); return 1;
    }
    autolink::Clear(); return 0;
}
