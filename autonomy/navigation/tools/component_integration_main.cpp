/* Copyright 2026 The Navigation Authors (navigation) */
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "autolink/action/client.hpp"
#include "autolink/autolink.hpp"
#include "autolink/message/raw_message.hpp"
#include "autolink/proto/dag_conf.pb.h"
#include "autolink/task/task.hpp"
#include "autolink/timer/timer.hpp"
#include "automsgs/msgs/status_msgs/status_msgs.pb.h"
#include "automsgs/msgs/time_utils.hpp"
#include "google/protobuf/text_format.h"
#include "navigation/behavior/behavior_component.hpp"
#include "navigation/common/configuration_loader.hpp"
#include "navigation/guardian/guardian_component.hpp"
#include "navigation/navigation_component.hpp"
#include "navigation/perception/observation_builder_component.hpp"
#include "navigation/proto/config.pb.h"
#include "navigation/safety/safety_component.hpp"

using namespace std::chrono_literals;
namespace msgs = automsgs::msgs;
using Action = automsgs::actions::NavigateToPoseAction;

void Require(bool ok, const std::string& message) {
    if (!ok)
        throw std::runtime_error(message);
}

void CheckTimerLifecycle() {
    std::atomic<int> entered{0}, calls{0};
    std::promise<void> release;
    const auto gate = release.get_future().share();
    struct ReleaseGate {
        std::promise<void>& promise;
        bool released{false};
        void Release() {
            if (!released) {
                promise.set_value();
                released = true;
            }
        }
        ~ReleaseGate() {
            Release();
        }
    } guard{release};
    std::array<std::future<void>, 4> blocked;
    for (auto& future : blocked)
        future = autolink::Async([&entered, gate] {
            ++entered;
            gate.wait();
        });
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (entered < 4 && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(1ms);
    Require(entered == 4, "timer regression could not gate dispatchers");
    autolink::Timer once(
        10, [&calls] { ++calls; }, true);
    once.Start();
    std::this_thread::sleep_for(50ms);
    once.Stop();
    guard.Release();
    for (auto& future : blocked)
        future.get();
    auto drained = autolink::Async([] {});
    Require(drained.wait_for(2s) == std::future_status::ready,
            "timer queue did not drain");
    drained.get();
    Require(calls == 0, "queued timer ran after Stop");
    autolink::Timer periodic(
        10, [&calls] { ++calls; }, false);
    periodic.Start();
    const auto periodic_deadline = std::chrono::steady_clock::now() + 2s;
    while (calls < 3 && std::chrono::steady_clock::now() < periodic_deadline)
        std::this_thread::sleep_for(1ms);
    periodic.Stop();
    const int stopped_calls = calls;
    std::this_thread::sleep_for(50ms);
    Require(stopped_calls >= 3 && calls == stopped_calls,
            "periodic timer continued after Stop");
    std::cout << "PASS queued and periodic timer shutdown\n";
}

void CheckCancelWire() {
    static_assert(autolink::message::HasSerializer<
                  autolink::action::internal::SendGoalRequest<Action>>::value);
    static_assert(autolink::message::HasSerializer<
                  autolink::action::internal::SendGoalResponse>::value);
    static_assert(autolink::message::HasSerializer<
                  autolink::action::internal::CancelGoalRequest>::value);
    static_assert(autolink::message::HasSerializer<
                  autolink::action::internal::CancelGoalResponse>::value);
    static_assert(autolink::message::HasSerializer<
                  autolink::action::internal::GetResultRequest>::value);
    static_assert(
        autolink::message::HasSerializer<
            autolink::action::internal::GetResultResponse<Action>>::value);
    struct BytesAction {
        using Goal = autolink::message::RawMessage;
        using Result = autolink::message::RawMessage;
    };
    Require(autolink::message::MessageType<
                    autolink::action::internal::SendGoalRequest<Action>>() ==
                    autolink::message::MessageType<
                        autolink::action::internal::SendGoalRequest<
                            BytesAction>>() &&
                autolink::message::MessageType<
                    autolink::action::internal::GetResultResponse<Action>>() ==
                    autolink::message::MessageType<
                        autolink::action::internal::GetResultResponse<
                            BytesAction>>(),
            "Python bytes and C++ actions advertise different wire types");
    autolink::action::internal::CancelGoalResponse sent;
    autolink::action::GoalUUID first{}, second{};
    first[0] = 1;
    second[0] = 2;
    sent.goals_canceling = 2;
    sent.canceling_goal_ids = {first, second};
    std::string bytes;
    Require(sent.SerializeToString(&bytes), "serialize cancel response");
    std::vector<char> array(bytes.size());
    Require(sent.SerializeToArray(array.data(), array.size()) &&
                !sent.SerializeToArray(array.data(), array.size() - 1) &&
                !sent.ParseFromArray(nullptr, 1),
            "RPC envelope array bounds failed");
    autolink::action::internal::CancelGoalResponse received;
    Require(received.ParseFromString(bytes) && received.goals_canceling == 2 &&
                received.canceling_goal_ids == sent.canceling_goal_ids,
            "cancel response lost goal IDs");
    sent.goals_canceling = 0;
    sent.canceling_goal_ids.clear();
    Require(sent.SerializeToString(&bytes) && received.ParseFromString(bytes) &&
                received.goals_canceling == 0 &&
                received.canceling_goal_ids.empty(),
            "rejected cancel retained old IDs");
    std::cout << "PASS cancellation response serialization\n";
}

class Fixture
{
public:
    explicit Fixture(const std::string& root) : root_(root) {
        const std::string fixture = "/tmp/navigation_component_fixture";
        std::filesystem::create_directories(fixture);
        navigation::config::PlatformProfile profile;
        Require(
            navigation::common::LoadTextProto(
                root_ + "/conf/platform/differential_drive.pb.txt", &profile)
                .Ok(),
            "load fixture platform");
        profile.set_footprint_radius(0.3);
        const auto write = [](const std::string& path,
                              const google::protobuf::Message& message) {
            std::string text;
            Require(google::protobuf::TextFormat::PrintToString(message, &text),
                    "serialize fixture config");
            std::ofstream output(path);
            output << text;
            Require(output.good(), "write fixture config");
        };
        const std::string platform = fixture + "/platform.pb.txt";
        write(platform, profile);
        navigation::config::NavigationConfig nav;
        navigation::config::SafetyConfig safety;
        Require(navigation::common::LoadTextProto(
                    root_ + "/conf/navigation.pb.txt", &nav)
                        .Ok() &&
                    navigation::common::LoadTextProto(
                        root_ + "/conf/safety.pb.txt", &safety)
                        .Ok(),
                "load fixture component configs");
        nav.set_platform_profile(platform);
        safety.set_platform_profile(platform);
        navigation_fixture_ = fixture + "/navigation.pb.txt";
        safety_fixture_ = fixture + "/safety.pb.txt";
        write(navigation_fixture_, nav);
        write(safety_fixture_, safety);
        node_ = autolink::CreateNode("navigation_integration_fixture");
        depth_ = node_->CreateWriter<msgs::sensor_msgs::Image>(
            "/camera/depth/image_raw");
        camera_ = node_->CreateWriter<msgs::sensor_msgs::CameraInfo>(
            "/camera/camera_info");
        odom_ = node_->CreateWriter<msgs::nav_msgs::Odometry>("/odom");
        goal_ = node_->CreateWriter<navigation::GoalSpec>("/navigation/goal");
        command_ = node_->CreateReader<msgs::geometry_msgs::TwistStamped>(
            "/cmd_vel", [&](const auto& command) {
                const std::lock_guard<std::mutex> lock(mutex_);
                command_value_ = *command;
                ++commands_;
                if (std::abs(command->twist().linear().x()) < 1e-5 &&
                    std::abs(command->twist().angular().z()) < 1e-5)
                    ++zero_commands_;
            });
        raw_command_ = node_->CreateReader<msgs::geometry_msgs::Twist>(
            "/navigation/cmd_vel_raw", [&](const auto& command) {
                const std::lock_guard<std::mutex> lock(mutex_);
                raw_command_value_ = *command;
                ++raw_commands_;
            });
        observation_ =
            node_->CreateReader<navigation::observation::Observation>(
                "/navigation/observation", [&](const auto& value) {
                    validity_ = value->validity();
                    ++observations_;
                });
        degradation_ = node_->CreateReader<navigation::lifecycle::Degradation>(
            "/guardian/degradation", [&](const auto& value) {
                if (value->from() == value->to() || value->reason().empty() ||
                    value->stamp_ns() <= 0 || value->sequence() == 0)
                    ++invalid_degradations_;
                ++degradations_;
                mode_ = value->to();
            });
        for (std::size_t i = 0; i < 3; ++i) {
            const std::array<std::string, 3> modules{"navigation", "safety",
                                                     "behavior"};
            heartbeat_readers_[i] =
                node_->CreateReader<navigation::lifecycle::Heartbeat>(
                    "/" + modules[i] + "/heartbeat",
                    [this, i](const auto&) { ++heartbeats_[i]; });
        }
        for (const auto& dag : {"navigation", "safety", "guardian"})
            LoadDag(dag);
        client_ = std::make_shared<autolink::action::Client<Action>>(
            node_, "/navigate_to_pose");
    }

    ~Fixture() {
        for (auto& [name, component] : components_)
            component->Shutdown();
    }

    void LoadDag(const std::string& name, const std::string& only = "") {
        autolink::proto::DagConfig dag;
        Require(navigation::common::LoadTextProto(
                    root_ + "/dag/" + name + ".dag", &dag)
                    .Ok(),
                "parse DAG " + name);
        for (const auto& module : dag.module_config()) {
            for (const auto& item : module.components()) {
                if (!only.empty() && item.config().name() != only)
                    continue;
                std::shared_ptr<autolink::ComponentBase> component;
                if (item.class_name() == "navigation::NavigationComponent")
                    component =
                        std::make_shared<navigation::NavigationComponent>();
                else if (item.class_name() ==
                         "navigation::perception::ObservationBuilderComponent")
                    component = std::make_shared<
                        navigation::perception::ObservationBuilderComponent>();
                Require(component != nullptr,
                        "unknown DAG component " + item.class_name());
                auto config = item.config();
                config.set_config_file_path(
                    root_ + "/conf/" +
                    std::filesystem::path(config.config_file_path())
                        .filename()
                        .string());
                if (item.class_name() == "navigation::NavigationComponent")
                    config.set_config_file_path(navigation_fixture_);
                Require(component->Initialize(config),
                        "initialize " + item.class_name());
                components_[config.name()] = component;
            }
            for (const auto& item : module.timer_components()) {
                if (!only.empty() && item.config().name() != only)
                    continue;
                std::shared_ptr<autolink::ComponentBase> component;
                if (item.class_name() == "navigation::safety::SafetyComponent")
                    component =
                        std::make_shared<navigation::safety::SafetyComponent>();
                else if (item.class_name() ==
                         "navigation::guardian::GuardianComponent")
                    component = std::make_shared<
                        navigation::guardian::GuardianComponent>();
                else if (item.class_name() ==
                         "navigation::behavior::BehaviorComponent")
                    component = std::make_shared<
                        navigation::behavior::BehaviorComponent>();
                Require(component != nullptr,
                        "unknown DAG timer " + item.class_name());
                auto config = item.config();
                config.set_config_file_path(
                    root_ + "/conf/" +
                    std::filesystem::path(config.config_file_path())
                        .filename()
                        .string());
                if (item.class_name() == "navigation::safety::SafetyComponent")
                    config.set_config_file_path(safety_fixture_);
                else if (item.class_name() ==
                         "navigation::behavior::BehaviorComponent")
                    config.set_config_file_path(navigation_fixture_);
                Require(component->Initialize(config),
                        "initialize " + item.class_name());
                components_[config.name()] = component;
            }
        }
    }

    void Advance(int milliseconds, bool send_depth = true,
                 bool optional_sensors = true, float distance = 8.0F) {
        for (int elapsed = 0; elapsed < milliseconds; elapsed += 20) {
            const auto command = Command();
            x_ += command.twist().linear().x() * std::cos(yaw_) * 0.02;
            y_ += command.twist().linear().x() * std::sin(yaw_) * 0.02;
            yaw_ += command.twist().angular().z() * 0.02;
            stamp_ns_ += 20000000;
            if (optional_sensors) {
                msgs::nav_msgs::Odometry odom;
                *odom.mutable_header()->mutable_stamp() =
                    msgs::builtin_interfaces::TimeFromNanoseconds(stamp_ns_);
                odom.mutable_header()->set_frame_id("map");
                odom.mutable_pose()
                    ->mutable_pose()
                    ->mutable_pose()
                    ->mutable_position()
                    ->set_x(x_);
                odom.mutable_pose()
                    ->mutable_pose()
                    ->mutable_pose()
                    ->mutable_position()
                    ->set_y(y_);
                odom.mutable_pose()
                    ->mutable_pose()
                    ->mutable_pose()
                    ->mutable_orientation()
                    ->set_z(std::sin(yaw_ / 2));
                odom.mutable_pose()
                    ->mutable_pose()
                    ->mutable_pose()
                    ->mutable_orientation()
                    ->set_w(std::cos(yaw_ / 2));
                *odom.mutable_twist()->mutable_twist() = command.twist();
                odom_->Write(odom);
                msgs::sensor_msgs::CameraInfo camera;
                camera.set_width(32);
                camera.set_height(24);
                for (double value : {32., 0., 16., 0., 32., 12., 0., 0., 1.})
                    camera.add_k(value);
                camera_->Write(camera);
            }
            if (send_depth) {
                msgs::sensor_msgs::Image image;
                *image.mutable_header()->mutable_stamp() =
                    msgs::builtin_interfaces::TimeFromNanoseconds(stamp_ns_);
                image.mutable_header()->set_frame_id("camera_optical_frame");
                image.set_width(32);
                image.set_height(24);
                image.set_encoding("32FC1");
                image.set_step(32 * sizeof(float));
                const std::vector<float> pixels(32 * 24, distance);
                image.set_data(reinterpret_cast<const char*>(pixels.data()),
                               pixels.size() * sizeof(float));
                depth_->Write(image);
            }
            std::this_thread::sleep_for(20ms);
        }
    }

    msgs::geometry_msgs::TwistStamped Command() {
        const std::lock_guard<std::mutex> lock(mutex_);
        return command_value_;
    }

    msgs::geometry_msgs::Twist RawCommand() {
        const std::lock_guard<std::mutex> lock(mutex_);
        return raw_command_value_;
    }

    void Topic(std::uint64_t sequence, double x, double y,
               bool exploration = false) {
        navigation::GoalSpec goal;
        goal.set_sequence(sequence);
        goal.set_goal_id("integration-topic");
        goal.set_type(exploration ? navigation::GoalSpec::TYPE_EXPLORATION
                                  : navigation::GoalSpec::TYPE_POSE);
        if (!exploration) {
            goal.mutable_pose()->mutable_pose()->mutable_position()->set_x(x);
            goal.mutable_pose()->mutable_pose()->mutable_position()->set_y(y);
            goal.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(
                1.0);
        }
        goal_->Write(goal);
    }

    auto Send(double x, double y) {
        Action::Goal goal;
        goal.mutable_pose()->mutable_header()->set_frame_id("map");
        goal.mutable_pose()->mutable_pose()->mutable_position()->set_x(x);
        goal.mutable_pose()->mutable_pose()->mutable_position()->set_y(y);
        goal.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(1.0);
        autolink::action::Client<Action>::SendGoalOptions options;
        const auto first_feedback = std::make_shared<std::atomic<bool>>(true);
        options.feedback_callback = [this, first_feedback](
                                        auto, const auto& feedback) {
            if (feedback->has_current_pose() &&
                feedback->has_navigation_time() &&
                std::isfinite(feedback->distance_remaining()))
                ++feedbacks_;
            if (first_feedback->exchange(false) &&
                feedback->navigation_time().sec() == 0)
                ++feedback_starts_;
        };
        auto future = client_->AsyncSendGoal(goal, options);
        Require(future.wait_for(2s) == std::future_status::ready,
                "action acceptance timeout");
        auto handle = future.get();
        Require(handle != nullptr, "action rejected");
        return handle;
    }

    void CheckResult(
        const std::shared_future<
            autolink::action::Client<Action>::WrappedResult>& result,
        int code) {
        Require(result.wait_for(2s) == std::future_status::ready,
                "action result timeout");
        Require(result.get().result != nullptr &&
                    result.get().result->error_code() == code,
                "unexpected action result");
    }

    void Run() {
        auto dispatch = autolink::Async([] { return true; });
        Require(dispatch.wait_for(5s) == std::future_status::ready &&
                    dispatch.get(),
                "Autolink async dispatcher unavailable after discovery warmup");
        Advance(300, true, false);
        Require(observations_ > 0 &&
                    validity_ ==
                        navigation::observation::OBSERVATION_FLAG_DEPTH_VALID,
                "depth-only observation validity");
        {
            const std::lock_guard<std::mutex> lock(mutex_);
            Require(raw_commands_ > 0 &&
                        std::abs(raw_command_value_.linear().x()) < 1e-5 &&
                        std::abs(raw_command_value_.angular().z()) < 1e-5,
                    "missing odometry produced a navigation command");
        }
        Advance(600);
        Require((validity_ & 7) == 7 &&
                    (validity_ &
                     navigation::observation::OBSERVATION_FLAG_IMU_VALID) == 0,
                "optional sensor fusion validity");
        Require(mode_ == navigation::lifecycle::DEGRADATION_MODE_NORMAL,
                "three heartbeats did not reach NORMAL: mode=" +
                    std::to_string(mode_.load()) +
                    " counts=" + std::to_string(heartbeats_[0]) + "," +
                    std::to_string(heartbeats_[1]) + "," +
                    std::to_string(heartbeats_[2]) +
                    " observations=" + std::to_string(observations_));
        std::cout << "PASS DAG components, optional sensors and NORMAL "
                     "heartbeat state\n";

        Topic(1, 10, 0);
        Advance(300);
        auto a = Send(10, 0);
        auto result_a = client_->AsyncGetResult(a);
        Advance(200);
        const auto point_start = std::chrono::steady_clock::now();
        auto b = Send(2, 1);
        auto result_b = client_->AsyncGetResult(b);
        Advance(200);
        CheckResult(result_a, msgs::status_msgs::NAVIGATION_CANCELLED);
        for (int elapsed = 0; elapsed < 10000 && result_b.wait_for(0ms) !=
                                                     std::future_status::ready;
             elapsed += 100) {
            Advance(100);
        }
        Require(result_b.wait_for(0ms) == std::future_status::ready,
                "point goal did not finish: x=" + std::to_string(x_) + " y=" +
                    std::to_string(y_) + " yaw=" + std::to_string(yaw_) +
                    " vx=" + std::to_string(Command().twist().linear().x()) +
                    " wz=" + std::to_string(Command().twist().angular().z()) +
                    " feedbacks=" + std::to_string(feedbacks_));
        CheckResult(result_b, msgs::status_msgs::OK);
        const auto point_time =
            std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                          point_start)
                .count();
        Require(point_time < 10.0 && std::hypot(x_ - 2, y_ - 1) < 0.3 &&
                    feedbacks_ > 0 && feedback_starts_ >= 2,
                "point navigation or feedback timing failed");
        std::cout << "PASS topic->Action, Action->Action, feedback, point "
                     "(2,1), error="
                  << std::hypot(x_ - 2, y_ - 1) << " elapsed_s=" << point_time
                  << "\n";

        auto c = Send(10, 0);
        auto result_c = client_->AsyncGetResult(c);
        Advance(200);
        Topic(2, 10, 1);
        Advance(200);
        CheckResult(result_c, msgs::status_msgs::NAVIGATION_CANCELLED);
        auto d = Send(10, 0);
        auto result_d = client_->AsyncGetResult(d);
        Advance(200);
        auto cancel = client_->AsyncCancelGoal(d);
        Require(
            cancel.wait_for(2s) == std::future_status::ready && cancel.get(),
            "cancel rejected");
        Advance(500);
        CheckResult(result_d, msgs::status_msgs::NAVIGATION_CANCELLED);
        // Check goal ownership at the navigation output; Safety still applies
        // acceleration/jerk limits to physical deceleration.
        {
            const std::lock_guard<std::mutex> lock(mutex_);
            Require(raw_commands_ > 0 &&
                        std::abs(raw_command_value_.linear().x()) < 1e-5 &&
                        std::abs(raw_command_value_.angular().z()) < 1e-5,
                    "cancel replayed cached topic goal");
        }
        std::cout << "PASS Action->topic and cancellation without cached-goal "
                     "replay\n";

        // The cancelled goal's jerk-limited deceleration can still move the
        // plant. Establish a stopped baseline before measuring exploration,
        // otherwise residual motion satisfies the distance check too early.
        for (int elapsed = 0; elapsed < 3000; elapsed += 100) {
            Advance(100);
            const auto command = Command();
            if (std::abs(command.twist().linear().x()) < 1e-3 &&
                std::abs(command.twist().angular().z()) < 1e-3)
                break;
        }
        Require(std::abs(Command().twist().linear().x()) < 1e-3 &&
                    std::abs(Command().twist().angular().z()) < 1e-3,
                "cancelled goal did not finish decelerating");

        const double exploration_x = x_, exploration_y = y_;
        Topic(3, 0, 0, true);
        double forward_distance = 0.0;
        for (int elapsed = 0; elapsed < 16000; elapsed += 100) {
            Advance(100);
            forward_distance =
                std::hypot(x_ - exploration_x, y_ - exploration_y);
            if (forward_distance > 0.02 && RawCommand().linear().x() > 0 &&
                Command().twist().linear().x() > 0)
                break;
        }
        {
            const std::lock_guard<std::mutex> lock(mutex_);
            Require(
                forward_distance > 0.02 &&
                    raw_command_value_.linear().x() > 0 &&
                    command_value_.twist().linear().x() > 0,
                "exploration did not move: raw_vx=" +
                    std::to_string(raw_command_value_.linear().x()) +
                    " raw_wz=" +
                    std::to_string(raw_command_value_.angular().z()) + " vx=" +
                    std::to_string(command_value_.twist().linear().x()) +
                    " wz=" +
                    std::to_string(command_value_.twist().angular().z()) +
                    " forward_distance=" + std::to_string(forward_distance) +
                    " mode=" + std::to_string(mode_.load()));
        }
        for (int elapsed = 0; elapsed < 2000; elapsed += 100) {
            Advance(100, true, true, 0.25F);
            if (std::abs(Command().twist().linear().x()) < 1e-5) break;
        }
        Require(std::abs(Command().twist().linear().x()) < 1e-5,
                "obstacle did not stop");
        Advance(600);
        Require(Command().twist().linear().x() > 0,
                "obstacle removal did not recover");
        int before;
        {
            const std::lock_guard<std::mutex> lock(mutex_);
            before = commands_;
        }
        Advance(260, false);
        auto stopped = Command();
        Require(std::abs(stopped.twist().linear().x()) < 1e-5 &&
                    std::abs(stopped.twist().angular().z()) < 1e-5 &&
                    stopped.header().frame_id() == "base_link" &&
                    msgs::builtin_interfaces::TimeToNanoseconds(
                        stopped.header().stamp()) < 100000000000ULL,
                "depth discontinuity or output header failed");
        int stopped_count, stopped_zero_count;
        {
            const std::lock_guard<std::mutex> lock(mutex_);
            stopped_count = commands_;
            stopped_zero_count = zero_commands_;
        }
        Advance(300, false);
        {
            const std::lock_guard<std::mutex> lock(mutex_);
            Require(commands_ >= before + 15 &&
                        commands_ >= stopped_count + 10 &&
                        commands_ - stopped_count ==
                            zero_commands_ - stopped_zero_count,
                    "zero commands stopped publishing or resumed stale motion");
        }
        const double recovery_x = x_, recovery_y = y_, recovery_yaw = yaw_;
        double recovery_distance = 0.0;
        for (int elapsed = 0; elapsed < 2000; elapsed += 100) {
            Advance(100);
            recovery_distance = (x_ - recovery_x) * std::cos(recovery_yaw) +
                                (y_ - recovery_y) * std::sin(recovery_yaw);
            if (recovery_distance > 0.02 && Command().twist().linear().x() > 0)
                break;
        }
        Require(recovery_distance > 0.02 && Command().twist().linear().x() > 0,
                "depth recovery did not resume motion");
        std::cout << "PASS exploration, obstacle stop/recovery and depth lease "
                     "with continuous TwistStamped\n";

        components_.at("behavior")->Shutdown();
        // Guardian polls every 500 ms. A final, not-yet-observed heartbeat
        // can first be received after Shutdown; allow two polls plus its
        // configured 200 ms receive lease before requiring degradation.
        Advance(1200);
        Require(mode_ == navigation::lifecycle::DEGRADATION_MODE_REFLEX_ONLY,
                "behavior heartbeat loss did not degrade: mode=" +
                    std::to_string(mode_.load()) +
                    " heartbeat_counts=" + std::to_string(heartbeats_[0]) +
                    "," + std::to_string(heartbeats_[1]) + "," +
                    std::to_string(heartbeats_[2]));
        std::cout << "PASS behavior shutdown -> REFLEX_ONLY\n";
        // Restore the timer from the same DAG before testing navigation loss.
        LoadDag("navigation", "behavior");
        Advance(700);
        Require(mode_ == navigation::lifecycle::DEGRADATION_MODE_NORMAL,
                "heartbeat recovery did not restore NORMAL");
        components_.at("navigation")->Shutdown();
        Advance(1200);
        Require(std::abs(Command().twist().linear().x()) < 1e-5 &&
                    std::abs(Command().twist().angular().z()) < 1e-5,
                "navigation shutdown replayed a cached command");
        Require(mode_ == navigation::lifecycle::DEGRADATION_MODE_REFLEX_ONLY,
                "navigation heartbeat loss did not degrade");
        components_.at("safety")->Shutdown();
        Advance(1200);
        Require(mode_ == navigation::lifecycle::DEGRADATION_MODE_SAFE_STOP,
                "safety heartbeat loss did not stop");
        Require(degradations_ >= 5 && invalid_degradations_ == 0,
                "degradation transition metadata missing");
        std::cout
            << "PASS navigation/safety shutdown -> REFLEX_ONLY/SAFE_STOP\n";
    }

private:
    std::string root_;
    std::shared_ptr<autolink::Node> node_;
    std::shared_ptr<autolink::Writer<msgs::sensor_msgs::Image>> depth_;
    std::shared_ptr<autolink::Writer<msgs::sensor_msgs::CameraInfo>> camera_;
    std::shared_ptr<autolink::Writer<msgs::nav_msgs::Odometry>> odom_;
    std::shared_ptr<autolink::Writer<navigation::GoalSpec>> goal_;
    std::shared_ptr<autolink::Reader<msgs::geometry_msgs::TwistStamped>>
        command_;
    std::shared_ptr<autolink::Reader<msgs::geometry_msgs::Twist>> raw_command_;
    std::shared_ptr<autolink::Reader<navigation::observation::Observation>>
        observation_;
    std::shared_ptr<autolink::Reader<navigation::lifecycle::Degradation>>
        degradation_;
    std::array<
        std::shared_ptr<autolink::Reader<navigation::lifecycle::Heartbeat>>, 3>
        heartbeat_readers_;
    std::array<std::atomic<int>, 3> heartbeats_{};
    std::shared_ptr<autolink::action::Client<Action>> client_;
    std::map<std::string, std::shared_ptr<autolink::ComponentBase>> components_;
    std::mutex mutex_;
    std::string navigation_fixture_;
    std::string safety_fixture_;
    msgs::geometry_msgs::TwistStamped command_value_;
    msgs::geometry_msgs::Twist raw_command_value_;
    int commands_{0};
    int zero_commands_{0};
    int raw_commands_{0};
    std::atomic<int> observations_{0};
    std::atomic<std::uint32_t> validity_{0};
    std::atomic<int> feedbacks_{0};
    std::atomic<int> feedback_starts_{0};
    std::atomic<int> degradations_{0}, invalid_degradations_{0};
    std::atomic<navigation::lifecycle::DegradationMode> mode_{
        navigation::lifecycle::DEGRADATION_MODE_UNSPECIFIED};
    double x_{0}, y_{0}, yaw_{0};
    std::uint64_t stamp_ns_{10000000000ULL};
};

int main(int argc, char** argv) {
    if (argc != 2)
        return 2;
    namespace fs = std::filesystem;
    const auto temporary =
        fs::temp_directory_path() /
        ("navigation-integration-" +
         std::to_string(
             std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(temporary / "conf");
    struct Cleanup {
        fs::path path;
        ~Cleanup() {
            fs::remove_all(path);
        }
    } cleanup{temporary};
    std::ofstream(temporary / "conf/autolink.pb.conf")
        << "run_mode_conf { run_mode: MODE_REALITY clock_mode: MODE_AUTOLINK "
           "}\n"
           "scheduler_conf { routine_num: 100 default_proc_num: 4 }\n";
    setenv("AUTOLINK_PATH", temporary.c_str(), 1);
    setenv("NAVIGATION_CONF_DIR", argv[1], 1);
    if (!autolink::Init("navigation_component_integration"))
        return 1;
    int result = 0;
    try {
        CheckCancelWire();
        CheckTimerLifecycle();
        Fixture fixture(argv[1]);
        fixture.Run();
    } catch (const std::exception& error) {
        std::cerr << error.what() << "\n";
        result = 1;
    }
    autolink::Clear();
    return result;
}
