/* Copyright 2026 The Openbot Authors. */
/** @file native_controller_io_test.cpp
 * @brief Validate actual ControllerServer state/command injection without global-source fallback.
 * @details Uses the real server and generated automsgs. Controlled feedback
 * tests the native IO contract and does not claim real robot tracking accuracy.
 */
#include <iostream>
#include <stdexcept>
#include <automsgs/msgs/time_utils.hpp>
#include "autolink/autolink.hpp"
#include "autonomy/control/controller_server.hpp"
#include "autonomy/control/native_command_gate.hpp"
#include "autonomy/control/common/controller_exceptions.hpp"
#include "autonomy/map/costmap_2d/footprint.hpp"

namespace {
namespace c = autonomy::control;
using Odometry = automsgs::msgs::nav_msgs::Odometry;
using Twist = automsgs::msgs::geometry_msgs::TwistStamped;
/** @brief Fail with the exact native state/transport invariant. */
void Require(bool value,const char* reason) { if (!value) throw std::runtime_error(reason); }
/** @brief Actual generated feedback representation in the shared native navigation frame. */
Odometry State() {
    Odometry value;
    *value.mutable_header()->mutable_stamp() = automsgs::msgs::builtin_interfaces::TimeNow();
    value.mutable_header()->set_frame_id("native_io_world"); value.set_child_frame_id("base_link");
    auto* pose = value.mutable_pose()->mutable_pose(); *pose->mutable_header() = value.header();
    pose->mutable_pose()->mutable_position()->set_x(2); pose->mutable_pose()->mutable_position()->set_y(3);
    pose->mutable_pose()->mutable_orientation()->set_w(1);
    value.mutable_twist()->mutable_twist()->mutable_linear()->set_x(.1);
    return value;
}
/** @brief Prepared actual map wrapper; no background thread or fabricated localization is started. */
std::shared_ptr<autonomy::map::costmap_2d::Costmap2DWrapper> Map() {
    autonomy::map::proto::Costmap2DOptions options;
    options.set_enabled(false); options.set_frame_id("native_io_world"); options.set_resolution(.1);
    options.set_width(4); options.set_height(4); options.add_plugins("none");
    return std::make_shared<autonomy::map::costmap_2d::Costmap2DWrapper>(options,"native_io_map");
}
/** @brief No missing native state can be replaced by default TF or velocity. */
void StateContract() {
    auto state = std::make_shared<std::optional<Odometry>>(State());
    c::NativeControllerIO io;
    io.read_odometry = [state] { return *state; };
    io.publish_command = [](const Twist&) { return true; };
    c::ControllerServer server({},Map(),io);
    Odometry read; automsgs::msgs::geometry_msgs::PoseStamped pose;
    Require(server.GetLatestOdometry(read) && server.GetRobotPose(pose) && pose.pose().position().x() == 2 &&
        pose.header().stamp().nanosec() == state->value().header().stamp().nanosec(),
        "native server consumes actual original feedback identity and pose");
    state->value().mutable_header()->set_frame_id("unrelated_world");
    Require(!server.GetLatestOdometry(read) && !server.GetRobotPose(pose),"wrong-frame source cannot enter native control");
    *state = State(); state->value().mutable_pose()->mutable_pose()->mutable_header()->mutable_stamp()->set_nanosec(0);
    Require(!server.GetRobotPose(pose),"mismatched nested acquisition identity is rejected");
    *state = State(); state->value().mutable_header()->mutable_stamp()->set_sec(1);
    *state->value().mutable_pose()->mutable_pose()->mutable_header() = state->value().header();
    Require(!server.GetLatestOdometry(read),"expired original acquisition is not renewed by a read");
    state->reset(); Require(!server.GetRobotPose(pose),"missing owner feedback has no global TF fallback");
}
/** @brief Rejected native proposal attempts a bounded stop through the same callback and cannot use global cmd_vel. */
void CommandContract() {
    auto commands = std::make_shared<std::vector<Twist>>();
    c::NativeControllerIO io;
    io.read_odometry = [] { return std::optional<Odometry>{State()}; };
    io.publish_command = [commands](const Twist& command) {
        commands->push_back(command); return command.has_header() && command.has_twist() && command.twist().linear().x() == 0;
    };
    c::ControllerServer server({},Map(),io);
    Twist command; *command.mutable_header()->mutable_stamp() = automsgs::msgs::builtin_interfaces::TimeNow();
    command.mutable_header()->set_frame_id("base_link"); command.mutable_twist()->mutable_linear()->set_x(.2);
    bool rejected = false;
    try { server.PublishVelocity(command); } catch (const c::common::ControllerException&) { rejected = true; }
    Require(rejected && commands->size() == 2 && commands->front().twist().linear().x() == .2 &&
        commands->back().twist().linear().x() == 0 && commands->back().header().frame_id() == "base_link",
        "denied command cannot become an admitted speed or escape the native transport");
    server.PublishZeroVelocity(); Require(commands->size() == 3 && commands->back().has_twist(),
        "explicit stop includes its actual native payload and uses the injected transport");
}
/** @brief Actual Node registration excludes legacy global state, map and unused policy inputs. */
void NativeRegistrations() {
    c::NativeControllerIO io; io.read_odometry = [] { return std::optional<Odometry>{State()}; };
    io.publish_command = [](const Twist&) { return true; };
    c::ControllerServer server({},Map(),io);
    auto node = autolink::CreateNode("native_controller_io_registration");
    Require(server.AttachAutolinkNode(node),"actual native action/server Node configured");
    Require(!node->GetReader<Odometry>("/odom") &&
        !node->GetReader<automsgs::msgs::map_msgs::OccupancyGrid>("/map") &&
        !node->GetReader<automsgs::msgs::sensor_msgs::Image>("/image"),
        "native mode cannot attach global state or unused policy input endpoints");
    server.Shutdown();
}
/** @brief Command admission checks the enlarged full stopping footprint and original evidence identity. */
void MeasuredCommandGate() {
    auto state = State();
    auto grid = std::make_shared<autonomy::navigation::GridMap>(80,80,.1,0,0,"native_io_world");
    grid->Fill({autonomy::navigation::CellState::kFree,0,1});
    autonomy::perception::MapReadView map;
    map.grid = grid; map.map_epoch = 1; map.frame_id = "native_io_world";
    const double now = state.header().stamp().sec()+state.header().stamp().nanosec()*1e-9;
    map.source_stamp = now;
    c::NativeCommandGateOptions options; options.body.robot_radius = .1; options.body.robot_height = .2;
    options.body.horizontal_margin = .02; options.body.vertical_margin = .02;
    Twist command; *command.mutable_header() = state.header(); command.mutable_header()->set_frame_id("base_link");
    command.mutable_twist()->mutable_linear()->set_x(.2);
    Require(c::VerifyNativeCommand(command,state,map,options,now).safe,"actual same-frame command fits known full-body and stopping volume");
    auto side = std::make_shared<autonomy::navigation::GridMap>(*grid);
    side->Set(20,27,{autonomy::navigation::CellState::kOccupied,0,1}); map.grid = side;
    Require(c::VerifyNativeCommand(command,state,map,options,now).safe,
        "directional stopping volume permits travel parallel to a nearby side wall");
    map.grid = grid;
    auto blocked = std::make_shared<autonomy::navigation::GridMap>(*grid);
    blocked->Set(22,30,{autonomy::navigation::CellState::kOccupied,0,1}); map.grid = blocked;
    Require(!c::VerifyNativeCommand(command,state,map,options,now).safe,"obstacle in stopping envelope denies the actual proposal");
    map.grid = grid; command.mutable_twist()->mutable_linear()->set_y(.1);
    Require(!c::VerifyNativeCommand(command,state,map,options,now).safe,"nonplanar proposal cannot enter the differential-drive transport");
    command.mutable_twist()->mutable_linear()->set_y(0); map.has_height_evidence = false;
    Require(!c::VerifyNativeCommand(command,state,map,options,now).safe,"a plain planar map cannot invent vertical/ground evidence");
    map.has_height_evidence = true;
    Require(!c::VerifyNativeCommand(command,state,map,options,now+2).safe,"old native feedback and map do not refresh their acquisition time");
    map.frame_id = "different_world";
    Require(!c::VerifyNativeCommand(command,state,map,options,now).safe,"command admission retains actual grid/feedback coordinate identity");
}
/** @brief Startup admission is yaw-only, drift-bounded and expires without map fabrication. */
void StartupRotationGate() {
    auto state = State();
    const double now = state.header().stamp().sec() +
        state.header().stamp().nanosec() * 1e-9;
    Twist command;
    *command.mutable_header() = state.header();
    command.mutable_header()->set_frame_id("base_link");
    command.mutable_twist()->mutable_angular()->set_z(.4);
    c::NativeStartupRotationOptions options;
    options.maximum_position_drift = .03;
    Require(c::VerifyNativeStartupRotation(command, state, 2, 3, now + 5,
        options, now).safe,
        "pure yaw at the deployment-confirmed startup position is admitted");
    command.mutable_twist()->mutable_linear()->set_x(.01);
    Require(!c::VerifyNativeStartupRotation(command, state, 2, 3, now + 5,
        options, now).safe,
        "startup admission never permits translation");
    command.mutable_twist()->mutable_linear()->set_x(0);
    state.mutable_pose()->mutable_pose()->mutable_pose()->mutable_position()->set_x(2.1);
    Require(!c::VerifyNativeStartupRotation(command, state, 2, 3, now + 5,
        options, now).safe,
        "measured drift revokes the startup envelope");
    state = State();
    Require(!c::VerifyNativeStartupRotation(command, state, 2, 3, now - 1,
        options, now).safe,
        "expired startup admission cannot authorize another motion command");
}
/** @brief Start support closes only an unknown self-occluded disc and never obstacles. */
void StartupSupportSafety() {
    auto grid = std::make_shared<autonomy::navigation::GridMap>(40, 30, .2);
    grid->Fill({autonomy::navigation::CellState::kFree, 0, 1});
    for (int y = 14; y <= 16; ++y)
        for (int x = 19; x <= 21; ++x)
            grid->Set(x, y, {autonomy::navigation::CellState::kUnknown, 0, 1});
    autonomy::navigation::SafetyOptions body;
    body.robot_radius = .1; body.horizontal_margin = .01;
    body.robot_height = .2; body.vertical_margin = .02;
    Require(!autonomy::navigation::GeometrySafety(grid, body).VerifyPose({4.1, 3.1, 0}).safe,
        "unconfirmed startup hole remains blocked");
    body.startup_support = autonomy::navigation::StartupSupportRegion{4.1, 3.1, .45, 0, 1};
    Require(autonomy::navigation::GeometrySafety(grid, body).VerifyPose({4.1, 3.1, 0}).safe,
        "bounded startup support admits only the measured self-occluded hole");
    grid->Set(25, 15, {autonomy::navigation::CellState::kUnknown, 0, 1});
    Require(!autonomy::navigation::GeometrySafety(grid, body).VerifyPose({5.1, 3.1, 0}).safe,
        "startup support does not clear unknown space outside its envelope");
    grid->Set(20, 15, {autonomy::navigation::CellState::kOccupied, 0, 1});
    Require(!autonomy::navigation::GeometrySafety(grid, body).VerifyPose({4.1, 3.1, 0}).safe,
        "startup support never overrides an observed obstacle");
}
/** @brief Actual MPPI configuration recognizes the shared polygon mode selected by its original map proto. */
void SharedMPPIConfiguration() {
    autonomy::map::proto::Costmap2DOptions map_options;
    map_options.set_enabled(false); map_options.set_frame_id("native_io_world"); map_options.set_resolution(.1);
    map_options.set_width(4); map_options.set_height(4); map_options.add_plugins("none");
    const auto footprint = autonomy::map::costmap_2d::makeFootprintFromRadius(.2);
    for (const auto& point : footprint) {
        auto* position = map_options.mutable_footprint()->add_points();
        position->set_x(point.x()); position->set_y(point.y());
    }
    auto shared = std::make_shared<autonomy::map::costmap_2d::Costmap2DWrapper>(map_options,"native_mppi_map");
    Require(!shared->getUseRadius() && shared->getRobotFootprint().size() == footprint.size(),
        "shared map selects its actual footprint mode before optimizer configuration");
    c::proto::ControllerOptions options;
    options.add_controller_plugins("mppi_controller:MPPIController"); options.set_controller_frequency(10);
    options.mutable_mppi_controller_options()->set_motion_model("DiffDrive");
    options.mutable_mppi_controller_options()->set_model_dt(.1); options.mutable_mppi_controller_options()->set_batch_size(32);
    options.mutable_mppi_controller_options()->set_time_steps(12);
    c::NativeControllerIO io; io.read_odometry = [] { return std::optional<Odometry>{State()}; };
    io.publish_command = [](const Twist&) { return true; };
    c::ControllerServer server(options,shared,io);
    server.LoadPlugins();
    Odometry state; Require(server.GetLatestOdometry(state),"actual configured MPPI server retains native acquisition IO");
    server.Shutdown();
}
}  // namespace
/** @brief Run actual native server contracts with initialized Autolink. */
int main(int argc,char** argv) {
    if (!autolink::Init(argv[0])) return 2;
    int code = 0;
    try { StateContract(); CommandContract(); NativeRegistrations(); MeasuredCommandGate(); StartupRotationGate(); StartupSupportSafety(); SharedMPPIConfiguration(); std::cout << "PASS native_controller_io" << std::endl; }
    catch (const std::exception& error) { code = 1; std::cerr << "FAIL native_controller_io: " << error.what() << std::endl; }
    autolink::Clear(); return code;
}
