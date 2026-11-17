/**
 * @file vista_server_test.cpp
 * @brief Exercise native server alias resolution, settings and shared graph ownership.
 * @details Uses the actual Costmap and PlannerServer without a substitute plugin factory.
 */
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/planning/planner_server.hpp"
#include "autonomy/planning/planner/vista/vista_planner.hpp"
#include "autonomy/task/exploration/integration/vista_path_provider.hpp"

namespace {
namespace vista = autonomy::planning::planner::vista;
namespace proto = autonomy::planning::proto;
using Pose = automsgs::msgs::geometry_msgs::PoseStamped;

/** @brief Fail the native regression immediately when a contract is violated. */
void Require(bool result, const char* reason) {
    if (!result) throw std::runtime_error(reason);
}

/** @brief Construct a planar native goal in the configured global coordinate frame. */
Pose At(double x, double y, double yaw = 0) {
    Pose pose;
    pose.mutable_header()->set_frame_id("map");
    pose.mutable_pose()->mutable_position()->set_x(x);
    pose.mutable_pose()->mutable_position()->set_y(y);
    pose.mutable_pose()->mutable_orientation()->set_z(std::sin(yaw / 2));
    pose.mutable_pose()->mutable_orientation()->set_w(std::cos(yaw / 2));
    return pose;
}

/** @brief Preserve explicit zero options and reject invalid capacity/geometry. */
void SettingsContract() {
    proto::VistaPlannerOptions options;
    auto settings = vista::ReadVistaPlannerSettings(options);
    Require(settings.maximum_nodes == 4096 && settings.maximum_cells == 2000000,
            "unset protobuf capacities retain documented defaults");
    options.set_horizontal_margin(0);
    options.set_vertical_margin(0);
    settings = vista::ReadVistaPlannerSettings(options);
    Require(settings.safety.horizontal_margin == 0 && settings.safety.vertical_margin == 0,
            "explicit zero clearance is not confused with an omitted field");
    options.set_maximum_nodes(0);
    bool rejected = false;
    try { (void)vista::ReadVistaPlannerSettings(options); }
    catch (const std::invalid_argument&) { rejected = true; }
    Require(rejected, "zero graph capacity must be rejected");
    options.set_maximum_nodes(1);
    options.set_horizontal_margin(std::numeric_limits<double>::quiet_NaN());
    rejected = false;
    try { (void)vista::ReadVistaPlannerSettings(options); }
    catch (const std::invalid_argument&) { rejected = true; }
    Require(rejected, "nonfinite safety settings must be rejected before planning");
}

/** @brief Verify that server ids construct configured Vista objects with one reusable graph. */
void ServerContract() {
    proto::PlannerOptions options;
    options.add_planner_plugins("vista_planner");
    options.add_planner_plugins("secondary:VistaPlanner");
    options.set_default_planner_id("vista_planner");
    options.mutable_vista()->set_horizontal_margin(0);
    options.mutable_vista()->set_maximum_nodes(1);
    auto* map_options = options.mutable_costmap();
    map_options->set_enabled(true);
    map_options->set_name("vista_test_costmap");
    map_options->set_frame_id("map");
    map_options->set_resolution(.2);
    map_options->set_width(8);
    map_options->set_height(6);
    map_options->set_robot_radius(.1);
    map_options->add_plugins("none");

    autonomy::planning::PlannerServer server(options);
    auto primary = std::dynamic_pointer_cast<vista::VistaPlanner>(server.GetPlanner());
    auto secondary = std::dynamic_pointer_cast<vista::VistaPlanner>(server.GetPlanner("secondary"));
    Require(primary && secondary && primary != secondary,
            "alias and id:class entries create distinct configured native instances");
    Require(primary == server.GetPlanner("vista_planner") && !server.GetPlanner("missing"),
            "shared access returns the actual configured registry object");
    auto routes = primary->SharedRoutePlanner();
    Require(routes != nullptr, "server constructor initializes the Vista graph");
    autonomy::task::exploration::VistaPathProvider provider(routes);
    Require(provider.planner() == primary->SharedRoutePlanner(),
            "exploration and navigation hold exactly the same route graph");

    auto* map = server.GetCostmapWrapper()->getCostmap();
    map->resetMapToValue(0, 0, map->getSizeInCellsX(), map->getSizeInCellsY(),
                        autonomy::map::costmap_2d::FREE_SPACE);
    const auto start = At(1, 1, -.2), goal = At(5, 1, .7);
    const auto path = server.GetPlan(start, goal, "vista_planner", [] { return false; });
    Require(path.poses_size() >= 2 && path.header().frame_id() == "map",
            "native server returns a real protobuf path through Vista");
    Require(path.poses(path.poses_size() - 1).pose().SerializeAsString() == goal.pose().SerializeAsString(),
            "native server preserves the requested final observation orientation");
    Require(primary->LastRoute() && !primary->LastRoute()->execution_verified,
            "global planning success does not manufacture execution safety");
    const auto snapshot = primary->LatestMapView();
    Require(snapshot && !snapshot->has_height_evidence && !snapshot->mapping_synchronized,
            "server integration cannot manufacture height or upstream fusion evidence");
}
}  // namespace

/** @brief Run settings and full native PlannerServer regression contracts. */
int main() {
    try {
        SettingsContract();
        std::cout << "PASS native_vista_settings\n";
        ServerContract();
        std::cout << "PASS native_vista_server\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL native_vista_server: " << error.what() << '\n';
        return 1;
    }
}
