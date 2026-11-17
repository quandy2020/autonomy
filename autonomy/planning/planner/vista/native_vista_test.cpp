/* Copyright 2026 The Openbot Authors. */
#include <atomic>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>
#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/planning/planner/vista/vista_planner.hpp"
#include "autonomy/task/exploration/integration/vista_path_provider.hpp"

namespace v = autonomy::planning::planner::vista;
namespace m = autonomy::map::costmap_2d;
namespace p = autonomy::planning::proto;
namespace e = autonomy::task::exploration;
using Pose = automsgs::msgs::geometry_msgs::PoseStamped;
using Path = automsgs::msgs::nav_msgs::Path;

namespace {
void Require(bool ok,const char* why) { if (!ok) throw std::runtime_error(why); }
void InitializeMap(m::Costmap2D& map, unsigned width, unsigned height, double resolution) {
    // The native checkout defines default construction and resizeMap. Its
    // dimension constructor is declared but has no implementation; fixtures
    // use the implemented API without changing the protected map module.
    map.resizeMap(width, height, resolution, 0, 0);
    for (unsigned y = 0; y < height; ++y) {
        for (unsigned x = 0; x < width; ++x) {
            map.setCost(x, y, m::FREE_SPACE);
        }
    }
}
Pose At(double x,double y,double yaw=0,const std::string& frame="map") {
    Pose pose; pose.mutable_header()->set_frame_id(frame);
    pose.mutable_pose()->mutable_position()->set_x(x);
    pose.mutable_pose()->mutable_position()->set_y(y);
    pose.mutable_pose()->mutable_orientation()->set_z(std::sin(yaw/2));
    pose.mutable_pose()->mutable_orientation()->set_w(std::cos(yaw/2)); return pose;
}
v::VistaPlannerSettings Settings() {
    v::VistaPlannerSettings settings; settings.safety.robot_radius=.1;
    settings.safety.horizontal_margin=0; return settings;
}
void NativeSnapshotOwnership() {
    m::Costmap2D map; InitializeMap(map, 20, 20, .1); v::NativeCostmapReader reader;
    const auto a=reader.Read(map,"map");
    Require(a.source_stamp==0 && !a.mapping_synchronized && !a.has_height_evidence,
            "a native read must not manufacture fresh observations, fusion or 3D clearance");
    map.setCost(4,4,m::LETHAL_OBSTACLE);
    auto b=reader.Read(map,"map");
    Require(a.grid->At(4,4).state==opt::navigation::CellState::kFree,
            "the old native snapshot must own immutable content");
    Require(b.grid->At(4,4).state==opt::navigation::CellState::kOccupied &&
            b.geometry_revision>a.geometry_revision,"content changes advance geometry");
    const auto c=reader.Read(map,"map");
    Require(c.geometry_revision==b.geometry_revision && c.dirty_regions.empty(),
            "reading again does not fabricate a geometric update");
    v::CostmapReadMetadata meta; meta.alignment_id=5;
    meta.observation_cut.Include(autonomy::perception::SensorKind::kLidar,9);
    const auto d=reader.Read(map,"map",meta);
    Require(d.map_epoch>c.map_epoch && d.observation_cut.sequence[0]==9 &&
            !d.mapping_synchronized,"realignment and source cut are explicit independent evidence");
}
void GlobalPlannerContractAndSharing() {
    m::Costmap2D map; InitializeMap(map, 30, 20, .2); v::NativeCostmapReader reader;
    v::VistaPlanner planner; Path path; path.add_poses()->CopyFrom(At(99,99));
    Require(planner.CreatePlan(At(.7,.7),At(5.1,.7),path,{})==p::PLANNER_NOT_INITIALIZED &&
            !path.poses_size(),"unconfigured native plugin clears stale output");
    planner.Configure({ },"vista_planner",[&]{return reader.Read(map,"map");},Settings());
    auto shared=planner.SharedRoutePlanner(); e::VistaPathProvider exploration(shared);
    autonomy::planning::common::GlobalPlanner& native=planner;
    const Pose start=At(.7,.7,-.2),goal=At(5.1,.7,.8);
    Require(native.CreatePlan(start,goal,path,{})==p::PLANNER_SUCCESS && path.poses_size()>=2,
            "the real GlobalPlanner interface returns a native Protobuf path");
    Require(path.header().frame_id()=="map" &&
            path.poses(0).pose().SerializeAsString()==start.pose().SerializeAsString() &&
            path.poses(path.poses_size()-1).pose().SerializeAsString()==goal.pose().SerializeAsString(),
            "native endpoints and requested observation headings survive serialization");
    Require(exploration.planner()==shared && exploration.Evidence()->map_epoch==planner.LastRoute()->map_epoch,
            "exploration reuses the exact same updated Vista graph instance");
    Require(!planner.LastRoute()->execution_verified,"global path success never certifies execution");
    Require(native.CreatePlan(start,At(5.1,.7,0,"odom"),path,{})==p::PLANNER_TF_ERROR && !path.poses_size(),
            "mixed coordinate frames cannot publish a path");
    Pose invalid=goal;invalid.mutable_pose()->mutable_position()->set_x(std::numeric_limits<double>::quiet_NaN());
    Require(native.CreatePlan(start,invalid,path,{})==p::PLANNER_INVALID_GOAL,
            "invalid native goal is reported distinctly");
    Require(native.CreatePlan(start,goal,path,[]{return true;})==p::PLANNER_CANCELED &&
            !path.poses_size() && !planner.LastRoute(),"cancellation cannot retain the previous plan");
    planner.SetMapViewSource([]()->std::optional<autonomy::perception::MapReadView>{throw 17;});
    Require(native.CreatePlan(start,goal,path,{})==p::PLANNER_INTERNAL_ERROR && !path.poses_size(),
            "a non-standard callback exception cannot publish a partial native path");
}
void DoorFallbackAndOldEpoch() {
    m::Costmap2D map; InitializeMap(map, 30, 20, .2); v::NativeCostmapReader reader;
    for(unsigned y=0;y<20;++y)if(y<8 || y>11)map.setCost(15,y,m::LETHAL_OBSTACLE);
    auto view=reader.Read(map,"map"); v::VistaPlanner planner; auto settings=Settings();settings.maximum_nodes=1;
    planner.Configure({ },"vista_planner",[&]{return view;},settings);
    Path path; const auto start=At(.7,.7),goal=At(5.1,.7);
    Require(planner.CreatePlan(start,goal,path,{})==p::PLANNER_SUCCESS &&
            planner.LastRoute()->used_grid_fallback,"native sparse-graph failure uses the safe grid route");
    for(unsigned y=8;y<=11;++y)map.setCost(15,y,m::LETHAL_OBSTACLE);
    view=reader.Read(map,"map");
    Require(planner.CreatePlan(start,goal,path,{})==p::PLANNER_NO_PATH_FOUND && !path.poses_size(),
            "a newly sealed doorway invalidates the native path");
    for(unsigned y=8;y<=11;++y)map.setCost(15,y,m::FREE_SPACE);
    const auto old=view; v::CostmapReadMetadata metadata;metadata.alignment_id=7;
    view=reader.Read(map,"map",metadata);
    Require(planner.CreatePlan(start,goal,path,{})==p::PLANNER_SUCCESS,"removed obstacle restores a native route");
    const auto current_epoch=planner.SharedRoutePlanner()->GraphSnapshot().map_epoch;
    view=old;
    Require(planner.CreatePlan(start,goal,path,{})==p::PLANNER_TF_ERROR && !path.poses_size() &&
            planner.SharedRoutePlanner()->GraphSnapshot().map_epoch==current_epoch,
            "old map epochs cannot roll back the shared graph or escape as plans");
}
void NativePluginFactory() {
    using Base=autonomy::planning::common::GlobalPlanner;
    auto* pm=autolink::plugin_manager::PluginManager::Instance();
    Require(pm->RegisterInProcessClass<Base>("VistaPlanner"),"native manager registers the in-process Vista class");
    auto instance=pm->CreateInstance<Base>("VistaPlanner");
    Require(instance && dynamic_cast<v::VistaPlanner*>(instance.get()),
            "native plugin manager creates Vista through the real GlobalPlanner base");
    Path path;Require(instance->CreatePlan(At(.7,.7),At(1,.7),path,{})==p::PLANNER_NOT_INITIALIZED,
                     "factory construction alone cannot bypass configuration");
}
void ReentrantCallbacksAndConfigurationChange() {
    m::Costmap2D map; InitializeMap(map, 30, 20, .2); v::NativeCostmapReader reader;
    v::VistaPlanner planner; Path path;
    planner.Configure({}, "vista_planner", [&] {
        Require(planner.SharedRoutePlanner() != nullptr, "map callback can read native planner state");
        planner.LatestMapView(); planner.LastRoute();
        return reader.Read(map, "map");
    }, Settings());
    auto canceled = [&] {
        planner.LastRoute(); planner.LatestMapView();
        planner.SharedRoutePlanner()->GraphSnapshot();
        return false;
    };
    Require(planner.CreatePlan(At(.7,.7), At(5.1,.7), path, canceled) == p::PLANNER_SUCCESS,
            "map and cancellation callbacks can reenter both native and route state without deadlock");
    planner.SetMapViewSource([&] {
        planner.SetMapViewSource([&]{ return reader.Read(map, "map"); });
        return reader.Read(map, "map");
    });
    Require(planner.CreatePlan(At(.7,.7), At(5.1,.7), path, {}) == p::PLANNER_CANCELED &&
            !path.poses_size(), "configuration changes cannot publish a path from the old source contract");
    Require(planner.CreatePlan(At(.7,.7), At(5.1,.7), path, canceled) == p::PLANNER_SUCCESS,
            "the next request uses the replacement source and remains operational");
}
void LockedNativeRead() {
    m::Costmap2D map; InitializeMap(map, 10, 10, .1); v::NativeCostmapReader reader;
    std::atomic<bool> stop{false};
    std::thread writer([&]{unsigned char state=m::FREE_SPACE;while(!stop.load()) {
        std::unique_lock<m::Costmap2D::mutex_t> lock(*map.getMutex());
        state=state==m::FREE_SPACE?m::LETHAL_OBSTACLE:m::FREE_SPACE;
        for(unsigned y=0;y<10;++y)for(unsigned x=0;x<10;++x)map.setCost(x,y,state);
    }});
    bool coherent=true;
    try {for(unsigned n=0;n<30;++n) {
        const auto view=reader.Read(map,"map"); const auto expected=view.grid->At(0,0).state;
        for(int y=0;y<10;++y)for(int x=0;x<10;++x)coherent=coherent && view.grid->At(x,y).state==expected;
    }} catch (...) {stop=true;writer.join();throw;}
    stop=true;writer.join();Require(coherent,"native snapshots must not contain torn concurrent updates");
}
}  // namespace
int main() {
    int failures=0;
    for(const auto& test:std::vector<std::pair<const char*,std::function<void()>>>{
        {"native_snapshot",NativeSnapshotOwnership},{"native_global_interface",GlobalPlannerContractAndSharing},
        {"native_door_epoch",DoorFallbackAndOldEpoch},{"native_plugin_factory",NativePluginFactory},
        {"native_locked_read",LockedNativeRead},
        {"native_reentrant_callbacks",ReentrantCallbacksAndConfigurationChange}}) {
        try {test.second();std::cout<<"PASS "<<test.first<<'\n';}
        catch(const std::exception& error){++failures;std::cerr<<"FAIL "<<test.first<<": "<<error.what()<<'\n';}
    }
    return failures?1:0;
}
