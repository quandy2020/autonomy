/* Copyright 2026 The Openbot Authors. */
#include <functional>
#include <iostream>
#include <stdexcept>
#include "autonomy/perception/data/observation_buffer.hpp"
#include "autonomy/perception/observation/lidar_observation_adapter.hpp"
#include "autonomy/perception/observation/rgbd_observation_adapter.hpp"
#include "autonomy/planning/planner/vista/route_planner.hpp"
#include "autonomy/task/exploration/coverage/completion_evaluator.hpp"
#include "autonomy/task/exploration/coverage/coverage_evaluator.hpp"
#include "autonomy/task/exploration/coverage/coverage_fusion.hpp"
#include "autonomy/task/exploration/integration/map_reader.hpp"

namespace p = autonomy::perception;
namespace e = autonomy::task::exploration;
namespace v = autonomy::planning::planner::vista;
namespace n = autonomy::navigation;
namespace {
void Require(bool ok, const std::string& why) { if (!ok) throw std::runtime_error(why); }
void Near(double a,double b,const char* why) { Require(std::abs(a-b)<1e-7,why); }
template<class F> void Reject(F f,const char* why) {
    bool rejected = false; try { f(); } catch (const std::exception&) { rejected = true; }
    Require(rejected,why);
}
n::GridMap OpenMap(int w=30,int h=20,double origin=0) {
    n::GridMap map(w,h,.2,origin,0); map.Fill({n::CellState::kFree,0}); return map;
}
p::SensorObservation Observation(p::SensorKind sensor,std::uint64_t epoch=1,double stamp=1) {
    p::SensorObservation o;
    o.sensor=sensor; o.sequence=1; o.map_epoch=epoch; o.stamp=stamp; o.valid_until=stamp+1;
    o.localization_valid=true; o.valid_fraction=1;
    o.rays.push_back({{0,0,0},{1,0,0},2,true,1,stamp}); return o;
}
void CameraDepthAndMasks() {
    p::RgbdFrame f;
    f.camera={3,1,1,1,1,0,.1,8}; f.depth={1000,0,2000}; f.depth_scale=.001;
    f.localization_valid=true; f.stamp=1;
    auto o=p::RgbdObservationAdapter{}.Adapt(f);
    Require(o && o->Valid(1) && o->rays.size()==2,"camera adapter retains only measured pixels");
    Near(o->rays[0].distance,std::sqrt(2.0),"axial depth is converted to ray range");
    Near(o->rays[0].Endpoint().z,1,"camera depth stays axial Z");
    auto view=p::RgbdObservationAdapter{}.MakeDepthView(f);
    Require(view && !view->valid[1] && view->axial_depth[1]==0,"holes retain invalid mask");
    f.world_from_camera.rotation[0]=2;
    Require(!p::RgbdObservationAdapter{}.Adapt(f),"invalid extrinsic rotation rejected");
}
void LidarMotionCompensation() {
    p::LidarScan scan;
    scan.sequence=1;scan.stamp=1;scan.duration=.1;scan.localization_valid=true;
    scan.beams={{{1,0,0},2,0,true,true,1},{{1,0,0},2,.1,true,true,1},
                {{0,1,0},0,.1,false,false,1}};
    auto o=p::LidarObservationAdapter{}.Adapt(scan,[](double stamp)->std::optional<p::RigidTransform>{
        p::RigidTransform t;t.translation.x=stamp;return t;
    });
    Require(o && o->rays.size()==2 && o->Valid(1.1),"valid timestamped lidar rays survive");
    Near(o->rays[1].origin.x-o->rays[0].origin.x,.1,"each beam uses its own motion-compensated pose");
    auto missing=p::LidarObservationAdapter{}.Adapt(scan,[](double)->std::optional<p::RigidTransform>{return {};});
    Require(missing && missing->rays.empty() && !missing->Valid(1.1),"missing transforms cannot generate free evidence");
}
void ModesHealthAndQueue() {
    for (auto mode:{p::SensorMode::kLidarOnly,p::SensorMode::kRgbdOnly,p::SensorMode::kLidarRgbd}) {
        p::SensorHealthOptions options;options.recovery_hold=0;options.dropout_hold=.2;
        p::SensorHealthMonitor health(mode,options);
        health.Observe(Observation(p::SensorKind::kLidar));health.Observe(Observation(p::SensorKind::kRgbd));
        const auto ready=health.Evaluate(1,p::Mask(mode),{p::Mask(mode)});
        Require(ready.ready && ready.active_sensors==p::Mask(mode),"each configured mode works independently");
        const auto expired=health.Evaluate(2.1,p::Mask(mode),{1,2,3});
        Require(!expired.ready,"dropout hysteresis never authorizes expired geometry");
    }
    p::SensorHealthOptions options;options.recovery_hold=0;
    p::SensorHealthMonitor health(p::SensorMode::kLidarRgbd,options);
    health.Observe(Observation(p::SensorKind::kLidar));
    Require(!health.Evaluate(1,1,{3}).ready,"joint-only controller cannot execute lidar degradation");
    Require(health.Evaluate(1,1,{1,3}).ready,"validated remaining mode and localization can degrade");
    Require(!health.Evaluate(1,0,{1,3}).ready,"messages alone cannot replace localization");
    p::ObservationBuffer buffer(2);buffer.Reset(1);
    auto lidar=Observation(p::SensorKind::kLidar),rgbd=Observation(p::SensorKind::kRgbd);
    Require(buffer.Push(lidar,1) && !buffer.Push(lidar,1),"duplicate sequence rejected");
    Require(buffer.Push(rgbd,1),"sensor sequence namespaces are independent");
    lidar.sequence=2;Require(!buffer.Push(lidar,1),"overflow rejects explicitly instead of silently losing coverage");
    Require(buffer.Drain(1,1).size()==2,"both modalities drain once");
    buffer.Reset(2);Require(!buffer.Push(lidar,1),"previous epoch observation rejected");
}
void MapVersionsAndRollingHistory() {
    e::MapReader reader;
    auto map=OpenMap(10,10);map.Set(2,2,{n::CellState::kOccupied,0});
    auto a=reader.Read(map,1,0,1);
    auto b=reader.Read(map,2,0,2);
    Require(a.geometry_revision==b.geometry_revision && a.grid==b.grid && b.dirty_regions.empty(),
            "publication is not a geometry change");
    map.Set(2,2,{n::CellState::kFree,0});auto c=reader.Read(map,3,0,3);
    Require(c.geometry_revision>b.geometry_revision && a.grid->At(2,2).state==n::CellState::kOccupied,
            "snapshot owns immutable content");
    auto roll=OpenMap(10,10,1);
    auto d=reader.Read(roll,4,0,4);
    Require(d.map_epoch==c.map_epoch && d.grid->width()==15,"rolling history is preserved on a common lattice");
    auto previous=d.grid->WorldToCell({.1,.1,0});
    Require(previous && d.grid->At(previous->x,previous->y).state==n::CellState::kFree,
            "old visited space is not deleted when window moves");
    auto reset=reader.Read(roll,5,8,5);
    Require(reset.map_epoch>d.map_epoch && reset.grid->width()==10,"alignment epoch invalidates old history");
    Reject([&]{reader.Read(roll,4,8,4);},"old map result rejected");
    e::MapReader small(100);
    small.Read(OpenMap(10,10),1,0,1);
    Reject([&]{small.Read(roll,2,0,2);},"history expansion respects capacity");
    Require(small.latest()->grid->width()==10,"failed update leaves previous snapshot intact");
}
void ActualCoverageAndDeduplication() {
    e::CoverageState state;state.Reset(1);
    e::CoverageUnit unit;unit.id={2,0,0,0,e::CoverageKind::kSurface};unit.center={2,0,0};unit.area=.2;
    state.Upsert(unit);
    std::vector<e::CoverageContribution> c{{unit.id,p::SensorKind::kLidar,.9},
                                         {unit.id,p::SensorKind::kRgbd,.8},
                                         {unit.id,p::SensorKind::kLidar,.9}};
    // Raw .9 saturates at required .8, reserving the full .2 area.
    Near(e::CoverageFusion::Gain(state,c),.2,"multisensor and point-density duplicates are not summed");
    auto selected=e::CoverageFusion::Fuse(c);
    Near(e::CoverageFusion::Gain(state,c,selected),0,"selected viewpoints use marginal union gain");
    Require(state.PendingCount()==1,"prediction or arrival does not confirm coverage");
    auto actual=Observation(p::SensorKind::kRgbd);
    actual.rays[0].hit=false;
    e::CoverageEvaluator evaluator;
    Require(evaluator.Apply(actual,1,state).confirmed_units==0,"no-return ray cannot complete surface observation");
    actual.rays[0].hit=true;
    Require(evaluator.Apply(actual,1,state).confirmed_units==1 && state.Complete(unit.id),"actual measured surface completes");
    e::CoverageUnit frontier;
    frontier.id={6,0,0,0,e::CoverageKind::kFrontier};
    frontier.center={2,0,0};
    state.Upsert(frontier);
    actual.rays[0].hit=false;
    Require(evaluator.Apply(actual,1,state).confirmed_units==0 && state.PendingCount()==1,
            "a no-return ray cannot resolve a frontier without moving or revealing its cell");
    actual.rays[0].hit=true;
    Require(evaluator.Apply(actual,1,state).confirmed_units==1 && state.Complete(frontier.id),
            "a measured frontier return resolves the frontier");
    Require(evaluator.Apply(actual,1,state).realized_gain==0,"repeated frame does not add coverage");
    unit.id.x=3;unit.center={2,0,0};unit.required_sensors=3;state.Upsert(unit);
    evaluator.Apply(actual,1,state);
    Require(!state.Complete(unit.id),"joint sensor quality requirements survive missing lidar");
    evaluator.Apply(Observation(p::SensorKind::kLidar),1,state);
    Require(state.Complete(unit.id),"each required sensor contributes actual evidence");
    unit.id.x=4;unit.center={3,0,0};unit.required_sensors=0;state.Upsert(unit);
    evaluator.Apply(actual,1,state);
    Require(!state.Complete(unit.id),"occluded point behind first return stays pending");
}
/**
 * @brief Checks quality improvement against measured evidence and selected views.
 *
 * Predictions use the same completion scale as actual observations. A worse
 * repeated view must have zero gain, and ANY_SENSOR evidence must not satisfy a
 * PER_SENSOR requirement. Prediction and selection must not mutate the ledger.
 */
void QualityImprovementAndIndependentSources() {
    e::CoverageState state;
    state.Reset(7);
    e::CoverageUnit unit;
    unit.id = {5, 0, 0, 0, e::CoverageKind::kSurface};
    unit.required_quality = .8;
    state.Upsert(unit);
    Require(state.Confirm(unit.id, p::SensorKind::kRgbd, .6, 1),
            "fixture records measured partial quality");
    const auto revision = state.revision();
    Near(e::CoverageFusion::Gain(state, {{unit.id, p::SensorKind::kRgbd, .4}}), 0,
         "a worse repeated view cannot improve measured coverage");
    Near(e::CoverageFusion::Gain(state, {{unit.id, p::SensorKind::kLidar, .7}}), .125,
         "either-source gain is normalized quality improvement");
    Near(e::CoverageFusion::Gain(state, {{unit.id, p::SensorKind::kLidar, 1}}), .25,
         "predicted improvement saturates at the task quality target");
    e::CoverageSelection selected;
    e::CoverageFusion::Accumulate({{unit.id, p::SensorKind::kLidar, .7}}, selected);
    Near(e::CoverageFusion::MarginalGain(state, {{unit.id, p::SensorKind::kRgbd, .65}}, selected), 0,
         "a lower-quality selected overlap has no additional gain");
    Near(e::CoverageFusion::MarginalGain(state, {{unit.id, p::SensorKind::kRgbd, .8}}, selected), .125,
         "selected union reserves only its own predicted improvement");
    Require(state.revision() == revision && !state.Complete(unit.id) &&
                state.units().at(unit.id).observed_quality[0] == 0,
            "prediction and selection never become measured evidence");

    unit.id.x = 6;
    unit.required_sensors = p::Mask(p::SensorMode::kLidarRgbd);
    state.Upsert(unit);
    state.Confirm(unit.id, p::SensorKind::kLidar, .9, 1);
    state.Confirm(unit.id, p::SensorKind::kRgbd, .2, 1);
    Near(e::CoverageFusion::Gain(state, {{unit.id, p::SensorKind::kLidar, 1},
                                       {unit.id, p::SensorKind::kRgbd, .1}}), 0,
         "complete lidar cannot reward a worse pending camera observation");
    Near(e::CoverageFusion::Gain(state, {{unit.id, p::SensorKind::kLidar, 1},
                                       {unit.id, p::SensorKind::kRgbd, .6}}), .25,
         "camera improvement retains its independent requirement weight");
    selected = {};
    e::CoverageFusion::Accumulate({{unit.id, p::SensorKind::kLidar, 1},
                                  {unit.id, p::SensorKind::kRgbd, .6}}, selected);
    Near(e::CoverageFusion::MarginalGain(state, {{unit.id, p::SensorKind::kRgbd, .8}}, selected), .125,
         "per-source selected maxima do not erase remaining camera work");
    Require(!state.Complete(unit.id), "predicted camera completion leaves the real ledger pending");

    const auto invalid_sensor = static_cast<p::SensorKind>(3);
    const auto invalid_kind = static_cast<e::CoverageKind>(99);
    const auto before_invalid = state.revision();
    Require(!state.Confirm(unit.id, invalid_sensor, 1, 2),
            "invalid source cannot silently populate the camera ledger");
    Reject([&] { e::CoverageFusion::Gain(state, {{unit.id, invalid_sensor, .8}}); },
           "predicted source identity is validated");
    auto invalid_id = unit.id;
    invalid_id.kind = invalid_kind;
    Reject([&] { e::CoverageFusion::Gain(state, {{invalid_id, p::SensorKind::kRgbd, .8}}); },
           "invalid category cannot index outside the reference scales");
    unit.id.kind = invalid_kind;
    Reject([&] { state.Upsert(unit); }, "ledger rejects an unsupported category");
    const auto reserved_geometry = selected.geometry;
    const auto reserved_sources = selected.sources;
    Reject([&] { e::CoverageFusion::Accumulate(
        {{reserved_geometry.begin()->first, p::SensorKind::kRgbd, 1},
         {invalid_id, p::SensorKind::kRgbd, .8}}, selected); },
        "invalid contribution rejects the full selection batch");
    Require(selected.geometry == reserved_geometry && selected.sources == reserved_sources &&
                state.revision() == before_invalid,
            "invalid batch cannot partially mutate reservations or actual evidence");
}
void CompletionProtocolAndResults() {
    e::CompletionEvaluator evaluator(3);e::CompletionEvidence evidence;
    evidence.map_epoch=1;evidence.geometry_revision=2;evidence.sweep_revision=2;
    evidence.sensing_healthy=evidence.localization_healthy=evidence.observation_queue_empty=true;
    evidence.grid_fallback_checked=evidence.sweep_checked=true;
    evidence.map_watermark=evidence.graph_watermark=evidence.observation_watermark=5;
    evidence.confirmation_round=1;
    Require(!evaluator.Evaluate(evidence).complete,"one empty candidate cycle is insufficient");
    Require(evaluator.Evaluate(evidence).stable_rounds==1,"same tick cannot fake stable rounds");
    evidence.confirmation_round=2;
    Require(evaluator.Evaluate(evidence).stable_rounds==1,"new round with old observations cannot advance completion");
    evidence.map_watermark=evidence.graph_watermark=evidence.observation_watermark=6;
    Require(!evaluator.Evaluate(evidence).complete,"second fresh round waits");
    evidence.confirmation_round=3;
    evidence.map_watermark=evidence.graph_watermark=evidence.observation_watermark=7;
    Require(evaluator.Evaluate(evidence).complete,"consistent fresh evidence completes");
    evidence.sensing_healthy=false;Require(!evaluator.Evaluate(evidence).complete,"blindness is not complete exploration");
    evidence.sensing_healthy=true;evidence.pending_revisits=1;
    Require(!evaluator.Evaluate(evidence).complete,"low-gain revisit prevents completion");
    evidence.pending_revisits=0;evidence.graph_watermark=4;
    Require(!evaluator.Evaluate(evidence).complete,"pending graph updates prevent completion");
    e::ExplorationResult result;result.exploration=e::ExplorationOutcome::kCompleted;
    result.returned=e::ReturnOutcome::kReturnFailed;result.saved=e::MapSaveOutcome::kSaved;
    Require(!result.Success(true,true) && result.Success(false,true),"coverage, return and save results remain separate");
    result.exploration=e::ExplorationOutcome::kPartialBudget;
    Require(!result.Success(false,true),"saved partial map is not full success");
}
void SourceFreshnessAndBoundedDrain() {
    e::CompletionEvaluator evaluator(2); e::CompletionEvidence evidence;
    evidence.map_epoch = evidence.geometry_revision = evidence.sweep_revision = 1;
    evidence.sensing_healthy = evidence.localization_healthy = evidence.observation_queue_empty = true;
    evidence.grid_fallback_checked = evidence.sweep_checked = evidence.mapping_synchronized = true;
    evidence.require_source_cuts = true;
    evidence.required_observation_sources = p::Mask(p::SensorMode::kLidarRgbd);
    evidence.observation_cut.Include(p::SensorKind::kLidar, 10);
    evidence.observation_cut.Include(p::SensorKind::kRgbd, 20);
    evidence.map_cut = evidence.graph_cut = evidence.valid_observation_cut = evidence.observation_cut;
    evidence.confirmation_round = 1;
    Require(evaluator.Evaluate(evidence).stable_rounds == 1, "first valid joint source round counts once");
    evidence.confirmation_round = 2;
    Require(evaluator.Evaluate(evidence).stable_rounds == 1, "different round id with the same source cuts cannot advance");
    evidence.observation_cut.Include(p::SensorKind::kLidar, 11);
    evidence.map_cut = evidence.graph_cut = evidence.valid_observation_cut = evidence.observation_cut;
    Require(!evaluator.Evaluate(evidence).complete, "only one required source advancing is insufficient");
    evidence.observation_cut.Include(p::SensorKind::kRgbd, 21);
    evidence.map_cut = evidence.graph_cut = evidence.observation_cut;
    Require(!evaluator.Evaluate(evidence).complete, "accepted bad frames do not advance valid sensing evidence");
    evidence.valid_observation_cut = evidence.observation_cut;
    Require(evaluator.Evaluate(evidence).complete, "every required source contributes new valid fused evidence");

    p::ObservationBuffer buffer(4); buffer.Reset(1);
    for (unsigned sequence = 1; sequence <= 3; ++sequence) {
        auto observation = Observation(p::SensorKind::kRgbd, 1, 1);
        observation.sequence = sequence;
        observation.valid_until = sequence == 1 ? 1.5 : 3;
        Require(buffer.Push(observation, 1), "bounded drain fixture accepts ordered observations");
    }
    Require(buffer.Drain(2, 1, 1).empty() && !buffer.Empty(), "expired input consumes only one processing slot");
    Require(buffer.processed_cut().sequence[1] == 1 && buffer.discarded_cut().sequence[1] == 1 &&
            buffer.accepted_cut().sequence[1] == 3, "accepted, processed and discarded evidence are distinct");
    Require(buffer.Drain(2, 1, 1).size() == 1 && !buffer.Empty(), "bounded drain preserves its remaining observations");
    Require(buffer.Drain(2, 1, 1).size() == 1 && buffer.Empty() &&
            buffer.processed_cut().Covers(buffer.accepted_cut()), "all examined inputs eventually reach the processed cut");
}
void IncrementalLongEdgesAndReconnection() {
    auto map=OpenMap(45,30);
    // Two obstacles generate separated contour vertices and long visibility edges.
    for(int y=8;y<=11;++y)for(int x=8;x<=11;++x)map.Set(x,y,{n::CellState::kOccupied,0});
    for(int y=18;y<=21;++y)for(int x=30;x<=33;++x)map.Set(x,y,{n::CellState::kOccupied,0});
    e::MapReader reader;n::SafetyOptions options;options.robot_radius=.05;options.horizontal_margin=0;
    v::VistaRoutePlanner planner(options,150);
    auto view=reader.Read(map,1,0,1);planner.Update(view);auto before=planner.GraphSnapshot();
    std::optional<v::VisibilityEdge> crossing;n::Pose2d midpoint;
    for(const auto& item:before.edges) {
        const auto& edge=item.second;if(!edge.valid || edge.length<4)continue;
        const auto& a=before.nodes.at(edge.id.first).pose;const auto& b=before.nodes.at(edge.id.second).pose;
        n::Pose2d m{(a.x+b.x)/2,(a.y+b.y)/2,0};auto cell=map.WorldToCell(m);
        if(cell && cell->x>5 && cell->x<39 && cell->y>4 && cell->y<25 && map.At(cell->x,cell->y).state==n::CellState::kFree) {
            crossing=edge;midpoint=m;break;
        }
    }
    Require(crossing.has_value(),"test map supplies a long edge whose endpoints are outside changed region");
    const auto cell=map.WorldToCell(midpoint).value();map.Set(cell.x,cell.y,{n::CellState::kOccupied,0});
    auto blocked=reader.Read(map,2,0,2);auto changed=planner.Update(blocked);auto after=planner.GraphSnapshot();
    Require(changed.invalidated_edges>0 && after.edges.count(crossing->id) && !after.edges.at(crossing->id).valid,
            "edge spatial index finds obstacle crossing long edge away from endpoints");
    Require(after.nodes.at(crossing->id.first).id==before.nodes.at(crossing->id.first).id,"unaffected node keeps stable id");
    map.Set(cell.x,cell.y,{n::CellState::kFree,0});auto reopened=reader.Read(map,3,0,3);
    auto restored=planner.Update(reopened);auto graph=planner.GraphSnapshot();
    Require(restored.reconnected_edges>0 && graph.edges.at(crossing->id).valid,"removed obstacle reopens blocked edges");
    auto unchanged=reader.Read(map,4,0,4);auto stats=planner.Update(unchanged);
    Require(stats.revalidated_edges==0 && planner.GraphSnapshot().observation_watermark==4,
            "unchanged geometry only advances evidence watermark");
    auto route=planner.CreateRoute({.7,.7,0},{8.1,5.1,0});
    Require(route.success && !route.execution_verified,"route success does not grant dynamic/3D execution permission");
    auto canceled=planner.CreateRoute({.7,.7,0},{8.1,5.1,0},[]{return true;});
    Require(canceled.canceled && !canceled.success,"canceled queries cannot publish stale path");
    auto aligned=reader.Read(map,5,9,5);planner.Update(aligned);
    Require(planner.GraphSnapshot().map_epoch!=before.map_epoch,"loop alignment replaces graph epoch");
    const auto aligned_graph=planner.GraphSnapshot();
    planner.Update(reopened);
    Require(planner.GraphSnapshot().map_epoch==aligned_graph.map_epoch &&
            planner.GraphSnapshot().geometry_revision==aligned_graph.geometry_revision,
            "late old-epoch maps cannot roll back the current visibility graph");
}
void GridFallbackAndNarrowDoor() {
    auto map=OpenMap(30,20);
    for(int y=0;y<20;++y) if(y<8 || y>11) map.Set(15,y,{n::CellState::kOccupied,0});
    e::MapReader reader;n::SafetyOptions options;options.robot_radius=.1;options.horizontal_margin=.01;
    v::VistaRoutePlanner planner(options,1);planner.Update(reader.Read(map,1,0,1));
    auto route=planner.CreateRoute({.7,.7,0},{5.1,.7,0});
    Require(route.success && route.used_grid_fallback,
            "sparse graph failure falls back to known-space connectivity success=" + std::to_string(route.success) +
            " fallback=" + std::to_string(route.used_grid_fallback));
    n::GeometrySafety safety(reader.latest()->grid,options);
    Require(safety.VerifyPath(route.path).safe,"narrow-door fallback route respects footprint once");
    for(int y=8;y<=11;++y)map.Set(15,y,{n::CellState::kOccupied,0});
    planner.Update(reader.Read(map,2,0,2));
    Require(!planner.CreateRoute({.7,.7,0},{5.1,.7,0}).success,"sealed wall is unreachable in both graph and grid");
}
}  // namespace
int main() {
    int failures=0;
    for(const auto& test:std::vector<std::pair<std::string,std::function<void()>>>{
        {"camera_units_masks",CameraDepthAndMasks},{"lidar_motion",LidarMotionCompensation},
        {"sensor_modes_health_queue",ModesHealthAndQueue},{"map_versions_rolling",MapVersionsAndRollingHistory},
        {"actual_coverage_dedup",ActualCoverageAndDeduplication},{"completion_protocol",CompletionProtocolAndResults},
        {"quality_improvement_sources",QualityImprovementAndIndependentSources},
        {"source_freshness_bounded_drain",SourceFreshnessAndBoundedDrain},
        {"long_edge_update_reconnect",IncrementalLongEdgesAndReconnection},{"grid_fallback_narrow_door",GridFallbackAndNarrowDoor}}) {
        try {test.second();std::cout<<"PASS "<<test.first<<'\n';}
        catch(const std::exception& error){++failures;std::cerr<<"FAIL "<<test.first<<": "<<error.what()<<'\n';}
    }
    return failures?1:0;
}
