// SPDX-License-Identifier: Apache-2.0
/** @file coverage_test.cpp
 * @brief Native complete-coverage coverage test implementation. */
#include "autonomy/task/navigation/coverage/coverage_server.hpp"
#include "autonomy/task/navigation/coverage/region_orderer.hpp"
#include "autonomy/task/navigation/coverage/region_segmenter.hpp"
#include "autonomy/task/navigation/coverage/native_execution.hpp"
#include "autonomy/task/navigation/coverage/path_segments.hpp"
#include "autonomy/task/navigation/coverage/bt/coverage_nodes.hpp"
#include "autonomy/task/navigation/coverage/tests/fake_execution.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <functional>
#include <cmath>
#include <cstdlib>

using namespace autonomy::task::coverage;
void Require(bool value,const std::string& message){if(!value)throw std::runtime_error(message);}
proto::CoverageRequest Rectangle(bool hole=false) {
  proto::CoverageRequest r;r.set_frame_id("map");
  auto add=[&](std::initializer_list<std::pair<double,double>> points){auto* ring=r.add_polygons();for(auto [x,y]:points){auto* p=ring->add_coordinates();p->set_axis1(x);p->set_axis2(y);}};
  add({{0,0},{40,0},{40,30},{0,30},{0,0}});
  if(hole)add({{16,12},{16,17},{22,17},{22,12},{16,12}});
  return r;
}
proto::CoverageRequest TwoRoomMap() {
  proto::CoverageRequest r;
  r.set_frame_id("map");
  r.set_coordinates_in_cartesian_frame(true);
  r.set_generate_regions(true);
  r.set_robot_radius(0.25);
  r.mutable_headland_mode()->set_mode("CONSTANT");
  r.mutable_headland_mode()->set_width(0.6);
  auto* ring = r.add_polygons();
  for (auto [x, y] : {std::pair{0.3, 0.3}, std::pair{7.7, 0.3},
                      std::pair{7.7, 3.7}, std::pair{0.3, 3.7},
                      std::pair{0.3, 0.3}}) {
    auto* p = ring->add_coordinates();
    p->set_axis1(x);
    p->set_axis2(y);
  }
  auto* map = r.mutable_map();
  map->set_frame_id("map");
  map->set_resolution(0.1);
  map->set_width(80);
  map->set_height(40);
  map->set_origin_x(0.0);
  map->set_origin_y(0.0);
  map->set_occupied_threshold(65);
  for (int row = 0; row < 40; ++row) {
    for (int column = 0; column < 80; ++column) {
      const bool border =
          row == 0 || row == 39 || column == 0 || column == 79;
      // Wall at x = 4 with a 1.0 m door near the bottom of the map.
      const bool wall =
          column >= 39 && column <= 41 && !(row >= 2 && row < 12);
      map->add_data(border || wall ? 100 : 0);
    }
  }
  return r;
}
void RequireInsideRooms(const ComputeAction::Result& out) {
  for (const auto& pose : out.nav_path().poses()) {
    const double x = pose.pose().position().x();
    const double y = pose.pose().position().y();
    Require(x > 0.0 && x < 8.0 && y > 0.0 && y < 4.0,
            "Global path leaves the map");
    Require(!(x > 3.6 && x < 4.4 && !(y > 0.2 && y < 1.2)),
            "Global path crosses the dividing wall");
  }
}

void Success(const ComputeAction::Result& result){Require(result.error_code()==proto::NONE,"Planning failed: "+result.error_msg());Require(result.coverage_path().swaths_size()>0,"Missing swaths");}
int main(int argc,char** argv) {
  std::vector<std::pair<std::string,std::function<void()>>> tests;
  auto add=[&](std::string name,std::function<void()> test){tests.emplace_back(std::move(name),std::move(test));};
  CoverageServer server;
  add("debug_rooms",[&]{Configuration tight;tight.robot_width=0.3;tight.operation_width=0.5;tight.min_turning_radius=0.3;tight.headland_width=0.6;CoverageServer map_server(tight);auto r=TwoRoomMap();auto out=map_server.Compute(r);std::cout<<"REGIONS "<<out.regions_size()<<std::endl;for(const auto& reg:out.regions()){std::cout<<"REGION "<<reg.name()<<" area="<<reg.area()<<" polys="<<reg.polygons_size()<<std::endl;for(const auto& poly:reg.polygons()){std::cout<<" POLY "<<poly.coordinates_size()<<":";for(const auto& p:poly.coordinates())std::cout<<" ("<<p.axis1()<<","<<p.axis2()<<")";std::cout<<std::endl;}}for(const auto& w:out.warnings())std::cout<<"WARN "<<w<<std::endl;});
  add("complete_rect",[&]{auto result=server.Compute(Rectangle());Success(result);Require(result.core_coverage()>.95,"Coverage below 95%");Require(result.nav_path().poses_size()>0,"Missing nav path");});
  add("complete_hole",[&]{auto result=server.Compute(Rectangle(true));Success(result);Require(result.core_coverage()>.95,"Hole core coverage below 95%");});
  add("robot_radius_drives_swath",[&]{auto r=Rectangle();r.set_robot_radius(0.3);auto out=server.Compute(r);Success(out);Require(out.coverage_path().swaths_size()>1,"Radius did not raise swath count");for(const auto& s:out.coverage_path().swaths())Require(std::abs(s.width()-0.6)<1e-9,"Swath width is not twice the radius");});
  add("invalid_robot_radius",[&]{auto r=Rectangle();r.set_robot_radius(0);Require(server.Compute(r).error_code()==proto::INVALID_REQUEST,"Zero radius accepted");r.set_robot_radius(-1);Require(server.Compute(r).error_code()==proto::INVALID_REQUEST,"Negative radius accepted");r.set_robot_radius(NAN);Require(server.Compute(r).error_code()==proto::INVALID_REQUEST,"NaN radius accepted");});
  add("region_tsp",[]{std::vector<std::pair<double,double>> points{{0,0},{10,0},{10,10},{0,10}};const auto result=RegionOrderer::Order(points,0,0);Require(result.order.size()==4,"TSP dropped rooms");Require(std::abs(result.distance-30.0)<1e-6,"TSP distance should be 30 m");});
  add("region_split",[&]{auto r=Rectangle();r.set_generate_regions(true);r.set_region_columns(2);r.set_region_rows(2);auto out=server.Compute(r);Require(out.error_code()==proto::NONE,std::string("Region split failed: ")+out.error_msg());Require(out.regions_size()==4,"Expected four regions");double area=0;for(const auto& region:out.regions()){area+=region.area();Require(region.states_size()>0,"Region has no coverage states");Require(region.polygons_size()>0,"Region has no polygon");}Require(std::abs(area-1200.0)<1.0,"Region areas do not sum to the field");});
  add("region_tsp_matrix",[]{std::vector<std::vector<double>> c{{0,1,9},{1,0,1},{9,1,0}};std::vector<double> s{0,5,5};const auto r=RegionOrderer::Order(c,s);Require(r.order.size()==3,"Matrix TSP dropped rooms");Require(r.order[0]==0&&r.order[1]==1&&r.order[2]==2,"Matrix TSP order wrong");Require(std::abs(r.distance-2.0)<1e-9,"Matrix TSP cost wrong");});
  add("swath_spacing_override",[&]{auto r=Rectangle();r.set_robot_radius(0.4);r.set_swath_spacing(1.0);auto out=server.Compute(r);Success(out);for(const auto& s:out.coverage_path().swaths())Require(std::abs(s.width()-1.0)<1e-9,"Swath width ignores swath_spacing");auto wide=Rectangle();wide.set_robot_radius(0.4);wide.set_swath_spacing(2.0);auto wide_out=server.Compute(wide);Success(wide_out);Require(out.coverage_path().swaths_size()>wide_out.coverage_path().swaths_size(),"Tighter spacing should produce more swaths");});
  add("invalid_swath_spacing",[&]{auto r=Rectangle();r.set_swath_spacing(0);Require(server.Compute(r).error_code()==proto::INVALID_REQUEST,"Zero spacing accepted");r.set_swath_spacing(-1);Require(server.Compute(r).error_code()==proto::INVALID_REQUEST,"Negative spacing accepted");});
  add("map_rooms_connect_through_door",[&]{Configuration tight;tight.robot_width=0.3;tight.operation_width=0.5;tight.min_turning_radius=0.3;tight.headland_width=0.6;CoverageServer map_server(tight);auto r=TwoRoomMap();auto out=map_server.Compute(r);Require(out.error_code()==proto::NONE,std::string("Map plan failed: ")+out.error_msg());Require(out.regions_size()==2,"Expected two rooms");Require(out.nav_path().poses_size()>0,"Missing global path");RequireInsideRooms(out);Require(out.travel_distance()>4.0,"Travel distance ignores the door detour");});
  add("flags_path_requires_route",[&]{auto r=Rectangle();r.set_generate_route(false);Require(server.Compute(r).error_code()==proto::INVALID_REQUEST,"Bad flag combination accepted");});
  add("swaths_only",[&]{auto r=Rectangle();r.set_generate_route(false);r.set_generate_path(false);auto out=server.Compute(r);Success(out);Require(!out.coverage_path().swaths_ordered()&&out.nav_path().poses().empty(),"Wrong stage output");});
  add("route_only",[&]{auto r=Rectangle();r.set_generate_path(false);auto out=server.Compute(r);Success(out);Require(out.coverage_path().swaths_ordered()&&!out.coverage_path().contains_turns(),"Wrong route output");});
  add("no_headland",[&]{auto r=Rectangle();r.set_generate_headland(false);r.set_generate_path(false);Success(server.Compute(r));});
  add("empty_input",[&]{Require(server.Compute({}).error_code()==proto::INVALID_REQUEST,"Empty field accepted");});
  add("nonfinite_input",[&]{auto r=Rectangle();r.mutable_polygons(0)->mutable_coordinates(0)->set_axis1(NAN);Require(server.Compute(r).error_code()==proto::INVALID_REQUEST,"NaN accepted");});
  add("self_intersection",[&]{auto r=Rectangle();auto* ring=r.mutable_polygons(0);ring->mutable_coordinates(1)->set_axis1(40);ring->mutable_coordinates(1)->set_axis2(30);ring->mutable_coordinates(2)->set_axis1(40);ring->mutable_coordinates(2)->set_axis2(0);Require(server.Compute(r).error_code()==proto::INVALID_REQUEST,"Self-intersecting polygon accepted");});
  add("collapsed_headland",[&]{auto r=Rectangle();r.mutable_headland_mode()->set_mode("CONSTANT");r.mutable_headland_mode()->set_width(100);auto out=server.Compute(r);Require(out.error_code()!=proto::NONE&&out.nav_path().poses().empty(),"Collapsed field accepted");});
  add("cancel_before_start",[&]{Require(server.Compute(Rectangle(),[]{return true;}).error_code()==proto::CANCELED,"Cancel ignored");});
  add("cancel_between_stages",[&]{bool canceled=false;auto out=server.Compute(Rectangle(),[&]{return canceled;},[&](const auto& stage,double){if(stage=="swaths")canceled=true;});Require(out.error_code()==proto::CANCELED&&out.nav_path().poses().empty(),"Partial plan leaked after cancellation");});
  for(const std::string mode:{"BOUSTROPHEDON","SNAKE","SPIRAL","TSP"})add("route_"+mode,[&,mode]{auto r=Rectangle();r.mutable_route_mode()->set_mode(mode);r.mutable_route_mode()->set_spiral_n(4);r.mutable_route_mode()->set_tsp_d_tol(1e-4);r.mutable_route_mode()->set_tsp_time_limit(1);r.mutable_route_mode()->set_tsp_redirect_swaths(true);Success(server.Compute(r));});
  add("custom_route",[&]{auto r=Rectangle();r.set_generate_path(false);const auto baseline=server.Compute(r);Success(baseline);r.mutable_route_mode()->set_mode("CUSTOM");for(unsigned i=0;i<static_cast<unsigned>(baseline.coverage_path().swaths_size());++i)r.mutable_route_mode()->add_custom_order(i);Success(server.Compute(r));});
  add("bad_custom_permutation",[&]{auto r=Rectangle();r.mutable_route_mode()->set_mode("CUSTOM");r.mutable_route_mode()->add_custom_order(0);Require(server.Compute(r).error_code()==proto::INVALID_MODE_SET,"Invalid custom order accepted");});
  for(const std::string objective:{"LENGTH","NUMBER","NUMBER_MODIFIED","COVERAGE"})add("objective_"+objective,[&,objective]{auto r=Rectangle();r.set_generate_path(false);r.mutable_swath_mode()->set_mode("SET_ANGLE");r.mutable_swath_mode()->set_objective(objective);Success(server.Compute(r));});
  add("incomplete_swath_mode",[&]{auto r=Rectangle();r.mutable_swath_mode()->set_mode("SET_ANGLE");Require(server.Compute(r).error_code()==proto::INVALID_MODE_SET,"Partial override accepted");});
  add("invalid_step_angle",[&]{auto r=Rectangle();r.mutable_swath_mode()->set_mode("BRUTE_FORCE");r.mutable_swath_mode()->set_objective("NUMBER");r.mutable_swath_mode()->set_step_angle(0);Require(server.Compute(r).error_code()==proto::INVALID_REQUEST,"Zero step accepted");});
  add("invalid_path_mode",[&]{auto r=Rectangle();r.mutable_path_mode()->set_mode("BAD");r.mutable_path_mode()->set_continuity_mode("DISCONTINUOUS");Require(server.Compute(r).error_code()==proto::INVALID_MODE_SET,"Invalid path mode accepted");});
  add("reeds_shepp",[&]{auto r=Rectangle();r.mutable_path_mode()->set_mode("REEDS_SHEPP");r.mutable_path_mode()->set_continuity_mode("DISCONTINUOUS");auto out=server.Compute(r);Success(out);Require(out.coverage_path().velocities_size()==out.nav_path().poses_size()&&out.coverage_path().is_backward_size()==out.nav_path().poses_size(),"Pose metadata length mismatch");});
  for(const std::string mode:{"TRAPEZOIDAL","BOUSTROPHEDON","NONE"})add("decomposition_"+mode,[&,mode]{auto r=Rectangle();r.set_generate_decomp(true);r.set_generate_path(false);r.mutable_decomp_mode()->set_mode(mode);Success(server.Compute(r));});
  add("tsp_start_pose",[&]{auto r=Rectangle();r.mutable_route_mode()->set_mode("TSP");r.mutable_route_mode()->set_tsp_d_tol(1e-4);r.mutable_route_mode()->set_tsp_redirect_swaths(true);r.set_use_start_pose(true);r.mutable_start_pose()->set_axis1(4);r.mutable_start_pose()->set_axis2(4);Success(server.Compute(r));});
  add("non_tsp_start_rejected",[&]{auto r=Rectangle();r.set_use_start_pose(true);Require(server.Compute(r).error_code()==proto::INVALID_REQUEST,"Ignored start pose silently");});
  add("headland_perimeter",[&]{auto r=Rectangle();r.set_generate_headland_swaths(true);Success(server.Compute(r));});
  add("headland_first",[&]{auto r=Rectangle();r.set_generate_headland_swaths(true);r.set_headland_first(true);Success(server.Compute(r));});
  add("headland_full_band",[&]{auto r=Rectangle();r.set_generate_headland_swaths(true);r.set_headland_full_coverage(true);r.mutable_headland_mode()->set_mode("CONSTANT");r.mutable_headland_mode()->set_width(6);Success(server.Compute(r));});
  add("gps_wrong_frame",[&]{auto r=Rectangle();r.set_coordinates_in_cartesian_frame(false);Require(server.Compute(r).error_code()==proto::INVALID_REQUEST,"Invalid GPS frame accepted");});
  add("metric_reference",[&]{auto r=Rectangle();for(auto& ring:*r.mutable_polygons())for(auto& p:*ring.mutable_coordinates()){p.set_axis1(p.axis1()+120);p.set_axis2(p.axis2()+240);}auto out=server.Compute(r);Success(out);for(const auto& p:out.nav_path().poses())Require(p.pose().position().x()>=120&&p.pose().position().y()>=240,"Reference point not restored");});
  add("segments_keep_geometry",[&]{auto out=server.Compute(Rectangle());Success(out);auto segments=BuildSegments(out,40,false);double length=0;for(const auto& s:segments){Require(s.path.poses_size()<=40,"Transport budget exceeded");length+=s.length;}double expected=0;for(const auto& s:out.coverage_path().states())expected+=s.len();Require(std::abs(length-expected)<1e-6,"Segmentation changed path length");});
  add("reverse_requires_contract",[&]{auto out=server.Compute(Rectangle());Success(out);out.mutable_coverage_path()->mutable_states(0)->set_direction(-1);bool rejected=false;try{BuildSegments(out,40,false);}catch(const CoverageException&){rejected=true;}Require(rejected,"Reverse direction lost");Require(!BuildSegments(out,40,true).empty(),"Reverse-capable segmentation failed");});
  for(const std::string mode:{"CENTER","OFFSET","ROWSARESWATHS"})add("rows_"+mode,[&,mode]{auto r=Rectangle();r.set_generate_headland(false);r.set_generate_path(false);r.mutable_row_swath_mode()->set_mode(mode);r.mutable_row_swath_mode()->set_offset(1);for(int i=0;i<3;++i){auto* row=r.add_rows();row->set_id(i);for(double x:{4.,36.}){auto* p=row->add_coordinates();p->set_axis1(x);p->set_axis2(5.+i*5);}}auto out=server.Compute(r);Success(out);Require(out.coverage_path().swaths_size()==(mode=="ROWSARESWATHS"?3:2),"Wrong row count");});
  add("bt_compute_approach_follow",[]{BT::BehaviorTreeFactory factory;RegisterCoverageNodes(factory);auto board=BT::Blackboard::create();auto ctx=std::make_shared<FakeExecution>();board->set<std::shared_ptr<Execution>>(kExecutionBlackboard,ctx);auto tree=factory.createTreeFromText("<root BTCPP_format='4'><BehaviorTree ID='Coverage'><Sequence><ComputeCoveragePath/><NavigateToCoverageStart/><FollowCoveragePath/></Sequence></BehaviorTree></root>",board);BT::NodeStatus s=BT::NodeStatus::RUNNING;for(int i=0;i<5&&s==BT::NodeStatus::RUNNING;++i)s=tree.tickOnce();Require(s==BT::NodeStatus::SUCCESS&&ctx->compute==1&&ctx->approach==1&&ctx->follow==3,"BT ordering/repetition wrong");});
  add("bt_failure_stops_motion",[]{BT::BehaviorTreeFactory factory;RegisterCoverageNodes(factory);auto board=BT::Blackboard::create();auto ctx=std::make_shared<FakeExecution>();ctx->fail=true;board->set<std::shared_ptr<Execution>>(kExecutionBlackboard,ctx);auto tree=factory.createTreeFromText("<root BTCPP_format='4'><BehaviorTree ID='Coverage'><Sequence><ComputeCoveragePath/><FollowCoveragePath/></Sequence></BehaviorTree></root>",board);Require(tree.tickOnce()==BT::NodeStatus::FAILURE&&ctx->follow==0,"Motion ran after failed plan");});
  add("bt_halt_cancels",[]{BT::BehaviorTreeFactory factory;RegisterCoverageNodes(factory);auto board=BT::Blackboard::create();auto ctx=std::make_shared<FakeExecution>();board->set<std::shared_ptr<Execution>>(kExecutionBlackboard,ctx);auto tree=factory.createTreeFromText("<root BTCPP_format='4'><BehaviorTree ID='Coverage'><FollowCoveragePath/></BehaviorTree></root>",board);Require(tree.tickOnce()==BT::NodeStatus::RUNNING,"Expected running");tree.haltTree();Require(ctx->cancel==1,"Halt did not cancel");});
  size_t passed=0;size_t selected=0;for(const auto& [name,test]:tests){const char* filter=std::getenv("COVERAGE_TEST_FILTER");if(filter&&name.find(filter)==std::string::npos)continue;++selected;try{test();++passed;std::cout<<"PASS "<<name<<'\n';}catch(const std::exception& e){std::cerr<<"FAIL "<<name<<": "<<e.what()<<'\n';}}
  std::cout<<passed<<'/'<<selected<<" coverage checks passed\n";
  if(argc>1){auto out=server.Compute(Rectangle(true));std::ofstream(argv[1])<<out.DebugString();}
  return passed==selected?0:1;
}
