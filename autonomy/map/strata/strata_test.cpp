/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fstream>

#include <gtest/gtest.h>

#include "autonomy/commsgs/strata_msgs.hpp"
#include "autonomy/map/strata/bridge/pathfinder_utils.hpp"
#include "autonomy/map/strata/bridge/scene_converter.hpp"
#include "autonomy/map/strata/constants.hpp"
#include "autonomy/map/strata/drawing/drawing_controller.hpp"
#include "autonomy/map/strata/map_features/canvas_labels.hpp"
#include "autonomy/map/strata/markers/marker_controller.hpp"
#include "autonomy/map/strata/navigation/graph_path_build.hpp"
#include "autonomy/map/strata/navigation/graph_pathfinder.hpp"
#include "autonomy/map/strata/navigation/path_build.hpp"
#include "autonomy/map/strata/navigation/pathfinder.hpp"
#include "autonomy/map/strata/navigation/geo_utils.hpp"
#include "autonomy/map/strata/navigation/poi_util.hpp"
#include "autonomy/map/strata/overlay/label_bubble.hpp"
#include "autonomy/map/strata/robot/core/robot_phase.hpp"
#include "autonomy/map/strata/robot/core/robot_profile.hpp"
#include "autonomy/map/strata/robot/infra/display_manager.hpp"
#include "autonomy/map/strata/robot/robot_engine.hpp"
#include "autonomy/map/strata/robot/tasks/composites.hpp"
#include "autonomy/map/strata/robot/tasks/task.hpp"
#include "autonomy/map/strata/robot/tasks/wait_condition.hpp"
#include "autonomy/map/strata/robot/visual/fov_geometry.hpp"
#include "autonomy/map/strata/robot/visual/robot3d_presets.hpp"
#include "autonomy/map/strata/urdf/urdf_loader.hpp"
#include "autonomy/map/strata/urdf/urdf_plugin.hpp"
#include "autonomy/map/strata/render/scene_state.hpp"
#include "autonomy/map/strata/shapes/edit_controller.hpp"
#include "autonomy/map/strata/strata_engine.hpp"
#include "autonomy/map/strata/utils/map_utils.hpp"
#include "autonomy/map/strata/utils/point_util.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace {

TEST(StrataMapUtilsTest, CartesianGpsRoundTrip) {
    utils::CartesianToGpsParams toGps;
    toGps.x = 100.;
    toGps.y = 200.;
    const auto gps = utils::CartesianToGps(toGps);

    utils::GpsToCartesianParams toCart;
    toCart.latitude = gps.latitude;
    toCart.longitude = gps.longitude;
    const auto cart = utils::GpsToCartesian(toCart);

    EXPECT_NEAR(cart.x, 100., 0.01);
    EXPECT_NEAR(cart.y, 200., 0.01);
}

TEST(StrataPointUtilTest, PointInPolygon) {
    std::vector<LngLat> polygon = {{0, 0}, {1, 0}, {1, 1}, {0, 1}, {0, 0}};
    EXPECT_TRUE(utils::PointInPolygon(0.5, 0.5, polygon));
    EXPECT_FALSE(utils::PointInPolygon(2., 2., polygon));
}

TEST(StrataPathfinderTest, FindPathWithoutObstacles) {
    navigation::PathfinderOptions options;
    options.widthMeters = 10.;
    options.heightMeters = 10.;
    navigation::Pathfinder pathfinder(options);
    pathfinder.SetObstacles({});

    const auto path = pathfinder.FindPath({0.1, 0.1}, {0.9, 0.9});
    ASSERT_TRUE(path.has_value());
    EXPECT_GE(path->size(), 1u);
}

TEST(StrataGraphPathfinderTest, FindPathOnGraph) {
    RoadGraph graph;
    graph.nodes = {{"a", "waypoint", {0., 0., 0.}}, {"b", "waypoint", {1., 0., 0.}}};
    graph.edges = {{"e1", "a", "b", 1.0}};
    navigation::GraphPathfinder pathfinder(graph);
    const auto path = pathfinder.FindPath("a", "b");
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(path->size(), 2u);
}

TEST(StrataEngineTest, InitAndCreateMap) {
    StrataEngine engine;
    engine.Init({});
    EXPECT_TRUE(engine.IsLoaded());

    MapViewOptions options;
    options.container = "map";
    options.center.x = 116.4074;
    options.center.y = 39.9042;
    options.zoom = 16.;
    engine.CreateMap(options);
    EXPECT_DOUBLE_EQ(engine.GetMapOptions().zoom, 16.);
}

TEST(StrataRobotPhaseTest, ValidTransition) {
    EXPECT_TRUE(robot::core::CanTransition(RobotPhase::kIdle, RobotPhase::kMoving));
    EXPECT_FALSE(robot::core::CanTransition(RobotPhase::kIdle, RobotPhase::kArrived));
}

TEST(StrataPathBuildTest, BuildRouteWithStraightSegments) {
    navigation::PathBuildService service;
    navigation::PathfinderOptions options;
    options.widthMeters = 10.;
    options.heightMeters = 10.;
    service.InitPathfinder(10., 10., options);

    const std::vector<std::pair<double, double>> points = {{0.1, 0.1}, {0.9, 0.9}, {0.1, 0.9}};
    const auto route = service.BuildPathfindingRoute(points, {});
    EXPECT_GE(route.coords.size(), 2u);
}

TEST(StrataPoiUtilTest, ForbiddenZoneDetection) {
    ObstaclePolygon zone;
    zone.id = "z1";
    zone.polygon = {{0.2, 0.2}, {0.8, 0.2}, {0.8, 0.8}, {0.2, 0.8}, {0.2, 0.2}};
    EXPECT_TRUE(navigation::IsPointInForbiddenZone(0.5, 0.5, {zone}));
    EXPECT_FALSE(navigation::IsPointInForbiddenZone(0.05, 0.05, {zone}));
}

TEST(StrataDrawingTest, RectangleDrawingFlow) {
    render::SceneState::SharedPtr scene = render::SceneState::make_shared();
    drawing::DrawingController controller(scene);
    bool completed = false;
    controller.EnableRectangleDrawing([&](const std::string& type, const std::vector<LngLat>& pts,
                                          double area) {
        completed = true;
        EXPECT_EQ(type, "rectangle");
        EXPECT_EQ(pts.size(), 2u);
        EXPECT_GT(area, 0.);
    });
    LngLat p1;
    p1.x = 0.;
    p1.y = 0.;
    LngLat p2;
    p2.x = 1.;
    p2.y = 1.;
    controller.OnPointerDown(p1);
    controller.OnPointerMove(p2);
    controller.OnPointerUp(p2);
    EXPECT_TRUE(completed);
}

TEST(StrataGeoUtilsTest, FracToCart) {
    navigation::GeoUtilsConfig config;
    config.startX = 0.;
    config.startY = 0.;
    config.width = 100.;
    config.height = 50.;
    navigation::GeoUtils utils(config);
    const auto cart = utils.FracToCart(0.5, 0.5);
    EXPECT_DOUBLE_EQ(cart.x, 50.);
    EXPECT_DOUBLE_EQ(cart.y, 25.);
}

TEST(StrataRobotProfileTest, CreateGuideProfile) {
    const auto profile = robot::core::CreateRobotProfile(robot::core::RobotProfileTypes::kGuideIndoor);
    EXPECT_EQ(profile.type, "guide");
    EXPECT_TRUE(robot::core::HasCapability(profile, "announce"));
}

TEST(StrataWaitConditionTest, DurationConditionCompletes) {
    auto condition = robot::tasks::WaitCondition::Duration(100.);
    robot::tasks::WaitContext context;
    context.elapsed_ms = 0.;
    EXPECT_FALSE(condition->Evaluate(context).met);
    context.elapsed_ms = 150.;
    EXPECT_TRUE(condition->Evaluate(context).met);
}

TEST(StrataWaitConditionTest, AllAndRaceConditions) {
    auto duration = robot::tasks::WaitCondition::Duration(100.);
    auto signal = robot::tasks::WaitCondition::Signal("go");
    robot::tasks::WaitContext context;
    context.elapsed_ms = 0.;
    context.signals["go"] = "1";

    auto race = robot::tasks::WaitCondition::Race({duration, signal});
    EXPECT_TRUE(race->Evaluate(context).met);

    context.signals.clear();
    auto all = robot::tasks::WaitCondition::All({duration, signal});
    EXPECT_FALSE(all->Evaluate(context).met);
    context.elapsed_ms = 150.;
    context.signals["go"] = "1";
    EXPECT_TRUE(all->Evaluate(context).met);
}

TEST(StrataFovGeometryTest, BuildFovBands) {
    LngLat position;
    position.x = 0.;
    position.y = 0.;
    RobotFovOptions options;
    const auto bands = robot::visual::BuildFovBands(position, 0., options);
    EXPECT_GE(bands.size(), 2u);
    EXPECT_FALSE(bands.front().polygon.empty());
}

TEST(StrataCanvasLabelsTest, BuildFeaturesFromPolygon) {
    map_features::CanvasLabelInput input;
    input.id = "room-1";
    input.label = "Room A";
    input.polygon = {{0., 0.}, {2., 0.}, {2., 2.}, {0., 2.}, {0., 0.}};
    const auto features = map_features::CanvasLabelsController::BuildFeatures({input});
    ASSERT_EQ(features.size(), 1u);
    EXPECT_EQ(features.front().label, "Room A");
    EXPECT_DOUBLE_EQ(features.front().position.x, 1.);
    EXPECT_DOUBLE_EQ(features.front().position.y, 1.);
}

TEST(StrataPolygonEditTest, EnterAndCommitEdit) {
    render::SceneState::SharedPtr scene = render::SceneState::make_shared();
    auto collection = shapes::PolygonCollection::make_shared(
        scene, &render::SceneState::polygons, LayerIds::kPolygon);
    PolygonFeature polygon;
    polygon.id = "poly-1";
    polygon.points = {{0., 0.}, {1., 0.}, {1., 1.}, {0., 1.}};
    collection->Add(polygon);

    shapes::PolygonEditController editor(scene, collection);
    ASSERT_TRUE(editor.EnterEditMode("poly-1"));
    EXPECT_TRUE(editor.IsEditMode());
    ASSERT_TRUE(editor.UpdateEditPoint(0, LngLat{0.5, 0.5}));
    EXPECT_TRUE(editor.CommitEdit());
    const auto updated = collection->GetById("poly-1");
    ASSERT_TRUE(updated.has_value());
    EXPECT_DOUBLE_EQ(updated->points.front().x, 0.5);
}

TEST(StrataPoiMarkerTest, SelectionApi) {
    render::SceneState::SharedPtr scene = render::SceneState::make_shared();
    markers::PoiMarkerController controller(scene);
    PoiMarker marker_a;
    marker_a.id = "a";
    PoiMarker marker_b;
    marker_b.id = "b";
    controller.AddBatch({marker_a, marker_b});
    controller.SelectAll();
    EXPECT_EQ(controller.GetSelectedMarkers().size(), 2u);
    controller.ToggleSelection("a");
    EXPECT_EQ(controller.GetSelectedMarkers().size(), 1u);
}

TEST(StrataLoopTaskTest, RepeatsChildTask) {
    class CounterTask : public robot::tasks::Task {
     public:
        TaskStatus Tick(double deltaTimeMs) override {
            (void)deltaTimeMs;
            if (status_ == TaskStatus::kSuccess) {
                return status_;
            }
            status_ = TaskStatus::kSuccess;
            ++count;
            return status_;
        }
        void Reset() override {
            Task::Reset();
        }
        int count{0};
    };
    auto counter = std::make_shared<CounterTask>();
    robot::tasks::LoopTask loop(counter, 3);
    loop.Tick(1.);
    loop.Tick(1.);
    loop.Tick(1.);
    EXPECT_EQ(counter->count, 3);
    EXPECT_EQ(loop.status(), TaskStatus::kSuccess);
}

TEST(StrataIotBubbleTest, DismissSingleBubble) {
    render::SceneState::SharedPtr scene = render::SceneState::make_shared();
    overlay::IotBubbleController controller(scene, 0, 10);
    LngLat position;
    const std::string first = controller.Emit(IotEventType::kCustom, position, "one");
    controller.Emit(IotEventType::kCustom, position, "two");
    controller.Dismiss(first);
    EXPECT_EQ(scene->iotBubbles.size(), 1u);
    EXPECT_NE(scene->iotBubbles.front().id, first);
}

TEST(StrataIotBubbleTest, PruneExpiredRemovesStaleBubbles) {
    render::SceneState::SharedPtr scene = render::SceneState::make_shared();
    overlay::IotBubbleController controller(scene, 0, 10);
    render::IotBubbleState stale;
    stale.id = "stale";
    stale.visible = true;
    stale.expireAtMs = 1;
    scene->iotBubbles.push_back(stale);
    controller.PruneExpired();
    EXPECT_TRUE(scene->iotBubbles.empty());
}

TEST(StrataSceneConverterTest, ConvertsPoiAndPath) {
    render::ExportedScene exported;
    PoiMarker poi;
    poi.id = "poi-1";
    poi.name = "Gate";
    poi.lngLat.x = 1.;
    poi.lngLat.y = 2.;
    exported.poiMarkers.push_back(poi);

    PolylineFeature polyline;
    polyline.id = "path-1";
    LngLat p1;
    p1.x = 0.;
    p1.y = 0.;
    LngLat p2;
    p2.x = 1.;
    p2.y = 1.;
    polyline.points = {p1, p2};
    exported.polylines.push_back(polyline);

    const auto messages = bridge::ConvertExportedScene(exported, "map", 1000);
    EXPECT_EQ(messages.poi_markers.markers.size(), 1u);
    EXPECT_EQ(messages.poi_markers.markers[0].name, "Gate");
    EXPECT_GE(messages.path.poses.size(), 2u);
    EXPECT_GE(messages.markers.markers_size(), 1);
}

TEST(StrataCommsgsTest, StrataMsgsRoundTrip) {
    commsgs::strata_msgs::RoadGraph graph;
    graph.header.frame_id = "map";
    commsgs::strata_msgs::GraphNode node_a;
    node_a.id = "a";
    node_a.type = "waypoint";
    node_a.coordinates.x = 1.0;
    graph.nodes.push_back(node_a);
    commsgs::strata_msgs::GraphEdge edge;
    edge.id = "e1";
    edge.from = "a";
    edge.to = "b";
    graph.edges.push_back(edge);

    const auto proto = commsgs::strata_msgs::ToProto(graph);
    const auto round_trip = commsgs::strata_msgs::FromProto(proto);
    EXPECT_EQ(round_trip.nodes.size(), 1u);
    EXPECT_EQ(round_trip.nodes[0].id, "a");
    EXPECT_EQ(round_trip.edges.size(), 1u);
    EXPECT_EQ(round_trip.edges[0].from, "a");

    std::string serialized;
    ASSERT_TRUE(graph.SerializeToString(&serialized));
    commsgs::strata_msgs::RoadGraph parsed;
    ASSERT_TRUE(parsed.ParseFromString(serialized));
    EXPECT_EQ(parsed.nodes.size(), 1u);
}

TEST(StrataEngineTest, SetRoadGraphExports) {
    StrataEngine engine;
    engine.Init({});
    RoadGraph graph;
    graph.nodes = {{"a", "waypoint", {0., 0., 0.}}, {"b", "waypoint", {2., 0., 0.}}};
    graph.edges = {{"e1", "a", "b", 2.0}};
    engine.SetRoadGraph(graph);

    const auto exported = engine.ExportScene();
    ASSERT_TRUE(exported.roadGraph.has_value());
    EXPECT_EQ(exported.roadGraph->nodes.size(), 2u);
    EXPECT_EQ(exported.roadGraph->edges.size(), 1u);

    const auto messages = bridge::ConvertExportedScene(exported, "map", 2000);
    EXPECT_EQ(messages.road_graph.nodes.size(), 2u);
    EXPECT_EQ(messages.road_graph.edges.size(), 1u);
    EXPECT_GE(messages.markers.markers_size(), 2);
}

TEST(StrataFloorManagerTest, ConfigureAndSwitchFloor) {
    StrataEngine engine;
    engine.Init({});
    FloorConfig floor_a;
    floor_a.id = "F1";
    floor_a.name = "1F";
    floor_a.slamOptions.xGridCount = 10;
    floor_a.slamOptions.yGridCount = 10;
    FloorConfig floor_b = floor_a;
    floor_b.id = "F2";
    floor_b.name = "2F";

    auto manager = engine.CreateFloorManager({floor_a, floor_b}, "F1");
    EXPECT_EQ(manager->GetActiveFloor(), "F1");
    EXPECT_EQ(manager->GetFloors().size(), 2u);

    ASSERT_TRUE(manager->SwitchTo("F2"));
    EXPECT_EQ(manager->GetActiveFloor(), "F2");

    const auto exported = engine.ExportScene();
    EXPECT_EQ(exported.floors.size(), 2u);
    EXPECT_EQ(exported.activeFloorId, "F2");
}

TEST(StrataGraphPathBuildTest, RemoveEdgesThroughBuildings) {
    navigation::GeoUtilsConfig config;
    config.width = 100.;
    config.height = 100.;
    navigation::GeoUtils geoUtils(config);

    const LngLat start = geoUtils.FracToGps(0.2, 0.2);
    const LngLat end = geoUtils.FracToGps(0.8, 0.8);
    RoadGraph graph;
    graph.nodes = {{"a", "waypoint", start}, {"b", "waypoint", end}};
    graph.edges = {{"e1", "a", "b", 1.0}};

    FracPolygon building = {{0.4, 0.4}, {0.6, 0.4}, {0.6, 0.6}, {0.4, 0.6}, {0.4, 0.4}};
    const auto filtered =
        navigation::RemoveEdgesThroughBuildings(graph, geoUtils, {building}, 16);
    EXPECT_TRUE(filtered.edges.empty());
}

TEST(StrataGraphPathBuildTest, ValidatePointCollision) {
    FracPolygon building = {{0., 0.}, {1., 0.}, {1., 1.}, {0., 1.}, {0., 0.}};
    const auto corrected = navigation::ValidatePointCollision(0.5, 0.5, {building});
    ASSERT_TRUE(corrected.has_value());
    EXPECT_FALSE(utils::PointInPolygon(corrected->first, corrected->second,
                                       {LngLat{0., 0.}, LngLat{1., 0.}, LngLat{1., 1.},
                                        LngLat{0., 1.}, LngLat{0., 0.}}));
}

TEST(StrataMoveTaskTest, RotatesThenMovesToTarget) {
    robot::RobotController controller("r1");
    controller.SetPosition({0., 0., 0.});
    controller.SetHeading(0.);
    LngLat target;
    target.x = 0.;
    target.y = 5.;
    controller.MoveTo(target);

    for (int i = 0; i < 200; ++i) {
        controller.Tick(50.);
        if (controller.state().phase == RobotPhase::kArrived) {
            break;
        }
    }
    EXPECT_EQ(controller.state().phase, RobotPhase::kArrived);
    EXPECT_NEAR(controller.state().position.y, 5., 0.6);
    EXPECT_LT(controller.state().battery, 100.f);
}

TEST(StrataSensorTaskTest, RunsSensorAction) {
    robot::RobotController controller("r1");
    bool called = false;
    auto task = robot::tasks::SensorTask::make_shared(
        "lidar", &controller.mutableState(), [&called](robot::RobotState& /*state*/) {
            called = true;
            return true;
        });
    controller.Execute(task);
    controller.Tick(1.);
    EXPECT_TRUE(called);
    EXPECT_EQ(task->status(), TaskStatus::kSuccess);
}

TEST(StrataMarkerClickTest, PoiAndRobotHandleClick) {
    render::SceneState::SharedPtr scene = render::SceneState::make_shared();
    markers::PoiMarkerController poi(scene);
    markers::RobotMarkerController robot(scene);

    PoiMarker poi_marker;
    poi_marker.id = "poi-1";
    poi.AddMarker(poi_marker);
    RobotMarker robot_marker;
    robot_marker.id = "robot-1";
    robot.AddRobot(robot_marker);

    std::string clicked;
    poi.SetOnClick([&clicked](const std::string& id) { clicked = id; });
    EXPECT_TRUE(poi.HandleClick("poi-1"));
    EXPECT_EQ(clicked, "poi-1");

    clicked.clear();
    robot.SetOnClick([&clicked](const std::string& id) { clicked = id; });
    EXPECT_TRUE(robot.HandleClick("robot-1"));
    EXPECT_EQ(clicked, "robot-1");
}

TEST(StrataDisplayManagerTest, UpdateRobotSyncs2DRenderer) {
    render::SceneState::SharedPtr scene = render::SceneState::make_shared();
    auto renderer = robot::visual::StatusRobotMarkerController::make_shared(scene);
    robot::infra::DisplayManagerConfig config;
    config.renderer2d = renderer;
    robot::infra::DisplayManager display(config);

    RobotMarker robot;
    robot.id = "r1";
    robot.lngLat.x = 1.;
    robot.lngLat.y = 2.;
    display.AddRobot(robot);

    RobotMarkerPatch patch;
    patch.lngLat = LngLat{};
    patch.lngLat->x = 5.;
    patch.lngLat->y = 6.;
    patch.battery = 80.f;
    EXPECT_TRUE(display.UpdateRobot("r1", patch));

    const auto robots = renderer->GetRobots();
    ASSERT_EQ(robots.size(), 1u);
    EXPECT_DOUBLE_EQ(robots.front().lngLat.x, 5.);
    EXPECT_FLOAT_EQ(robots.front().battery, 80.f);
}

TEST(StrataRobotEngineTest, TickSyncsDisplayManager) {
    StrataEngine engine;
    engine.Init({});
    auto status_renderer = engine.CreateStatusRobotMarkers();
    auto robot_engine = engine.CreateRobotEngine();
    robot_engine->GetDisplayManager()->SetRenderer2d(status_renderer);

    const auto profile = engine.CreateRobotProfile(robot::core::RobotProfileTypes::kGuideIndoor);
    robot::RobotInitialState initial;
    initial.position.x = 0.;
    initial.position.y = 0.;
    auto controller = robot_engine->AddRobot("patrol-1", profile, initial);

    RobotMarker marker;
    marker.id = "patrol-1";
    marker.lngLat = initial.position;
    robot_engine->GetDisplayManager()->AddRobot(marker);

    LngLat target;
    target.x = 0.;
    target.y = 10.;
    controller->MoveTo(target);
    robot_engine->Start();
    for (int i = 0; i < 100; ++i) {
        robot_engine->Tick(50.);
        if (controller->state().phase == RobotPhase::kArrived) {
            break;
        }
    }

    const auto exported = engine.ExportScene();
    ASSERT_EQ(exported.robotMarkers.size(), 1u);
    EXPECT_GT(exported.robotMarkers.front().lngLat.y, 1.);
    EXPECT_LT(exported.robotMarkers.front().battery, 100.f);
}

TEST(StrataRobotEngineTest, FovSyncsOnTick) {
    StrataEngine engine;
    engine.Init({});
    auto robot_engine = engine.CreateRobotEngine();
    robot_engine->SetScene(engine.GetSceneState());

    const auto profile = engine.CreateRobotProfile(robot::core::RobotProfileTypes::kGuideIndoor);
    robot::RobotInitialState initial;
    initial.position.x = 1.;
    initial.position.y = 2.;
    initial.headingDeg = 45.;
    robot_engine->AddRobot("patrol-fov", profile, initial);
    robot_engine->CreateFovForRobot("patrol-fov");

    robot_engine->Start();
    robot_engine->Tick(16.);

    const auto exported = engine.ExportScene();
    ASSERT_EQ(exported.robotFovs.size(), 1u);
    EXPECT_EQ(exported.robotFovs.front().id, "patrol-fov");
    EXPECT_FALSE(exported.robotFovs.front().bands.empty());
    EXPECT_DOUBLE_EQ(exported.robotFovs.front().position.x, 1.);
}

TEST(StrataSceneConverterTest, ExportsRobotFovBands) {
    render::ExportedScene exported;
    RobotFovState fov;
    fov.id = "r1";
    fov.position.x = 0.;
    fov.position.y = 0.;
    fov.visible = true;
    fov.bands = robot::visual::BuildFovBands(fov.position, 0., fov.options);
    exported.robotFovs.push_back(fov);

    const auto messages = bridge::ConvertExportedScene(exported, "map", 3000);
    EXPECT_GE(messages.markers.markers_size(), 1);
}

TEST(StrataSceneConverterTest, ExportsSemanticZones) {
    render::ExportedScene exported;
    render::SemanticZoneFeature zone;
    zone.id = "zone-1";
    zone.type = ZoneType::kForbidden;
    zone.polygon = {{0., 0.}, {2., 0.}, {2., 2.}, {0., 2.}, {0., 0.}};
    const auto& preset = ZoneStylePresets().at(zone.type);
    zone.style.fillColor = preset.fillColor;
    zone.style.fillOpacity = preset.fillOpacity;
    zone.style.outlineColor = preset.outlineColor;
    zone.style.outlineWidth = preset.outlineWidth;
    exported.semanticZones.push_back(zone);

    const auto messages = bridge::ConvertExportedScene(exported, "map", 4000);
    ASSERT_EQ(messages.semantic_zones.zones.size(), 1u);
    EXPECT_EQ(messages.semantic_zones.zones.front().zone_type, "forbidden");
    EXPECT_EQ(messages.semantic_zones.zones.front().label, "禁行区");
    EXPECT_GE(messages.markers.markers_size(), 1);
}

TEST(StrataPhaseToStatusTest, MapsMovingToRunning) {
    EXPECT_EQ(robot::core::PhaseToRobotStatus(RobotPhase::kMoving), RobotStatus::kRunning);
    EXPECT_EQ(robot::core::PhaseToRobotStatus(RobotPhase::kCharging), RobotStatus::kCharging);
    EXPECT_EQ(robot::core::PhaseToRobotStatus(RobotPhase::kIdle), RobotStatus::kIdle);
}

TEST(StrataRobotEngineTest, MoveRobotToUsesPathfinder) {
    navigation::PathfinderOptions options;
    options.widthMeters = 10.;
    options.heightMeters = 10.;
    auto pathfinder = navigation::Pathfinder::make_shared(options);

    ObstaclePolygon obstacle;
    obstacle.id = "block";
    obstacle.polygon = {{0.45, 0.}, {0.55, 0.}, {0.55, 1.}, {0.45, 1.}, {0.45, 0.}};
    pathfinder->SetObstacles({obstacle});

    robot::RobotEngine engine;
    engine.SetPathfinder(pathfinder);
    const auto profile = robot::core::CreateRobotProfile(robot::core::RobotProfileTypes::kGuideIndoor);
    robot::RobotInitialState initial;
    initial.position = {1., 1., 0.};
    auto controller = engine.AddRobot("path-robot", profile, initial);

    LngLat target;
    target.x = 9.;
    target.y = 9.;
    ASSERT_TRUE(engine.MoveRobotTo("path-robot", target));

    engine.Start();
    for (int i = 0; i < 400; ++i) {
        engine.Tick(50.);
        if (controller->state().phase == RobotPhase::kArrived) {
            break;
        }
    }
    EXPECT_EQ(controller->state().phase, RobotPhase::kArrived);
    EXPECT_NEAR(controller->state().position.x, 9., 1.0);
    EXPECT_NEAR(controller->state().position.y, 9., 1.0);
}

TEST(StrataRobotEngineTest, RegistersConditionAndActionTypes) {
    robot::RobotEngine engine;
    bool action_called = false;
    engine.RegisterConditionType("duration", []() {
        return robot::tasks::WaitCondition::Duration(100.);
    });
    engine.RegisterActionType("noop", [&action_called](robot::RobotState& /*state*/) {
        action_called = true;
        return true;
    });

    ASSERT_NE(engine.CreateCondition("duration"), nullptr);
    EXPECT_EQ(engine.CreateCondition("missing"), nullptr);
    ASSERT_TRUE(static_cast<bool>(engine.GetActionHandler("noop")));

    const auto profile = robot::core::CreateRobotProfile(robot::core::RobotProfileTypes::kGuideIndoor);
    engine.AddRobot("plugin-robot", profile);
    engine.PerformTypedAction("plugin-robot", "noop");
    auto robot = engine.GetRobot("plugin-robot");
    ASSERT_NE(robot, nullptr);
    robot->Tick(1.);
    EXPECT_TRUE(action_called);
}

TEST(StrataFloorManagerTest, BindLayers3DTogglesVisibility) {
    StrataEngine engine;
    engine.Init({});
    render::FloorState floor_a;
    floor_a.id = "F1";
    render::FloorState floor_b;
    floor_b.id = "F2";
    FloorConfig config_a;
    config_a.id = "F1";
    FloorConfig config_b;
    config_b.id = "F2";
    auto manager = engine.CreateFloorManager({config_a, config_b}, "F1");

    RobotFovState fov;
    fov.id = "fov-1";
    fov.visible = true;
    engine.GetSceneState()->robotFovs.push_back(fov);

    manager->BindLayers3D("F1", {LayerIds::kRobotFov});
    manager->SwitchTo("F2");
    EXPECT_FALSE(engine.GetSceneState()->robotFovs.front().visible);
    manager->SwitchTo("F1");
    EXPECT_TRUE(engine.GetSceneState()->robotFovs.front().visible);
}

TEST(StrataPathfinderUtilsTest, BuildObstaclesFromForbiddenZones) {
    render::SemanticZoneFeature zone;
    zone.id = "z1";
    zone.type = ZoneType::kForbidden;
    zone.polygon = {{2., 0., 0.}, {8., 0., 0.}, {8., 10., 0.}, {2., 10., 0.}, {2., 0., 0.}};
    const auto obstacles =
        bridge::BuildPathfinderObstacles({zone}, 10., 10.);
    ASSERT_EQ(obstacles.size(), 1u);
    EXPECT_NEAR(obstacles.front().polygon.front().x, 0.2, 1e-6);
    EXPECT_TRUE(navigation::IsPointInForbiddenZone(0.5, 0.5, obstacles));
}

TEST(StrataPathfinderUtilsTest, MapSizeFromSlam) {
    SlamMapOptions slam;
    slam.xGridCount = 200;
    slam.yGridCount = 100;
    slam.resolution = 0.05;
    const auto size = bridge::MapSizeFromSlam(slam);
    EXPECT_DOUBLE_EQ(size.first, 10.);
    EXPECT_DOUBLE_EQ(size.second, 5.);
}

TEST(StrataChargeTaskTest, IncreasesBatteryPerFrame) {
    robot::RobotController controller("charge-bot");
    controller.mutableState().battery = 20.f;
    controller.Charge(100., 1.0);

    for (int i = 0; i < 100; ++i) {
        controller.Tick(10.);
        if (controller.state().battery >= 99.9f) {
            break;
        }
        EXPECT_EQ(controller.state().phase, RobotPhase::kCharging);
    }

    EXPECT_GE(controller.state().battery, 99.f);
    EXPECT_FALSE(controller.state().currentTask);
}

TEST(StrataWaitForConditionTest, RobotControllerUsesConditionTask) {
    robot::RobotEngine engine;
    const auto profile = robot::core::CreateRobotProfile(robot::core::RobotProfileTypes::kGuideIndoor);
    auto robot = engine.AddRobot("wait-bot", profile);
    robot->WaitFor(robot::tasks::WaitCondition::Duration(100.));
    EXPECT_EQ(robot->state().phase, RobotPhase::kWaiting);

    engine.Start();
    engine.Tick(110.);
    EXPECT_EQ(robot->GetTaskStatus(), TaskStatus::kIdle);
    EXPECT_EQ(robot->state().phase, RobotPhase::kIdle);
}

TEST(StrataWaitForConditionTest, EnvironmentConditionsViaEngine) {
    robot::RobotEngine engine;
    const auto profile = robot::core::CreateRobotProfile(robot::core::RobotProfileTypes::kGuideIndoor);
    auto robot = engine.AddRobot("env-bot", profile);

    engine.SetSensor("temperature", 25.);
    engine.SetZoneOccupancy("zone-a", "free");
    robot::ElevatorState elevator;
    elevator.floor = "L2";
    elevator.doors_open = true;
    engine.SetElevator("elev-1", elevator);
    engine.SetTrafficLight("tl-1", "green");

    auto all = robot::tasks::WaitCondition::All(
        {robot::tasks::WaitCondition::Sensor("temperature", 25.),
         robot::tasks::WaitCondition::Occupancy("zone-a", "free"),
         robot::tasks::WaitCondition::Elevator("L2", "elev-1"),
         robot::tasks::WaitCondition::TrafficLight("green", "tl-1")});
    engine.WaitRobotFor("env-bot", std::move(all));
    engine.Start();
    engine.Tick(10.);
    EXPECT_EQ(robot->GetTaskStatus(), TaskStatus::kIdle);
}

TEST(StrataWaitForConditionTest, WaitTimeoutFailsTask) {
    robot::RobotEngine engine;
    const auto profile = robot::core::CreateRobotProfile(robot::core::RobotProfileTypes::kGuideIndoor);
    auto robot = engine.AddRobot("timeout-bot", profile);
    engine.RegisterResource(robot::infra::ResourceType::kDoor, "door-1");
    engine.WaitRobotFor("timeout-bot",
                        robot::tasks::WaitCondition::Resource(robot::infra::ResourceType::kDoor,
                                                              "door-1"),
                        50.);
    engine.Start();
    engine.Tick(60.);
    EXPECT_EQ(robot->GetTaskStatus(), TaskStatus::kFailure);
    EXPECT_EQ(robot->state().phase, RobotPhase::kError);
}

TEST(StrataDockTaskTest, RunsForConfiguredDuration) {
    robot::RobotController controller("dock-bot");
    controller.Dock("station-a", 100.);
    EXPECT_EQ(controller.state().phase, RobotPhase::kDocking);
    controller.Tick(50.);
    EXPECT_EQ(controller.GetTaskStatus(), TaskStatus::kRunning);
    EXPECT_EQ(controller.state().phase, RobotPhase::kDocking);
    controller.Tick(60.);
    EXPECT_EQ(controller.GetTaskStatus(), TaskStatus::kIdle);
}

TEST(StrataEventBusTest, OnceHandlerFiresOnlyOnce) {
    robot::infra::EventBus bus;
    int count = 0;
    bus.Once("door_open", [&](const std::string&, const std::string&) { ++count; });
    bus.Emit("door_open", "payload");
    bus.Emit("door_open", "payload");
    EXPECT_EQ(count, 1);
    EXPECT_EQ(bus.GetSignalPayload("door_open"), "payload");
}

TEST(StrataSceneConverterTest, ExportsBuildingExtrusionMarkers) {
    render::ExportedScene scene;
    render::BuildingFeature building;
    building.id = "b1";
    building.footprint = {{0., 0.}, {2., 0.}, {2., 1.5}, {0., 1.5}};
    building.height = 3.;
    building.heightScale = 1.;
    building.opacity = 0.8f;
    building.color = {0.4f, 0.5f, 0.7f, 0.8f};
    scene.buildings.push_back(building);

    const auto converted = bridge::ConvertExportedScene(scene, "map", 0);
    bool has_outline = false;
    bool has_extrusion = false;
    for (const auto& marker : converted.markers.markers()) {
        if (marker.ns() == "strata_building") {
            has_outline = true;
        }
        if (marker.ns() == "strata_building_extrusion") {
            has_extrusion = true;
            EXPECT_GT(marker.scale().z(), 0.);
        }
    }
    EXPECT_TRUE(has_outline);
    EXPECT_TRUE(has_extrusion);
}

TEST(StrataSceneConverterTest, ExportsCanvasLabelsAndIotBubbles) {
    render::ExportedScene scene;
    CanvasLabelFeature label;
    label.id = "l1";
    label.label = "Room A";
    label.position = {1., 2., 0.};
    scene.canvasLabels.push_back(label);

    render::IotBubbleState iot;
    iot.id = "iot-1";
    iot.type = IotEventType::kWarning;
    iot.message = "alert";
    iot.position = {3., 4., 0.};
    iot.visible = true;
    scene.iotBubbles.push_back(iot);

    LabelBubbleOptions bubble;
    bubble.lngLat = {5., 6., 0.};
    bubble.html = "<b>Hi</b>";
    scene.labelBubbles.push_back(bubble);

    Robot3DLayerState layer;
    layer.id = "r3d-1";
    layer.robotId = "bot-1";
    layer.model.modelUrl = "/models/test.glb";
    layer.position = {1., 1., 0.};
    scene.robot3DLayers.push_back(layer);

    const auto converted = bridge::ConvertExportedScene(scene, "map", 0);
    ASSERT_EQ(converted.canvas_labels.labels.size(), 1u);
    EXPECT_EQ(converted.canvas_labels.labels.front().label, "Room A");
    ASSERT_EQ(converted.iot_bubbles.bubbles.size(), 1u);
    EXPECT_EQ(converted.iot_bubbles.bubbles.front().message, "alert");
    ASSERT_EQ(converted.label_bubbles.bubbles.size(), 1u);
    EXPECT_EQ(converted.label_bubbles.bubbles.front().html, "<b>Hi</b>");
    ASSERT_EQ(converted.robot_3d_layers.layers.size(), 1u);
    EXPECT_EQ(converted.robot_3d_layers.layers.front().model_url, "/models/test.glb");
}

TEST(StrataRobot3DPresetsTest, MapsProfileModelFile) {
    const auto profile = robot::core::CreateRobotProfile(robot::core::RobotProfileTypes::kGuideIndoor);
    ResourcePaths paths;
    paths.assetsUrl = "/assets";
    const auto config = robot::visual::ModelConfigFromProfile(profile, paths);
    EXPECT_EQ(config.modelUrl, "/assets/models/guide-bot.glb");
    EXPECT_FALSE(config.animations.empty());
    EXPECT_EQ(config.defaultAnimation, "walk");
}

TEST(StrataUrdfPluginTest, LoadsLocalUrdf) {
    const std::string urdf_path = testing::TempDir() + "strata_test_robot.urdf";
    {
        std::ofstream out(urdf_path);
        out << R"(<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link"/>
  <link name="wheel_link"/>
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
  </joint>
</robot>)";
    }

    urdf::UrdfPlugin plugin;
    ASSERT_TRUE(plugin.UrdfLoad(urdf_path));
    EXPECT_TRUE(plugin.IsLoaded());
    EXPECT_EQ(plugin.Model().name, "test_robot");
    EXPECT_EQ(plugin.Model().links.size(), 2u);
    ASSERT_EQ(plugin.Model().joints.size(), 1u);
    EXPECT_EQ(plugin.Model().joints.front().name, "wheel_joint");
    plugin.SetJoint("wheel_joint", 0.5);
    plugin.Destroy();
    EXPECT_FALSE(plugin.IsLoaded());
}

}  // namespace
}  // namespace strata
}  // namespace map
}  // namespace autonomy
