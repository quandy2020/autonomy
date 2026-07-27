/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <gtest/gtest.h>

#include <functional>
#include <string>
#include <vector>

#include "autoviz/display/map_display.hpp"
#include "autoviz/display/marker_array_display.hpp"
#include "autoviz/display/path_display.hpp"
#include "autoviz/display/point_cloud2_display.hpp"
#include "autoviz/display/strata_building_display.hpp"
#include "autoviz/display/strata_canvas_label_display.hpp"
#include "autoviz/display/strata_fov_display.hpp"
#include "autoviz/display/strata_iot_bubble_display.hpp"
#include "autoviz/display/strata_label_bubble_display.hpp"
#include "autoviz/display/strata_poi_display.hpp"
#include "autoviz/display/strata_robot3d_display.hpp"
#include "autoviz/display/strata_robot_display.hpp"
#include "autoviz/display/strata_road_graph_display.hpp"
#include "autoviz/display/strata_semantic_zone_display.hpp"
#include "tests/bicmap_message_fixtures.hpp"
#include "tests/display_test_harness.hpp"

namespace autoviz {
namespace tests {
namespace {

using TestableMapDisplay = TestableDisplay<display::MapDisplay>;
using TestablePathDisplay = TestableDisplay<display::PathDisplay>;
using TestablePointCloud2Display = TestableDisplay<display::PointCloud2Display>;
using TestableMarkerArrayDisplay = TestableDisplay<display::MarkerArrayDisplay>;
using TestableStrataPoiDisplay = TestableDisplay<display::StrataPoiDisplay>;
using TestableStrataRobotDisplay = TestableDisplay<display::StrataRobotDisplay>;
using TestableStrataSemanticZoneDisplay =
    TestableDisplay<display::StrataSemanticZoneDisplay>;
using TestableStrataFovDisplay = TestableDisplay<display::StrataFovDisplay>;
using TestableStrataRoadGraphDisplay =
    TestableDisplay<display::StrataRoadGraphDisplay>;
using TestableStrataCanvasLabelDisplay =
    TestableDisplay<display::StrataCanvasLabelDisplay>;
using TestableStrataLabelBubbleDisplay =
    TestableDisplay<display::StrataLabelBubbleDisplay>;
using TestableStrataIotBubbleDisplay =
    TestableDisplay<display::StrataIotBubbleDisplay>;
using TestableStrataRobot3DDisplay =
    TestableDisplay<display::StrataRobot3DDisplay>;
using TestableStrataBuildingDisplay =
    TestableDisplay<display::StrataBuildingDisplay>;

struct BicMapExampleCase {
  std::string route;
  std::string name;
  std::function<void(DisplayTestFixture*)> run;
};

void RunIndoorSlam(DisplayTestFixture* fixture) {
  TestableMapDisplay display("/strata/map");
  fixture->feedAndDraw(display, bicmap::MakeOccupancyGrid(32, 24));
}

void RunIndoorBuildMap(DisplayTestFixture* fixture) {
  TestableMapDisplay display("/strata/map");
  auto grid = bicmap::MakeOccupancyGrid(48, 48);
  grid.set_data(100, -1);
  fixture->feedAndDraw(display, grid);
}

void RunIndoorRobotFollow(DisplayTestFixture* fixture) {
  TestableStrataRobotDisplay robot("/strata/robots");
  TestableStrataFovDisplay fov("/strata/markers");
  fixture->feedAndDraw(robot, bicmap::MakeRobotMarkers(1));
  fixture->feedAndDraw(fov, bicmap::MakeRobotFovMarker());
}

void RunBaseMapTools(DisplayTestFixture* fixture) {
  TestableMapDisplay display("/strata/map");
  fixture->feedAndDraw(display, bicmap::MakeOccupancyGrid(64, 64));
}

void RunBasePoiMarkers(DisplayTestFixture* fixture) {
  TestableStrataPoiDisplay display("/strata/poi");
  fixture->feedAndDraw(display, bicmap::MakePoiMarkers(4));
}

void RunExpandPoiAdvancedLabel(DisplayTestFixture* fixture) {
  TestableStrataPoiDisplay display("/strata/poi");
  fixture->feedAndDraw(display, bicmap::MakePoiMarkers(3, true));
}

void RunIndoorRelocate(DisplayTestFixture* fixture) {
  TestableStrataRobotDisplay display("/strata/robots");
  auto robots = bicmap::MakeRobotMarkers(1, "idle");
  robots.mutable_robots(0)->mutable_lng_lat()->set_x(3.5);
  robots.mutable_robots(0)->mutable_lng_lat()->set_y(2.0);
  fixture->feedAndDraw(display, robots);
}

void RunIndoorLocation(DisplayTestFixture* fixture) {
  TestableStrataRobotDisplay display("/strata/robots");
  auto robots = bicmap::MakeRobotMarkers(1);
  for (int step = 0; step < 3; ++step) {
    robots.mutable_robots(0)->mutable_lng_lat()->set_x(0.5 + 0.2 * step);
    fixture->feedAndDraw(display, robots);
  }
}

void RunIndoorSingleNavigation(DisplayTestFixture* fixture) {
  TestableStrataRobotDisplay robot("/strata/robots");
  TestablePathDisplay path("/strata/path");
  fixture->feedAndDraw(path, bicmap::MakePath(8));
  fixture->feedAndDraw(robot, bicmap::MakeRobotMarkers(1));
}

void RunIndoorLoad3D(DisplayTestFixture* fixture) {
  TestableMarkerArrayDisplay display("/strata/markers");
  fixture->feedAndDraw(display, bicmap::MakeCubeMarker(0));
}

void RunIndoorLoad3dControl(DisplayTestFixture* fixture) {
  TestableMarkerArrayDisplay display("/strata/markers");
  auto array = bicmap::MakeCubeMarker(0);
  array.mutable_markers(0)->mutable_pose()->mutable_orientation()->set_w(0.707);
  array.mutable_markers(0)->mutable_pose()->mutable_orientation()->set_z(0.707);
  fixture->feedAndDraw(display, array);
}

void RunIndoorPointCloud(DisplayTestFixture* fixture) {
  TestablePointCloud2Display display("/strata/pointcloud");
  fixture->feedAndDraw(display, bicmap::MakePointCloud2(200));
}

void RunIndoorSemanticMap(DisplayTestFixture* fixture) {
  TestableStrataSemanticZoneDisplay display("/strata/semantic_zones");
  fixture->feedAndDraw(display, bicmap::MakeSemanticZones("activity_area"));
}

void RunIndoorLoad3dMarker(DisplayTestFixture* fixture) {
  TestableMarkerArrayDisplay display("/strata/markers");
  fixture->feedAndDraw(display, bicmap::MakeSpaceMarkers());
}

void RunSceneRobotGuideTour(DisplayTestFixture* fixture) {
  TestableStrataRobotDisplay robot("/strata/robots");
  TestableStrataPoiDisplay poi("/strata/poi");
  TestablePathDisplay path("/strata/path");
  fixture->feedAndDraw(poi, bicmap::MakePoiMarkers(5));
  fixture->feedAndDraw(path, bicmap::MakePath(10));
  fixture->feedAndDraw(robot, bicmap::MakeRobotMarkers(1, "running"));
}

void RunSceneHotelDelivery(DisplayTestFixture* fixture) {
  TestableStrataRobotDisplay robot("/strata/robots");
  TestablePathDisplay path("/strata/path");
  fixture->feedAndDraw(path, bicmap::MakePath(6));
  fixture->feedAndDraw(robot, bicmap::MakeRobotMarkers(1, "running"));
}

void RunSceneMallRobotMonitor(DisplayTestFixture* fixture) {
  TestableStrataRobotDisplay robot("/strata/robots");
  TestableStrataPoiDisplay poi("/strata/poi");
  TestableStrataFovDisplay fov("/strata/markers");
  fixture->feedAndDraw(robot, bicmap::MakeRobotMarkers(2));
  fixture->feedAndDraw(poi, bicmap::MakePoiMarkers(3));
  fixture->feedAndDraw(fov, bicmap::MakeRobotFovMarker());
}

void RunSceneIndoorCleaning(DisplayTestFixture* fixture) {
  TestableStrataSemanticZoneDisplay zones("/strata/semantic_zones");
  TestableStrataRobotDisplay robot("/strata/robots");
  TestablePathDisplay path("/strata/path");
  fixture->feedAndDraw(zones, bicmap::MakeSemanticZones("waiting"));
  fixture->feedAndDraw(path, bicmap::MakePath(12));
  fixture->feedAndDraw(robot, bicmap::MakeRobotMarkers(1, "running"));
}

void RunSceneCommunityInspect(DisplayTestFixture* fixture) {
  TestableStrataRobotDisplay robot("/strata/robots");
  TestablePathDisplay path("/strata/path");
  TestableStrataPoiDisplay poi("/strata/poi");
  fixture->feedAndDraw(robot, bicmap::MakeRobotMarkers(1));
  fixture->feedAndDraw(path, bicmap::MakePath(7));
  fixture->feedAndDraw(poi, bicmap::MakePoiMarkers(2));
}

void RunBasePolylineDrawing(DisplayTestFixture* fixture) {
  TestableMarkerArrayDisplay display("/strata/markers");
  fixture->feedAndDraw(display, bicmap::MakeLineStripMarker(0, 6));
}

void RunBaseRectangleDraw(DisplayTestFixture* fixture) {
  TestableMarkerArrayDisplay display("/strata/markers");
  fixture->feedAndDraw(display, bicmap::MakeRectangleMarker(0));
}

void RunBasePolygonDraw(DisplayTestFixture* fixture) {
  TestableMarkerArrayDisplay display("/strata/markers");
  auto array = bicmap::MakeLineStripMarker(0, 5);
  auto* marker = array.mutable_markers(0);
  auto* close = marker->add_points();
  close->set_x(marker->points(0).x());
  close->set_y(marker->points(0).y());
  fixture->feedAndDraw(display, array);
}

void RunBaseCircleDraw(DisplayTestFixture* fixture) {
  TestableMarkerArrayDisplay display("/strata/markers");
  fixture->feedAndDraw(display, bicmap::MakeCircleMarker(0));
}

void RunBasePathReplay(DisplayTestFixture* fixture) {
  TestablePathDisplay display("/strata/path");
  fixture->feedAndDraw(display, bicmap::MakePath(20));
}

void RunBasePathPlanning(DisplayTestFixture* fixture) {
  TestableMapDisplay map("/strata/map");
  TestablePathDisplay path("/strata/path");
  fixture->feedAndDraw(map, bicmap::MakeOccupancyGrid(40, 30));
  fixture->feedAndDraw(path, bicmap::MakePath(9));
}

void RunExpandMapEditor(DisplayTestFixture* fixture) {
  TestableMapDisplay display("/strata/map");
  auto grid = bicmap::MakeOccupancyGrid(20, 20);
  grid.set_data(50, 100);
  grid.set_data(120, 0);
  fixture->feedAndDraw(display, grid);
}

void RunExpandGraphicDrawing(DisplayTestFixture* fixture) {
  TestableMarkerArrayDisplay display("/strata/markers");
  fixture->feedAndDraw(display, bicmap::MakeGraphicDrawingMarkers());
}

void RunBasePassableArea(DisplayTestFixture* fixture) {
  TestableStrataSemanticZoneDisplay display("/strata/semantic_zones");
  fixture->feedAndDraw(display, bicmap::MakeSemanticZones("activity_area"));
}

void RunBaseEditPassableArea(DisplayTestFixture* fixture) {
  TestableStrataSemanticZoneDisplay display("/strata/semantic_zones");
  auto zones = bicmap::MakeSemanticZones("speed_limit");
  auto* zone = zones.add_zones();
  *zone = zones.zones(0);
  zone->set_id("zone_1");
  zone->mutable_polygon(0)->set_x(3.0);
  fixture->feedAndDraw(display, zones);
}

void RunOutdoorBuildings(DisplayTestFixture* fixture) {
  TestableStrataBuildingDisplay display("/strata/markers");
  fixture->feedAndDraw(display, bicmap::MakeBuildingExtrusionMarkers());
}

void RunOutdoorPointCloud(DisplayTestFixture* fixture) {
  TestablePointCloud2Display display("/strata/pointcloud");
  fixture->feedAndDraw(display, bicmap::MakePointCloud2(800));
}

void RunSceneTransportHub(DisplayTestFixture* fixture) {
  TestableStrataRoadGraphDisplay graph("/strata/road_graph");
  TestableStrataPoiDisplay poi("/strata/poi");
  fixture->feedAndDraw(graph, bicmap::MakeRoadGraph());
  fixture->feedAndDraw(poi, bicmap::MakePoiMarkers(4, true));
}

void RunScenePublicClean(DisplayTestFixture* fixture) {
  TestableStrataSemanticZoneDisplay zones("/strata/semantic_zones");
  TestableStrataRobotDisplay robot("/strata/robots");
  fixture->feedAndDraw(zones, bicmap::MakeSemanticZones("forbidden"));
  fixture->feedAndDraw(robot, bicmap::MakeRobotMarkers(1, "running"));
}

void RunOutdoorMapTiles(DisplayTestFixture* fixture) {
  TestableMapDisplay display("/strata/map");
  fixture->feedAndDraw(display, bicmap::MakeOccupancyGrid(128, 128, 0.1f));
}

void RunOutdoorHdMap(DisplayTestFixture* fixture) {
  TestableStrataRoadGraphDisplay display("/strata/road_graph");
  fixture->feedAndDraw(display, bicmap::MakeRoadGraph());
}

void RunSceneSecurityPatrol(DisplayTestFixture* fixture) {
  TestableStrataRobotDisplay robot("/strata/robots");
  TestablePathDisplay path("/strata/path");
  TestableStrataSemanticZoneDisplay zones("/strata/semantic_zones");
  TestableStrataFovDisplay fov("/strata/markers");
  fixture->feedAndDraw(zones, bicmap::MakeSemanticZones("forbidden"));
  fixture->feedAndDraw(path, bicmap::MakePath(15));
  fixture->feedAndDraw(robot, bicmap::MakeRobotMarkers(1, "running"));
  fixture->feedAndDraw(fov, bicmap::MakeRobotFovMarker());
}

std::vector<BicMapExampleCase> AllBicMapExamples() {
  return {
      {"/indoor/slam", "Slam", RunIndoorSlam},
      {"/indoor/buildMap", "BuildMap", RunIndoorBuildMap},
      {"/indoor/robotFollow", "RobotFollow", RunIndoorRobotFollow},
      {"/base/mapTools", "MapTools", RunBaseMapTools},
      {"/base/POIMarkers", "POIMarkers", RunBasePoiMarkers},
      {"/expand/POIadvancedLabel", "POIadvancedLabel", RunExpandPoiAdvancedLabel},
      {"/indoor/relocate", "Relocate", RunIndoorRelocate},
      {"/indoor/location", "Location", RunIndoorLocation},
      {"/indoor/singleNavigation", "SingleNavigation", RunIndoorSingleNavigation},
      {"/indoor/load3D", "load3D", RunIndoorLoad3D},
      {"/indoor/load3dControl", "load3dControl", RunIndoorLoad3dControl},
      {"/indoor/pointCloud", "PointCloud", RunIndoorPointCloud},
      {"/indoor/semanticMap", "SemanticMap", RunIndoorSemanticMap},
      {"/indoor/load3dMarker", "Space", RunIndoorLoad3dMarker},
      {"/scene/robotGuideTour", "RobotGuideTour", RunSceneRobotGuideTour},
      {"/scene/hotelDelivery", "HotelDelivery", RunSceneHotelDelivery},
      {"/scene/mallRobotMonitor", "MallRobotMonitor", RunSceneMallRobotMonitor},
      {"/scene/indoorCleaning", "IndoorCleaning", RunSceneIndoorCleaning},
      {"/scene/communityInspect", "CommunityInspect", RunSceneCommunityInspect},
      {"/base/PolylineDrawing", "PolylineDrawing", RunBasePolylineDrawing},
      {"/base/RectangleDraw", "RectangleDraw", RunBaseRectangleDraw},
      {"/base/PolygonDraw", "PolygonDraw", RunBasePolygonDraw},
      {"/base/CircleDraw", "CircleDraw", RunBaseCircleDraw},
      {"/base/PathReplay", "PathReplay", RunBasePathReplay},
      {"/base/PathPlanning", "PathPlanning", RunBasePathPlanning},
      {"/expand/MapEditor", "MapEditor", RunExpandMapEditor},
      {"/expand/GraphicDrawing", "GraphicDrawing", RunExpandGraphicDrawing},
      {"/base/passableArea", "PassableArea", RunBasePassableArea},
      {"/base/editPassableArea", "EditPassableArea", RunBaseEditPassableArea},
      {"/outdoor/buildings", "OutdoorBuildings", RunOutdoorBuildings},
      {"/outdoor/pointCloud", "OutdoorPointCloud", RunOutdoorPointCloud},
      {"/scene/transportHub", "TransportHub", RunSceneTransportHub},
      {"/scene/publicClean", "PublicClean", RunScenePublicClean},
      {"/outdoor/mapTiles", "OutdoorMapTiles", RunOutdoorMapTiles},
      {"/outdoor/hdMap", "OutdoorHdMap", RunOutdoorHdMap},
      {"/scene/securityPatrol", "SecurityPatrol", RunSceneSecurityPatrol},
  };
}

class BicMapExampleDisplayTest
    : public DisplayTestFixture,
      public ::testing::WithParamInterface<BicMapExampleCase> {};

TEST_P(BicMapExampleDisplayTest, FeedAndDrawDoesNotCrash) {
  ASSERT_NE(nullptr, GetParam().run);
  EXPECT_NO_THROW(GetParam().run(this));
}

INSTANTIATE_TEST_SUITE_P(
    BicMapRouterExamples, BicMapExampleDisplayTest,
    ::testing::ValuesIn(AllBicMapExamples()),
    [](const ::testing::TestParamInfo<BicMapExampleCase>& info) {
      return info.param.name;
    });

TEST(BicMapExampleCatalog, CoversAllRouterExamples) {
  EXPECT_EQ(36u, AllBicMapExamples().size());
}

class StrataOverlayDisplaysTest : public DisplayTestFixture {};

TEST_F(StrataOverlayDisplaysTest, CanvasLabelLabelBubbleIotRobot3D) {
  TestableStrataCanvasLabelDisplay canvas("/strata/canvas_labels");
  TestableStrataLabelBubbleDisplay label_bubble("/strata/label_bubbles");
  TestableStrataIotBubbleDisplay iot("/strata/iot_bubbles");
  TestableStrataRobot3DDisplay robot3d("/strata/robot_3d_layers");
  EXPECT_NO_THROW(feedAndDraw(canvas, bicmap::MakeCanvasLabels()));
  EXPECT_NO_THROW(feedAndDraw(label_bubble, bicmap::MakeLabelBubbles()));
  EXPECT_NO_THROW(feedAndDraw(iot, bicmap::MakeIotBubbles()));
  EXPECT_NO_THROW(feedAndDraw(robot3d, bicmap::MakeRobot3DLayers()));
}

TEST_F(StrataOverlayDisplaysTest, BuildingExtrusionAndCanvasStyle) {
  TestableStrataBuildingDisplay building("/strata/markers");
  TestableStrataCanvasLabelDisplay canvas("/strata/canvas_labels");
  auto labels = bicmap::MakeCanvasLabels();
  labels.mutable_style()->mutable_text_color()->set_r(1.f);
  labels.mutable_style()->mutable_halo_color()->set_r(0.1f);
  labels.mutable_style()->mutable_halo_color()->set_g(0.2f);
  labels.mutable_style()->mutable_halo_color()->set_b(0.3f);
  labels.mutable_style()->set_halo_blur(4.f);
  EXPECT_NO_THROW(feedAndDraw(building, bicmap::MakeBuildingExtrusionMarkers()));
  EXPECT_NO_THROW(feedAndDraw(canvas, labels));
}

}  // namespace
}  // namespace tests
}  // namespace autoviz
