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

automsgs::msgs::map_msgs::OccupancyGrid MakePaletteProbeGrid() {
  automsgs::msgs::map_msgs::OccupancyGrid grid;
  auto* info = grid.mutable_info();
  info->set_resolution(1.f);
  info->set_width(5);
  info->set_height(1);
  info->mutable_origin()->mutable_orientation()->set_w(1.0);
  grid.add_data(-1);
  grid.add_data(0);
  grid.add_data(50);
  grid.add_data(99);
  grid.add_data(100);
  return grid;
}

automsgs::msgs::geometry_msgs::TransformStamped MakeTransform(
    const std::string& parent, const std::string& child, double stamp_seconds,
    double x) {
  automsgs::msgs::geometry_msgs::TransformStamped message;
  message.mutable_header()->set_frame_id(parent);
  message.set_child_frame_id(child);
  message.mutable_header()->mutable_stamp()->set_sec(
      static_cast<int32_t>(stamp_seconds));
  message.mutable_header()->mutable_stamp()->set_nanosec(static_cast<uint32_t>(
      (stamp_seconds - static_cast<int32_t>(stamp_seconds)) * 1e9));
  message.mutable_transform()->mutable_translation()->set_x(x);
  message.mutable_transform()->mutable_rotation()->set_w(1.0);
  return message;
}

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

TEST_F(StrataOverlayDisplaysTest, MapDisplayRendersOccupancyGridAsTexturedQuad) {
  TestableMapDisplay display("/strata/map");
  auto grid = bicmap::MakeOccupancyGrid(32, 24);
  EXPECT_NO_THROW(feedAndDraw(display, grid));
  EXPECT_EQ(1u, scene().texturedBatches().size());
  EXPECT_TRUE(scene().pointVertices().empty());
}

TEST_F(StrataOverlayDisplaysTest, MapDisplayUsesRvizMapPalette) {
  TestableMapDisplay display("/strata/map");
  display.setPropertyValue("color_scheme", "map");
  feedAndDraw(display, MakePaletteProbeGrid());
  ASSERT_EQ(1u, scene().texturedBatches().size());
  const QImage& image = scene().texturedBatches().front().image;
  const auto expect_rgb = [&](int x, const QColor& expected) {
    const QColor actual = image.pixelColor(x, 0);
    EXPECT_EQ(expected.red(), actual.red());
    EXPECT_EQ(expected.green(), actual.green());
    EXPECT_EQ(expected.blue(), actual.blue());
  };
  // RViz makeMapPalette: unknown / free / mid / near-lethal / lethal
  expect_rgb(0, QColor(0x70, 0x89, 0x86));
  expect_rgb(1, QColor(255, 255, 255));
  expect_rgb(2, QColor(128, 128, 128));  // 255 - (255*50)/100
  expect_rgb(3, QColor(3, 3, 3));
  expect_rgb(4, QColor(0, 0, 0));
}

TEST_F(StrataOverlayDisplaysTest, MapDisplayUsesRvizCostmapPalette) {
  TestableMapDisplay display("/strata/map");
  display.setPropertyValue("color_scheme", "costmap");
  feedAndDraw(display, MakePaletteProbeGrid());
  ASSERT_EQ(1u, scene().texturedBatches().size());
  const QImage& image = scene().texturedBatches().front().image;
  const auto expect_rgba = [&](int x, const QColor& expected) {
    const QColor actual = image.pixelColor(x, 0);
    EXPECT_EQ(expected.red(), actual.red());
    EXPECT_EQ(expected.green(), actual.green());
    EXPECT_EQ(expected.blue(), actual.blue());
    EXPECT_EQ(expected.alpha(), actual.alpha());
  };
  // RViz makeCostmapPalette: free is fully transparent (alpha 0).
  expect_rgba(0, QColor(0x70, 0x89, 0x86, 179));
  expect_rgba(1, QColor(0, 0, 0, 0));
  expect_rgba(2, QColor(127, 0, 128, 179));
  expect_rgba(3, QColor(0, 255, 255, 179));
  expect_rgba(4, QColor(255, 0, 255, 179));
}

TEST_F(StrataOverlayDisplaysTest, MapDisplayUsesRvizRawPalette) {
  TestableMapDisplay display("/strata/map");
  display.setPropertyValue("color_scheme", "raw");
  feedAndDraw(display, MakePaletteProbeGrid());
  ASSERT_EQ(1u, scene().texturedBatches().size());
  const QImage& image = scene().texturedBatches().front().image;
  const auto expect_rgb = [&](int x, int expected) {
    const QColor actual = image.pixelColor(x, 0);
    EXPECT_EQ(expected, actual.red());
    EXPECT_EQ(expected, actual.green());
    EXPECT_EQ(expected, actual.blue());
  };
  expect_rgb(0, 255);
  expect_rgb(1, 0);
  expect_rgb(2, 50);
  expect_rgb(3, 99);
  expect_rgb(4, 100);
}

TEST_F(StrataOverlayDisplaysTest, MapDisplayRebuildsImmediatelyOnPropertyChange) {
  TestableMapDisplay display("/strata/map");
  display.setContext(&context());
  display.setDisplayName(display.typeId());
  display.feedMessage(MakePaletteProbeGrid());

  scene().clear();
  display.drawOverlay(scene());
  ASSERT_EQ(1u, scene().texturedBatches().size());
  {
    const QColor actual = scene().texturedBatches().front().image.pixelColor(1, 0);
    EXPECT_EQ(255, actual.red());
    EXPECT_EQ(255, actual.green());
    EXPECT_EQ(255, actual.blue());
    EXPECT_EQ(179, actual.alpha());
  }

  display.setPropertyValue("color_scheme", "costmap");
  scene().clear();
  display.drawOverlay(scene());
  ASSERT_EQ(1u, scene().texturedBatches().size());
  {
    // Costmap free cells stay fully transparent regardless of Alpha.
    const QColor actual = scene().texturedBatches().front().image.pixelColor(1, 0);
    EXPECT_EQ(0, actual.red());
    EXPECT_EQ(0, actual.green());
    EXPECT_EQ(0, actual.blue());
    EXPECT_EQ(0, actual.alpha());
  }
}

TEST_F(StrataOverlayDisplaysTest, MapDisplayUsesMessageTimestampForTransformWhenEnabled) {
  transform::Buffer::Instance()->clear();
  transform::Buffer::Instance()->setTransform(
      MakeTransform("map", "odom", 10.0, 10.0), "test");
  transform::Buffer::Instance()->setTransform(
      MakeTransform("map", "odom", 5.0, 2.0), "test");

  auto grid = bicmap::MakeOccupancyGrid(2, 2);
  grid.mutable_header()->set_frame_id("odom");
  grid.mutable_header()->mutable_stamp()->set_sec(5);

  TestableMapDisplay latest_display("/strata/map");
  feedAndDraw(latest_display, grid);
  ASSERT_EQ(1u, scene().texturedBatches().size());
  const float latest_x = scene().texturedBatches().front().vertices.front().position.x();
  EXPECT_NEAR(9.f, latest_x, 1e-4f);

  TestableMapDisplay stamped_display("/strata/map");
  stamped_display.setPropertyValue("use_timestamp", "true");
  feedAndDraw(stamped_display, grid);
  ASSERT_EQ(1u, scene().texturedBatches().size());
  const float stamped_x = scene().texturedBatches().front().vertices.front().position.x();
  EXPECT_NEAR(1.f, stamped_x, 1e-4f);
}

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
