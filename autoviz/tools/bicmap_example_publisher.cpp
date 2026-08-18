/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <chrono>
#include <cctype>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "tests/bicmap_message_fixtures.hpp"

namespace autoviz {
namespace tools {

struct ExampleScene {
  std::string name;
  std::string route;
  std::string title;
  bool animate_location = false;
  std::function<void(const std::string& prefix,
                     const std::shared_ptr<autolink::Node>& node)> publish;
};

namespace {
using autoviz::tests::bicmap::MakeFloorInfoArray;
using autoviz::tests::bicmap::MakeBuildingMarkers;
using autoviz::tests::bicmap::MakeCanvasLabels;
using autoviz::tests::bicmap::MakeCircleMarker;
using autoviz::tests::bicmap::MakeCubeMarker;
using autoviz::tests::bicmap::MakeGraphicDrawingMarkers;
using autoviz::tests::bicmap::MakeIotBubbles;
using autoviz::tests::bicmap::MakeLabelBubbles;
using autoviz::tests::bicmap::MakeLineStripMarker;
using autoviz::tests::bicmap::MakeOccupancyGrid;
using autoviz::tests::bicmap::MakePath;
using autoviz::tests::bicmap::MakePoiMarkers;
using autoviz::tests::bicmap::MakePointCloud2;
using autoviz::tests::bicmap::MakeRectangleMarker;
using autoviz::tests::bicmap::MakeRoadGraph;
using autoviz::tests::bicmap::MakeRobot3DLayers;
using autoviz::tests::bicmap::MakeRobotFovMarker;
using autoviz::tests::bicmap::MakeRobotMarkers;
using autoviz::tests::bicmap::MakeSemanticZones;
using autoviz::tests::bicmap::MakeSpaceMarkers;

template <typename ProtoT>
void WriteProto(const std::shared_ptr<autolink::Node>& node,
                const std::string& channel, const ProtoT& proto) {
  static std::map<std::string, std::shared_ptr<autolink::Writer<ProtoT>>> writers;
  auto& writer = writers[channel];
  if (writer == nullptr) {
    writer = node->CreateWriter<ProtoT>(channel);
    if (writer == nullptr) {
      LOG(WARNING) << "Failed to create writer for " << channel;
      return;
    }
  }
  writer->Write(proto);
}

void PublishCommonChannels(const std::string& prefix,
                           const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/floors", MakeFloorInfoArray());
}

void PublishSlam(const std::string& prefix, const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(32, 24));
}

void PublishBuildMap(const std::string& prefix, const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(48, 48));
}

void PublishRobotFollow(const std::string& prefix, const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(32, 24));
  WriteProto(node, prefix + "/robot_markers", MakeRobotMarkers(1));
  WriteProto(node, prefix + "/markers", MakeRobotFovMarker());
}

void PublishMapTools(const std::string& prefix, const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(64, 64));
}

void PublishPoiMarkers(const std::string& prefix, const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(32, 24));
  WriteProto(node, prefix + "/poi_markers", MakePoiMarkers(4));
}

void PublishPoiAdvanced(const std::string& prefix, const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(32, 24));
  WriteProto(node, prefix + "/poi_markers", MakePoiMarkers(3, true));
  WriteProto(node, prefix + "/label_bubbles", MakeLabelBubbles(3));
  WriteProto(node, prefix + "/canvas_labels", MakeCanvasLabels(2));
}

void PublishRelocate(const std::string& prefix, const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(32, 24));
  auto robots = MakeRobotMarkers(1, "idle");
  robots.mutable_robots(0)->mutable_lng_lat()->set_x(3.5);
  robots.mutable_robots(0)->mutable_lng_lat()->set_y(2.0);
  WriteProto(node, prefix + "/robot_markers", robots);
}

void PublishLocation(const std::string& prefix, const std::shared_ptr<autolink::Node>& node,
                     int tick) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(32, 24));
  auto robots = MakeRobotMarkers(1);
  robots.mutable_robots(0)->mutable_lng_lat()->set_x(0.5 + 0.2 * (tick % 10));
  WriteProto(node, prefix + "/robot_markers", robots);
}

void PublishSingleNavigation(const std::string& prefix,
                             const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(32, 24));
  WriteProto(node, prefix + "/path", MakePath(8));
  WriteProto(node, prefix + "/robot_markers", MakeRobotMarkers(1));
}

void PublishLoad3D(const std::string& prefix, const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(32, 24));
  WriteProto(node, prefix + "/markers", MakeCubeMarker(0));
  WriteProto(node, prefix + "/robot_3d_layers", MakeRobot3DLayers(1));
}

void PublishPointCloud(const std::string& prefix, const std::shared_ptr<autolink::Node>& node,
                       int count) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(32, 24));
  WriteProto(node, prefix + "/pointcloud", MakePointCloud2(count));
}

void PublishSemanticMap(const std::string& prefix, const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(32, 24));
  WriteProto(node, prefix + "/semantic_zones", MakeSemanticZones("activity_area"));
}

void PublishSpace(const std::string& prefix, const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(32, 24));
  WriteProto(node, prefix + "/markers", MakeSpaceMarkers());
}

void PublishRobotGuideTour(const std::string& prefix, const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(40, 30));
  WriteProto(node, prefix + "/poi_markers", MakePoiMarkers(5));
  WriteProto(node, prefix + "/path", MakePath(10));
  WriteProto(node, prefix + "/robot_markers", MakeRobotMarkers(1, "running"));
  WriteProto(node, prefix + "/canvas_labels", MakeCanvasLabels(3));
}

void PublishHotelDelivery(const std::string& prefix, const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(40, 30));
  WriteProto(node, prefix + "/path", MakePath(6));
  WriteProto(node, prefix + "/robot_markers", MakeRobotMarkers(1, "running"));
  WriteProto(node, prefix + "/iot_bubbles", MakeIotBubbles(2));
}

void PublishMallRobotMonitor(const std::string& prefix,
                             const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(40, 30));
  WriteProto(node, prefix + "/robot_markers", MakeRobotMarkers(2));
  WriteProto(node, prefix + "/poi_markers", MakePoiMarkers(3));
  WriteProto(node, prefix + "/markers", MakeRobotFovMarker());
}

void PublishIndoorCleaning(const std::string& prefix, const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(40, 30));
  WriteProto(node, prefix + "/semantic_zones", MakeSemanticZones("waiting"));
  WriteProto(node, prefix + "/path", MakePath(12));
  WriteProto(node, prefix + "/robot_markers", MakeRobotMarkers(1, "running"));
}

void PublishCommunityInspect(const std::string& prefix,
                             const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(40, 30));
  WriteProto(node, prefix + "/robot_markers", MakeRobotMarkers(1));
  WriteProto(node, prefix + "/path", MakePath(7));
  WriteProto(node, prefix + "/poi_markers", MakePoiMarkers(2));
}

void PublishPolylineDrawing(const std::string& prefix, const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(32, 24));
  WriteProto(node, prefix + "/markers", MakeLineStripMarker(0, 6));
}

void PublishRectangleDraw(const std::string& prefix, const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(32, 24));
  WriteProto(node, prefix + "/markers", MakeRectangleMarker(0));
}

void PublishPolygonDraw(const std::string& prefix, const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(32, 24));
  auto array = MakeLineStripMarker(0, 5);
  auto* close = array.mutable_markers(0)->add_points();
  const auto& first = array.markers(0).points(0);
  close->set_x(first.x());
  close->set_y(first.y());
  WriteProto(node, prefix + "/markers", array);
}

void PublishCircleDraw(const std::string& prefix, const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(32, 24));
  WriteProto(node, prefix + "/markers", MakeCircleMarker(0));
}

void PublishPathReplay(const std::string& prefix, const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(32, 24));
  WriteProto(node, prefix + "/path", MakePath(20));
}

void PublishPathPlanning(const std::string& prefix, const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(40, 30));
  WriteProto(node, prefix + "/path", MakePath(9));
}

void PublishMapEditor(const std::string& prefix, const std::shared_ptr<autolink::Node>& node) {
  auto grid = MakeOccupancyGrid(20, 20);
  grid.set_data(50, 100);
  grid.set_data(120, 0);
  WriteProto(node, prefix + "/map", grid);
}

void PublishGraphicDrawing(const std::string& prefix, const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(32, 24));
  WriteProto(node, prefix + "/markers", MakeGraphicDrawingMarkers());
}

void PublishPassableArea(const std::string& prefix, const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(32, 24));
  WriteProto(node, prefix + "/semantic_zones", MakeSemanticZones("activity_area"));
}

void PublishEditPassableArea(const std::string& prefix,
                             const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(32, 24));
  WriteProto(node, prefix + "/semantic_zones", MakeSemanticZones("speed_limit"));
}

void PublishOutdoorBuildings(const std::string& prefix,
                             const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(64, 64, 0.1f));
  WriteProto(node, prefix + "/markers", MakeBuildingMarkers());
}

void PublishOutdoorPointCloud(const std::string& prefix,
                              const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(64, 64, 0.1f));
  WriteProto(node, prefix + "/pointcloud", MakePointCloud2(800));
}

void PublishTransportHub(const std::string& prefix, const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(48, 36));
  WriteProto(node, prefix + "/road_graph", MakeRoadGraph());
  WriteProto(node, prefix + "/poi_markers", MakePoiMarkers(4, true));
  WriteProto(node, prefix + "/label_bubbles", MakeLabelBubbles(2));
}

void PublishPublicClean(const std::string& prefix, const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(40, 30));
  WriteProto(node, prefix + "/semantic_zones", MakeSemanticZones("forbidden"));
  WriteProto(node, prefix + "/robot_markers", MakeRobotMarkers(1, "running"));
}

void PublishOutdoorMapTiles(const std::string& prefix, const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(128, 128, 0.1f));
}

void PublishOutdoorHdMap(const std::string& prefix, const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(64, 64, 0.1f));
  WriteProto(node, prefix + "/road_graph", MakeRoadGraph());
}

void PublishSecurityPatrol(const std::string& prefix, const std::shared_ptr<autolink::Node>& node) {
  WriteProto(node, prefix + "/map", MakeOccupancyGrid(48, 36));
  WriteProto(node, prefix + "/semantic_zones", MakeSemanticZones("forbidden"));
  WriteProto(node, prefix + "/path", MakePath(15));
  WriteProto(node, prefix + "/robot_markers", MakeRobotMarkers(1, "running"));
  WriteProto(node, prefix + "/markers", MakeRobotFovMarker());
}

}  // namespace

std::vector<ExampleScene> BuildExamples() {
  return {
      {"Slam", "/indoor/slam", "SLAM地图显示示例", false, PublishSlam},
      {"BuildMap", "/indoor/buildMap", "机器人建图", false, PublishBuildMap},
      {"RobotFollow", "/indoor/robotFollow", "机器人视角跟随", false, PublishRobotFollow},
      {"MapTools", "/base/mapTools", "地图工具", false, PublishMapTools},
      {"POIMarkers", "/base/POIMarkers", "点位Marker", false, PublishPoiMarkers},
      {"POIadvancedLabel", "/expand/POIadvancedLabel", "POI高级标注", false, PublishPoiAdvanced},
      {"Relocate", "/indoor/relocate", "机器人重定位", false, PublishRelocate},
      {"Location", "/indoor/location", "实时位置更新", true, PublishSlam},
      {"SingleNavigation", "/indoor/singleNavigation", "单点导航", false, PublishSingleNavigation},
      {"load3D", "/indoor/load3D", "加载3D模型", false, PublishLoad3D},
      {"load3dControl", "/indoor/load3dControl", "3D模型控制", false, PublishLoad3D},
      {"PointCloud", "/indoor/pointCloud", "二三维点云示例", false,
       [](const std::string& p, const std::shared_ptr<autolink::Node>& n) {
         PublishPointCloud(p, n, 200);
       }},
      {"SemanticMap", "/indoor/semanticMap", "语义地图分割", false, PublishSemanticMap},
      {"Space", "/indoor/load3dMarker", "三维空间数据渲染", false, PublishSpace},
      {"RobotGuideTour", "/scene/robotGuideTour", "博物馆导览", false, PublishRobotGuideTour},
      {"HotelDelivery", "/scene/hotelDelivery", "酒店配送", false, PublishHotelDelivery},
      {"MallRobotMonitor", "/scene/mallRobotMonitor", "商场监控", false, PublishMallRobotMonitor},
      {"IndoorCleaning", "/scene/indoorCleaning", "扫地清扫", false, PublishIndoorCleaning},
      {"CommunityInspect", "/scene/communityInspect", "社区巡检", false, PublishCommunityInspect},
      {"PolylineDrawing", "/base/PolylineDrawing", "折线绘制", false, PublishPolylineDrawing},
      {"RectangleDraw", "/base/RectangleDraw", "矩形绘制", false, PublishRectangleDraw},
      {"PolygonDraw", "/base/PolygonDraw", "多边形绘制", false, PublishPolygonDraw},
      {"CircleDraw", "/base/CircleDraw", "圆形绘制", false, PublishCircleDraw},
      {"PathReplay", "/base/PathReplay", "路径回放", false, PublishPathReplay},
      {"PathPlanning", "/base/PathPlanning", "路径规划", false, PublishPathPlanning},
      {"MapEditor", "/expand/MapEditor", "地图编辑", false, PublishMapEditor},
      {"GraphicDrawing", "/expand/GraphicDrawing", "图形绘制", false, PublishGraphicDrawing},
      {"PassableArea", "/base/passableArea", "可通行区域", false, PublishPassableArea},
      {"EditPassableArea", "/base/editPassableArea", "可通行区域编辑", false, PublishEditPassableArea},
      {"OutdoorBuildings", "/outdoor/buildings", "室外建筑物", false, PublishOutdoorBuildings},
      {"OutdoorPointCloud", "/outdoor/pointCloud", "室外点云", false, PublishOutdoorPointCloud},
      {"TransportHub", "/scene/transportHub", "三站一场引导", false, PublishTransportHub},
      {"PublicClean", "/scene/publicClean", "公共区域清洁", false, PublishPublicClean},
      {"OutdoorMapTiles", "/outdoor/mapTiles", "地图瓦片加载", false, PublishOutdoorMapTiles},
      {"OutdoorHdMap", "/outdoor/hdMap", "高精地图加载", false, PublishOutdoorHdMap},
      {"SecurityPatrol", "/scene/securityPatrol", "园区安防巡逻", false, PublishSecurityPatrol},
  };
}

const ExampleScene* FindExample(const std::vector<ExampleScene>& examples,
                                const std::string& name) {
  std::string lower = name;
  for (char& c : lower) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  const ExampleScene* match = nullptr;
  for (const auto& scene : examples) {
    std::string key = scene.name;
    for (char& c : key) {
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    if (key == lower) {
      return &scene;
    }
    if (key.find(lower) != std::string::npos) {
      if (match != nullptr) {
        return nullptr;
      }
      match = &scene;
    }
  }
  return match;
}

void PublishScene(const ExampleScene& scene, const std::string& prefix,
                  const std::shared_ptr<autolink::Node>& node, int tick) {
  PublishCommonChannels(prefix, node);
  if (scene.animate_location) {
    PublishLocation(prefix, node, tick);
    return;
  }
  scene.publish(prefix, node);
}

void ListExamples() {
  for (const auto& scene : BuildExamples()) {
    std::cout << scene.name << '\t' << scene.route << '\t' << scene.title << '\n';
  }
}

}  // namespace tools
}  // namespace autoviz

DEFINE_string(channel_prefix, "/strata", "Topic prefix for BICMap demo messages.");
DEFINE_string(example, "Slam", "BICMap example name (see --list).");
DEFINE_bool(list, false, "List all BICMap examples and exit.");
DEFINE_bool(cycle, false, "Rotate through all examples.");
DEFINE_double(rate_hz, 2.0, "Publish rate in Hz.");
DEFINE_double(cycle_interval_sec, 8.0, "Seconds per example in cycle mode.");

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_list) {
    autoviz::tools::ListExamples();
    return EXIT_SUCCESS;
  }

  if (!autolink::Init(argv[0])) {
    LOG(ERROR) << "autolink::Init failed";
    return EXIT_FAILURE;
  }

  auto node = autolink::CreateNode("autoviz_bicmap_publisher");
  const auto examples = autoviz::tools::BuildExamples();
  const double sleep_s = FLAGS_rate_hz > 0. ? 1.0 / FLAGS_rate_hz : 0.5;

  if (FLAGS_cycle) {
    std::cout << "Cycling " << examples.size() << " BICMap examples every "
              << FLAGS_cycle_interval_sec << "s\n";
    std::cout << "autoviz: autoviz -c config/default.autoviz (Fixed Frame=map)\n";
    size_t index = 0;
    int tick = 0;
    auto last_switch = std::chrono::steady_clock::now();
    while (autolink::OK()) {
      autoviz::tools::PublishScene(examples[index], FLAGS_channel_prefix, node, tick);
      const auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration<double>(now - last_switch).count() >=
          FLAGS_cycle_interval_sec) {
        index = (index + 1) % examples.size();
        last_switch = now;
        std::cout << "-> " << examples[index].name << " (" << examples[index].title
                  << ")\n";
      }
      ++tick;
      std::this_thread::sleep_for(std::chrono::duration<double>(sleep_s));
    }
  } else {
    const autoviz::tools::ExampleScene* scene =
        autoviz::tools::FindExample(examples, FLAGS_example);
    if (scene == nullptr) {
      LOG(ERROR) << "Unknown example: " << FLAGS_example << " (use --list)";
      autolink::Clear();
      return EXIT_FAILURE;
    }
    std::cout << "Publishing BICMap example: " << scene->name << " — " << scene->title
              << "\nRoute: " << scene->route << "\n";
    std::cout << "Channels: " << FLAGS_channel_prefix << "/{map,floors,...}\n";
    std::cout << "Start autoviz in another terminal, or use:\n"
              << "  scripts/run_bicmap_example.sh " << scene->name << " autoviz\n";
    int tick = 0;
    while (autolink::OK()) {
      autoviz::tools::PublishScene(*scene, FLAGS_channel_prefix, node, tick);
      ++tick;
      std::this_thread::sleep_for(std::chrono::duration<double>(sleep_s));
    }
  }

  autolink::Clear();
  google::ShutdownGoogleLogging();
  return EXIT_SUCCESS;
}
