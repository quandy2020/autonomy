/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/orbisview/backend/adapters/mock/mock_source.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

#include "autonomy/orbisview/backend/common/render_schemas.h"

namespace autonomy {
namespace orbisview {
namespace adapters {
namespace {

constexpr char kPoseChannel[] = "/orbisview/mock/pose";
constexpr char kPathChannel[] = "/orbisview/mock/path";
constexpr char kMapChannel[] = "/orbisview/mock/map";
constexpr char kCostmapChannel[] = "/orbisview/mock/costmap";
constexpr char kFootprintChannel[] = "/orbisview/mock/footprint";
constexpr char kTfChannel[] = "/orbisview/mock/tf";
constexpr char kLaserChannel[] = "/orbisview/mock/laser";
constexpr char kImageChannel[] = "/orbisview/mock/image";
constexpr char kCloudChannel[] = "/orbisview/mock/pointcloud";
constexpr char kDepthChannel[] = "/orbisview/mock/depth";
constexpr char kExploreChannel[] = "/orbisview/mock/exploration";
constexpr char kNavChannel[] = "/orbisview/mock/navigation";
constexpr char kMappingChannel[] = "/orbisview/mock/mapping";
constexpr char kTwistChannel[] = "/orbisview/mock/twist";
constexpr char kChassisChannel[] = "/orbisview/mock/chassis";
constexpr char kObstaclesChannel[] = "/orbisview/mock/obstacles";
constexpr char kWorldChannel[] = "/orbisview/mock/world";
constexpr char kRouteChannel[] = "/orbisview/mock/route";
constexpr char kVectorMapChannel[] = "/orbisview/mock/vector_map";
constexpr char kPredictionChannel[] = "/orbisview/mock/prediction";
constexpr char kPlanningChannel[] = "/orbisview/mock/planning";
constexpr char kHmiChannel[] = "/orbisview/mock/hmi";
constexpr char kComponentsChannel[] = "/orbisview/mock/components";

int64_t NowNs() {
  using namespace std::chrono;
  return duration_cast<nanoseconds>(system_clock::now().time_since_epoch())
      .count();
}

std::string BuildOccupancyGridJson() {
  constexpr int w = 40;
  constexpr int h = 40;
  std::ostringstream oss;
  oss << "{\"resolution\":0.2,\"width\":" << w << ",\"height\":" << h
      << ",\"origin\":{\"x\":-4,\"y\":-4,\"yaw\":0},\"data\":[";
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      if (x || y) oss << ',';
      int v = 0;
      if (x == 0 || y == 0 || x == w - 1 || y == h - 1) {
        v = 100;
      } else if ((x - 20) * (x - 20) + (y - 20) * (y - 20) < 9) {
        v = 100;
      } else if ((x + y) % 17 == 0) {
        v = -1;
      }
      oss << v;
    }
  }
  oss << "]}";
  return oss.str();
}

std::string BuildCostmapJson() {
  constexpr int w = 30;
  constexpr int h = 30;
  constexpr double res = 0.1;
  std::ostringstream oss;
  oss << "{\"resolution\":" << res << ",\"width\":" << w << ",\"height\":" << h
      << ",\"origin\":{\"x\":-1.5,\"y\":-1.5,\"yaw\":0},\"data\":[";
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      if (x || y) oss << ',';
      int v = 0;
      // Lethal blob ahead along +x in map frame (static demo).
      const int dx = x - 22;
      const int dy = y - 15;
      if (dx * dx + dy * dy < 16) {
        v = 100;
      } else if ((x + y) % 11 == 0) {
        v = 50;
      }
      oss << v;
    }
  }
  oss << "]}";
  return oss.str();
}

std::string BuildFootprintJson() {
  return R"({"shape":"POLYGON","padding":0,"points":[)"
         R"({"x":0.45,"y":0.28,"yaw":0},{"x":0.45,"y":-0.28,"yaw":0},)"
         R"({"x":-0.45,"y":-0.28,"yaw":0},{"x":-0.45,"y":0.28,"yaw":0}]})";
}

std::string BuildVectorMapJson() {
  return R"({"lanes":[{"id":"L1","points":[[-3,-2],[-1,-1],[1,0],[3,1]]},{"id":"L2","points":[[-3,2],[-1,1],[1,0.5],[3,0]]}],"keepouts":[{"id":"K1","polygon":[[0.5,0.5],[1.5,0.5],[1.5,1.5],[0.5,1.5]]}]})";
}

}  // namespace

std::vector<core::ChannelInfo> MockSource::Channels() const {
  using namespace rendering;
  return {
      {kPoseChannel, kSchemaPose, kSchemaPose, true, true},
      {kPathChannel, kSchemaPath, kSchemaPath, true, true},
      {kMapChannel, kSchemaOccupancyGrid, kSchemaOccupancyGrid, true, true},
      {kCostmapChannel, kSchemaOccupancyGrid, kSchemaOccupancyGrid, true, true},
      {kFootprintChannel, kSchemaFootprint, kSchemaFootprint, true, true},
      {kTfChannel, kSchemaTfTree, kSchemaTfTree, true, true},
      {kLaserChannel, kSchemaLaserScan, kSchemaLaserScan, true, true},
      {kImageChannel, kSchemaImage, kSchemaImage, true, true},
      {kCloudChannel, kSchemaPointCloud2, kSchemaPointCloud2, true, true},
      {kDepthChannel, kSchemaDepthImage, kSchemaDepthImage, true, true},
      {kExploreChannel, kSchemaExploration, kSchemaExploration, true, true},
      {kNavChannel, kSchemaNavigation, kSchemaNavigation, true, true},
      {kMappingChannel, kSchemaMapping, kSchemaMapping, true, true},
      {kTwistChannel, kSchemaTwist2D, kSchemaTwist2D, true, true},
      {kChassisChannel, kSchemaChassis, kSchemaChassis, true, true},
      {kObstaclesChannel, kSchemaObstacles, kSchemaObstacles, true, true},
      {kWorldChannel, kSchemaWorld, kSchemaWorld, true, true},
      {kRouteChannel, kSchemaRoute, kSchemaRoute, true, true},
      {kVectorMapChannel, kSchemaVectorMap, kSchemaVectorMap, true, true},
      {kPredictionChannel, kSchemaPrediction, kSchemaPrediction, true, true},
      {kPlanningChannel, kSchemaPlanningDebug, kSchemaPlanningDebug, true, true},
      {kHmiChannel, kSchemaHmiStatus, kSchemaHmiStatus, true, true},
      {kComponentsChannel, kSchemaComponents, kSchemaComponents, true, true},
  };
}

void MockSource::Start() {
  if (running_.exchange(true)) return;
  thread_ = std::thread([this] { Loop(); });
}

void MockSource::Stop() {
  if (!running_.exchange(false)) return;
  if (thread_.joinable()) thread_.join();
}

void MockSource::SetNavGoal(double x, double y) {
  std::lock_guard<std::mutex> lock(goal_mutex_);
  has_goal_ = true;
  goal_x_ = x;
  goal_y_ = y;
  world_.SetGoal(x, y);
}

void MockSource::ClearNavGoal() {
  std::lock_guard<std::mutex> lock(goal_mutex_);
  has_goal_ = false;
  world_.ClearGoal();
}

bool MockSource::HasNavGoal(double* x, double* y) const {
  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (!has_goal_) return false;
  if (x) *x = goal_x_;
  if (y) *y = goal_y_;
  return true;
}

void MockSource::SetCmdVel(double vx, double wz) {
  std::lock_guard<std::mutex> lock(motion_mutex_);
  cmd_vx_ = vx;
  cmd_wz_ = wz;
  cmd_stamp_ = std::chrono::steady_clock::now();
  teleop_active_ = true;
}

void MockSource::SetRoute(std::vector<std::pair<double, double>> waypoints) {
  std::lock_guard<std::mutex> lock(route_mutex_);
  route_ = std::move(waypoints);
}

void MockSource::ClearRoute() {
  std::lock_guard<std::mutex> lock(route_mutex_);
  route_.clear();
}

void MockSource::EmitJson(const char* channel, const char* schema,
                          const std::string& frame, uint64_t* seq,
                          const std::string& json) {
  if (!emit_) return;
  core::StreamEnvelope env;
  env.channel = channel;
  env.schema = schema;
  env.timestamp_ns = NowNs();
  env.frame_id = frame;
  env.sequence = ++(*seq);
  env.encoding = "json";
  env.payload.assign(json.begin(), json.end());
  emit_(std::move(env));
}

void MockSource::Loop() {
  const std::string static_map = BuildOccupancyGridJson();
  const std::string static_vmap = BuildVectorMapJson();
  constexpr double kDt = 0.1;
  constexpr auto kCmdExpire = std::chrono::milliseconds(300);
  while (running_.load()) {
    double x = 0;
    double y = 0;
    double yaw = 0;
    double vx = 0;
    double wz = 0;
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      const auto now = std::chrono::steady_clock::now();
      if (teleop_active_ && (now - cmd_stamp_) > kCmdExpire) {
        cmd_vx_ = 0;
        cmd_wz_ = 0;
      }
      vx = cmd_vx_;
      wz = cmd_wz_;
      const bool teleop = teleop_active_;
      if (std::abs(vx) > 1e-6 || std::abs(wz) > 1e-6) {
        pose_yaw_ += wz * kDt;
        pose_x_ += vx * std::cos(pose_yaw_) * kDt;
        pose_y_ += vx * std::sin(pose_yaw_) * kDt;
      } else if (!teleop) {
        const double t = static_cast<double>(seq_pose_) * 0.1;
        pose_x_ = std::cos(t);
        pose_y_ = std::sin(t);
        pose_yaw_ = t;
      }
      x = pose_x_;
      y = pose_y_;
      yaw = pose_yaw_;
      // stash teleop for chassis outside lock via local — re-read below
      (void)teleop;
    }
    bool teleop_active = false;
    {
      std::lock_guard<std::mutex> lock(motion_mutex_);
      teleop_active = teleop_active_;
    }
    const double t = yaw;

    world_.SetPose(x, y, yaw);

    {
      std::ostringstream payload;
      payload << "{\"x\":" << x << ",\"y\":" << y << ",\"yaw\":" << yaw << '}';
      EmitJson(kPoseChannel, rendering::kSchemaPose, "map", &seq_pose_,
               payload.str());
    }
    {
      std::ostringstream payload;
      payload << "{\"vx\":" << vx << ",\"wz\":" << wz << '}';
      EmitJson(kTwistChannel, rendering::kSchemaTwist2D, "base_link",
               &seq_twist_, payload.str());
    }
    {
      core::WorldChassis ch;
      ch.vx = vx;
      ch.wz = wz;
      ch.gear = vx >= 0 ? "D" : "R";
      ch.throttle = std::min(1.0, std::abs(vx) / 0.8);
      ch.brake = (std::abs(vx) < 1e-3 && teleop_active) ? 0.2 : 0.0;
      ch.steering = std::max(-1.0, std::min(1.0, wz / 1.2));
      ch.driving_mode = teleop_active ? "MANUAL" : "AUTO";
      world_.SetChassis(ch);
      std::ostringstream payload;
      payload << "{\"vx\":" << ch.vx << ",\"wz\":" << ch.wz << ",\"gear\":\""
              << ch.gear << "\",\"throttle\":" << ch.throttle
              << ",\"brake\":" << ch.brake << ",\"steering\":" << ch.steering
              << ",\"driving_mode\":\"" << ch.driving_mode
              << "\",\"motion_model\":\"DIFF\"}";
      EmitJson(kChassisChannel, rendering::kSchemaChassis, "base_link",
               &seq_chassis_, payload.str());
    }
    {
      std::vector<core::WorldObstacle> obs;
      std::ostringstream payload;
      payload << "{\"obstacles\":[";
      for (int i = 0; i < 3; ++i) {
        if (i) payload << ',';
        const double a = t + i * 2.1;
        const double ox = x + 2.0 * std::cos(a);
        const double oy = y + 2.0 * std::sin(a);
        core::WorldObstacle o;
        o.id = i + 1;
        o.x = ox;
        o.y = oy;
        o.yaw = a + 1.57;
        o.length = 0.7;
        o.width = 0.45;
        o.type = (i == 0) ? "PEDESTRIAN" : "VEHICLE";
        o.vx = -0.2 * std::sin(a);
        o.vy = 0.2 * std::cos(a);
        obs.push_back(o);
        payload << "{\"id\":" << o.id << ",\"x\":" << o.x << ",\"y\":" << o.y
                << ",\"yaw\":" << o.yaw << ",\"length\":" << o.length
                << ",\"width\":" << o.width << ",\"type\":\"" << o.type
                << "\",\"vx\":" << o.vx << ",\"vy\":" << o.vy << '}';
      }
      payload << "]}";
      world_.SetObstacles(obs);
      EmitJson(kObstaclesChannel, rendering::kSchemaObstacles, "map",
               &seq_obstacles_, payload.str());
    }
    {
      std::ostringstream payload;
      payload << "{\"poses\":[";
      std::vector<core::WorldPose> path_poses;
      for (int i = 0; i < 8; ++i) {
        if (i) payload << ',';
        const double u = yaw + i * 0.15;
        const double px = x + 0.3 * i * std::cos(yaw);
        const double py = y + 0.3 * i * std::sin(yaw);
        path_poses.push_back({px, py, u, true});
        payload << "{\"x\":" << px << ",\"y\":" << py << ",\"yaw\":" << u
                << '}';
      }
      payload << "]}";
      world_.SetPathPoses(std::move(path_poses));
      EmitJson(kPathChannel, rendering::kSchemaPath, "map", &seq_path_,
               payload.str());
    }
    EmitJson(kWorldChannel, rendering::kSchemaWorld, "map", &seq_world_,
             world_.ToJson());

    {
      std::ostringstream payload;
      payload << "{\"angle_min\":-1.57,\"angle_increment\":0.05,\"range_max\":8,"
                 "\"ranges\":[";
      for (int i = 0; i < 64; ++i) {
        if (i) payload << ',';
        const double a = -1.57 + i * 0.05;
        payload << (1.5 + 0.4 * std::sin(t + a));
      }
      payload << "]}";
      EmitJson(kLaserChannel, rendering::kSchemaLaserScan, "base_link",
               &seq_laser_, payload.str());
    }
    {
      std::ostringstream payload;
      payload << "{\"width\":16,\"height\":12,\"encoding\":\"mono8\",\"data\":[";
      for (int i = 0; i < 16 * 12; ++i) {
        if (i) payload << ',';
        payload << ((i + tick_) % 256);
      }
      payload << "]}";
      EmitJson(kImageChannel, rendering::kSchemaImage, "camera", &seq_image_,
               payload.str());
    }
    {
      std::ostringstream payload;
      payload << "{\"points\":[";
      constexpr int kN = 200;
      for (int i = 0; i < kN; ++i) {
        if (i) payload << ',';
        const double a = (2.0 * 3.14159265358979323846 * i) / kN + t;
        const double r = 1.2 + 0.15 * std::sin(3 * a + t);
        payload << "{\"x\":" << (x + r * std::cos(a)) << ",\"y\":"
                << (y + r * std::sin(a)) << ",\"z\":"
                << (0.3 + 0.2 * std::sin(a * 2 + t)) << ",\"i\":"
                << (0.4 + 0.6 * (0.5 + 0.5 * std::sin(a + t))) << '}';
      }
      payload << "]}";
      EmitJson(kCloudChannel, rendering::kSchemaPointCloud2, "map", &seq_cloud_,
               payload.str());
    }
    {
      constexpr int dw = 32;
      constexpr int dh = 24;
      std::ostringstream payload;
      payload << "{\"width\":" << dw << ",\"height\":" << dh
              << ",\"encoding\":\"mono8\",\"data\":[";
      for (int v = 0; v < dh; ++v) {
        for (int u = 0; u < dw; ++u) {
          if (u || v) payload << ',';
          const double d =
              0.5 + 0.5 * std::sin((u + tick_) * 0.2) * std::cos(v * 0.15 + t);
          payload << static_cast<int>(std::clamp(d * 255.0, 0.0, 255.0));
        }
      }
      payload << "]}";
      EmitJson(kDepthChannel, rendering::kSchemaDepthImage, "camera",
               &seq_depth_, payload.str());
    }
    {
      // Prediction trajectories for obstacles.
      std::ostringstream payload;
      payload << "{\"obstacles\":[";
      for (int i = 0; i < 3; ++i) {
        if (i) payload << ',';
        const double a = t + i * 2.1;
        const double ox = x + 2.0 * std::cos(a);
        const double oy = y + 2.0 * std::sin(a);
        payload << "{\"id\":" << (i + 1) << ",\"trajectory\":[";
        for (int k = 0; k < 6; ++k) {
          if (k) payload << ',';
          payload << "{\"x\":" << (ox + 0.15 * k * std::cos(a + 1.57))
                  << ",\"y\":" << (oy + 0.15 * k * std::sin(a + 1.57)) << '}';
        }
        payload << "]}";
      }
      payload << "]}";
      EmitJson(kPredictionChannel, rendering::kSchemaPrediction, "map",
               &seq_pred_, payload.str());
    }
    {
      std::ostringstream payload;
      payload << "{\"speed\":[";
      for (int i = 0; i < 20; ++i) {
        if (i) payload << ',';
        payload << (0.4 + 0.2 * std::sin(t + i * 0.2));
      }
      payload << "],\"acceleration\":[";
      for (int i = 0; i < 20; ++i) {
        if (i) payload << ',';
        payload << (0.05 * std::cos(t + i * 0.25));
      }
      payload << "],\"heading_error\":[";
      for (int i = 0; i < 20; ++i) {
        if (i) payload << ',';
        payload << (0.02 * std::sin(t * 2 + i * 0.1));
      }
      payload << "],\"station_error\":[";
      for (int i = 0; i < 20; ++i) {
        if (i) payload << ',';
        payload << (0.03 * std::cos(t + i * 0.15));
      }
      payload << "]}";
      EmitJson(kPlanningChannel, rendering::kSchemaPlanningDebug, "map",
               &seq_planning_, payload.str());
    }

    if (tick_ % 10 == 0) {
      EmitJson(kMapChannel, rendering::kSchemaOccupancyGrid, "map", &seq_map_,
               static_map);
      EmitJson(kCostmapChannel, rendering::kSchemaOccupancyGrid, "map",
               &seq_costmap_, BuildCostmapJson());
      EmitJson(kFootprintChannel, rendering::kSchemaFootprint, "base_link",
               &seq_footprint_, BuildFootprintJson());
      EmitJson(kVectorMapChannel, rendering::kSchemaVectorMap, "map",
               &seq_vmap_, static_vmap);

      std::ostringstream tf;
      tf << "{\"transforms\":["
         << "{\"parent\":\"map\",\"child\":\"odom\",\"x\":0,\"y\":0,\"yaw\":0},"
         << "{\"parent\":\"odom\",\"child\":\"base_link\",\"x\":" << x
         << ",\"y\":" << y << ",\"yaw\":" << t << "}"
         << "]}";
      EmitJson(kTfChannel, rendering::kSchemaTfTree, "map", &seq_tf_, tf.str());

      std::ostringstream explore;
      explore << "{\"frontier_count\":" << (3 + (tick_ % 5))
              << ",\"covered_ratio\":" << std::min(0.95, 0.1 + tick_ * 0.002)
              << ",\"active_waypoint\":{\"x\":" << x << ",\"y\":" << y << "}}";
      EmitJson(kExploreChannel, rendering::kSchemaExploration, "map",
               &seq_explore_, explore.str());

      double gx = 2.0;
      double gy = 1.0;
      bool has_goal = false;
      {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        has_goal = has_goal_;
        gx = goal_x_;
        gy = goal_y_;
      }
      std::ostringstream nav;
      nav << "{\"state\":\"" << (has_goal ? "FOLLOWING" : "IDLE")
          << "\",\"goal\":{\"x\":" << gx << ",\"y\":" << gy
          << "},\"has_goal\":" << (has_goal ? "true" : "false")
          << ",\"distance_remaining\":"
          << (has_goal ? (std::abs(gx - x) + std::abs(gy - y)) : 0.0) << '}';
      EmitJson(kNavChannel, rendering::kSchemaNavigation, "map", &seq_nav_,
               nav.str());

      std::ostringstream mapping;
      mapping << "{\"mode\":\"SLAM\",\"keyframes\":" << (10 + tick_ / 10)
              << ",\"loop_closures\":" << (tick_ / 50) << '}';
      EmitJson(kMappingChannel, rendering::kSchemaMapping, "map",
               &seq_map_task_, mapping.str());

      std::vector<std::pair<double, double>> route_copy;
      {
        std::lock_guard<std::mutex> lock(route_mutex_);
        route_copy = route_;
      }
      std::ostringstream route;
      route << "{\"state\":\"" << (route_copy.empty() ? "IDLE" : "ACTIVE")
            << "\",\"waypoints\":[";
      for (size_t i = 0; i < route_copy.size(); ++i) {
        if (i) route << ',';
        route << "{\"x\":" << route_copy[i].first << ",\"y\":"
              << route_copy[i].second << '}';
      }
      route << "]}";
      EmitJson(kRouteChannel, rendering::kSchemaRoute, "map", &seq_route_,
               route.str());

      EmitJson(kHmiChannel, rendering::kSchemaHmiStatus, "map", &seq_hmi_,
               "{\"mode\":\"default\",\"modes\":[{\"id\":\"default\","
               "\"title\":\"Default\"},{\"id\":\"pnc\",\"title\":\"PNC\"},{"
               "\"id\":\"mapping\",\"title\":\"Mapping\"}]}");
      EmitJson(kComponentsChannel, rendering::kSchemaComponents, "map",
               &seq_components_,
               "{\"components\":["
               "{\"id\":\"localization\",\"title\":\"localization\","
               "\"expected\":true,\"healthy\":true,\"delay_ms\":12,"
               "\"status\":\"OK\"},"
               "{\"id\":\"perception\",\"title\":\"perception\","
               "\"expected\":true,\"healthy\":true,\"delay_ms\":18,"
               "\"status\":\"OK\"},"
               "{\"id\":\"planning\",\"title\":\"planning\",\"expected\":true,"
               "\"healthy\":true,\"delay_ms\":22,\"status\":\"OK\"},"
               "{\"id\":\"control\",\"title\":\"control\",\"expected\":true,"
               "\"healthy\":true,\"delay_ms\":8,\"status\":\"OK\"}"
               "]}");
    }

    ++tick_;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

}  // namespace adapters
}  // namespace orbisview
}  // namespace autonomy
