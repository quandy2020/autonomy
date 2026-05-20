/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/

#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"

#include <memory>
#include <sstream>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/layers/denoise_layer.hpp"
#include "autonomy/map/costmap_2d/layers/inflation_layer.hpp"
#include "autonomy/map/costmap_2d/layers/obstacle_layer.hpp"
#include "autonomy/map/costmap_2d/layers/range_sensor_layer.hpp"
#include "autonomy/map/costmap_2d/layers/static_layer.hpp"
#include "autonomy/map/costmap_2d/layers/voxel_layer.hpp"
#include "autonomy/map/costmap_2d/utils/occ_grid_values.hpp"
#include "autonomy/map/costmap_2d/utils/validate_messages.hpp"
#include "autonomy/map/utils/data_loader_utils.hpp"
#include "autonomy/transform/tf2/exceptions.h"
#include "autonomy/transform/tf2/utils.h"

namespace {

using autonomy::map::costmap_2d::Layer;

std::shared_ptr<Layer> CreateCostmapLayerInstance(
    const std::string& plugin_name, const std::string& plugin_type) {
    const auto matches = [&](const char* short_name,
                             const char* class_suffix) -> bool {
        return plugin_name == short_name ||
               plugin_type.find(class_suffix) != std::string::npos;
    };

    if (matches("static_layer", "StaticLayer")) {
        return std::make_shared<autonomy::map::costmap_2d::StaticLayer>();
    }
    if (matches("obstacle_layer", "ObstacleLayer")) {
        return std::make_shared<autonomy::map::costmap_2d::ObstacleLayer>();
    }
    if (matches("inflation_layer", "InflationLayer")) {
        return std::make_shared<autonomy::map::costmap_2d::InflationLayer>();
    }
    if (matches("denoise_layer", "DenoiseLayer")) {
        return std::make_shared<autonomy::map::costmap_2d::DenoiseLayer>();
    }
    if (matches("voxel_layer", "VoxelLayer")) {
        return std::make_shared<autonomy::map::costmap_2d::VoxelLayer>();
    }
    if (matches("range_sensor_layer", "RangeSensorLayer")) {
        return std::make_shared<autonomy::map::costmap_2d::RangeSensorLayer>();
    }
    return nullptr;
}

}  // namespace

namespace autonomy {
namespace map {
namespace costmap_2d {

Costmap2DWrapper::Costmap2DWrapper(const proto::Costmap2DOptions& options,
                                   const std::string& name)
    : options_{options},
      name_{name},
      default_plugins_{"static_layer", "obstacle_layer", "inflation_layer"},
      default_types_{"autonomy::map::costmap_2d::StaticLayer",
                     "autonomy::map::costmap_2d::ObstacleLayer",
                     "autonomy::map::costmap_2d::InflationLayer"} {
    // declare_parameter("map_topic",
    // rclcpp::ParameterValue(std::string("map")));
    is_lifecycle_follower_ = false;
    init();
}

Costmap2DWrapper::~Costmap2DWrapper() {}

void Costmap2DWrapper::init() {
    AINFO << "Creating Costmap";

    // 从 options_ 初始化变量（proto3 中所有字段都是可选的，未设置时返回默认值）
    if (!options_.frame_id().empty()) {
        global_frame_ = options_.frame_id();
    } else {
        global_frame_ = "map";
    }

    rolling_window_ = options_.rolling_window();
    track_unknown_space_ = false;

    if (options_.resolution() > 0.0) {
        resolution_ = options_.resolution();
    } else {
        resolution_ = 0.1;
    }

    if (options_.robot_radius() > 0.0) {
        robot_radius_ = options_.robot_radius();
    } else {
        robot_radius_ = 0.1;
    }

    if (options_.footprint_padding() > 0.0) {
        footprint_padding_ = options_.footprint_padding();
    } else {
        footprint_padding_ = 0.01;
    }

    // 初始化更新和发布频率
    if (options_.update_frequency() > 0.0) {
        map_update_frequency_ = options_.update_frequency();
        // 如果没有单独设置发布频率，使用更新频率作为发布频率
        if (map_publish_frequency_ <= 0.0) {
            map_publish_frequency_ = map_update_frequency_;
        }
    } else {
        // 默认值
        map_update_frequency_ = 5.0;
        if (map_publish_frequency_ <= 0.0) {
            map_publish_frequency_ = 1.0;
        }
    }

    // 初始化 always_send_full_costmap
    always_send_full_costmap_ = options_.always_send_full_costmap();

    // 初始化 footprint 相关变量
    use_radius_ = false;
    footprint_ = "";

    // 检查是否有 footprint 配置
    const auto& footprint_proto = options_.footprint();
    // 如果 footprint 有 points，转换为字符串格式
    if (footprint_proto.points_size() >= 3) {
        std::stringstream ss;
        ss << "[";
        for (int i = 0; i < footprint_proto.points_size(); ++i) {
            if (i > 0)
                ss << ", ";
            ss << "[" << footprint_proto.points(i).x() << ", "
               << footprint_proto.points(i).y() << "]";
        }
        ss << "]";
        footprint_ = ss.str();
        use_radius_ = false;
    } else {
        // footprint 点数不足或未设置，使用 robot_radius
        use_radius_ = true;
    }

    // Initialize map dimensions from options
    if (options_.width() > 0) {
        map_width_meters_ = options_.width();
    } else {
        map_width_meters_ = 5;  // default
    }

    if (options_.height() > 0) {
        map_height_meters_ = options_.height();
    } else {
        map_height_meters_ = 5;  // default
    }

    // Initialize origin (default to 0.0 if not specified)
    origin_x_ = 0.0;
    origin_y_ = 0.0;

    // Initialize robot_base_frame (default to "base_link" if not specified)
    robot_base_frame_ = "base_link";

    // Initialize transform tolerance (default to 0.3 if not specified)
    transform_tolerance_ = 0.3;

    // Initialize initial_transform_timeout (default to 60.0 if not specified)
    initial_transform_timeout_ = 60.0;

    // Initialize map_vis_z (default to 0.0)
    map_vis_z_ = 0.0;

    // Initialize plugins from options
    plugin_names_.clear();
    plugin_types_.clear();

    // Helper function to map plugin name to type
    auto GetPluginType = [](const std::string& plugin_name) -> std::string {
        if (plugin_name == "static_layer") {
            return "autonomy::map::costmap_2d::StaticLayer";
        } else if (plugin_name == "obstacle_layer") {
            return "autonomy::map::costmap_2d::ObstacleLayer";
        } else if (plugin_name == "inflation_layer") {
            return "autonomy::map::costmap_2d::InflationLayer";
        } else if (plugin_name == "voxel_layer") {
            return "autonomy::map::costmap_2d::VoxelLayer";
        } else if (plugin_name == "range_sensor_layer") {
            return "autonomy::map::costmap_2d::RangeSensorLayer";
        } else if (plugin_name == "denoise_layer") {
            return "autonomy::map::costmap_2d::DenoiseLayer";
        }
        // Default: assume the plugin name is the type
        return plugin_name;
    };

    // Parse plugins from options
    if (options_.plugins_size() > 0) {
        for (int i = 0; i < options_.plugins_size(); ++i) {
            std::string plugin_str = options_.plugins(i);
            // Special cases:
            // - "noop_layer"/"none": explicitly request no plugins (for
            // lightweight demos/tests).
            //   When user explicitly provides plugins list, we should not fall
            //   back to defaults.
            if (plugin_str.empty() || plugin_str == "noop_layer" ||
                plugin_str == "none") {
                continue;
            }
            // Check if it's in format "name:type" or just "name"
            size_t colon_pos = plugin_str.find(':');
            if (colon_pos != std::string::npos) {
                // Format: "name:type"
                plugin_names_.push_back(plugin_str.substr(0, colon_pos));
                plugin_types_.push_back(plugin_str.substr(colon_pos + 1));
            } else {
                // Format: just "name", map to type
                plugin_names_.push_back(plugin_str);
                plugin_types_.push_back(GetPluginType(plugin_str));
            }
        }
    } else {
        // Use default plugins if none specified
        plugin_names_ = default_plugins_;
        plugin_types_ = default_types_;
    }

    // Initialize filters (empty by default, no options field for filters yet)
    filter_names_.clear();
    filter_types_.clear();

    // Create the costmap itself
    layered_costmap_ = std::make_unique<LayeredCostmap>(
        global_frame_, rolling_window_, track_unknown_space_);

    if (!layered_costmap_->isSizeLocked()) {
        layered_costmap_->resizeMap(
            (unsigned int)(map_width_meters_ / resolution_),
            (unsigned int)(map_height_meters_ / resolution_), resolution_,
            origin_x_, origin_y_);
    }

    loadPlugins();

    if (!filter_names_.empty()) {
        AWARN << "Costmap filters configured (" << filter_names_.size()
              << ") but filter plugins are not loaded yet";
    }

    // Set the footprint
    if (use_radius_) {
        setRobotFootprint(makeFootprintFromRadius(robot_radius_));
    } else {
        std::vector<commsgs::geometry_msgs::Point> new_footprint;
        if (!footprint_.empty() &&
            makeFootprintFromString(footprint_, new_footprint)) {
            if (new_footprint.size() >= 3) {
                setRobotFootprint(new_footprint);
            } else {
                // footprint 解析后点数不足，使用 robot_radius
                AWARN << "Footprint has less than 3 points, using robot_radius "
                         "instead.";
                setRobotFootprint(makeFootprintFromRadius(robot_radius_));
            }
        } else {
            // footprint 解析失败，使用 robot_radius
            AWARN << "Failed to parse footprint, using robot_radius instead.";
            setRobotFootprint(makeFootprintFromRadius(robot_radius_));
        }
    }
}

void Costmap2DWrapper::loadPlugins() {
    if (!layered_costmap_) {
        return;
    }

    for (size_t i = 0; i < plugin_names_.size(); ++i) {
        const auto& plugin_name = plugin_names_[i];
        const auto& plugin_type = plugin_types_[i];
        std::shared_ptr<Layer> plugin =
            CreateCostmapLayerInstance(plugin_name, plugin_type);
        if (!plugin) {
            AWARN << "Unknown costmap layer plugin: " << plugin_name
                  << " type=" << plugin_type;
            continue;
        }

        plugin->initialize(layered_costmap_.get(), plugin_name, &options_);
        layered_costmap_->addPlugin(plugin);
        AINFO << "Loaded costmap layer: " << plugin_name
              << " (type=" << plugin_type << ")";
    }

    if (plugin_names_.empty()) {
        AINFO << "Costmap has no layer plugins (base grid only)";
    }
}

void Costmap2DWrapper::applyLoadedOccupancyGrid() {
    if (!layered_costmap_ || occupancy_grid_.info.width == 0 ||
        occupancy_grid_.info.height == 0) {
        return;
    }

    bool fed_to_static_layer = false;
    auto* plugins = layered_costmap_->getPlugins();
    if (plugins != nullptr) {
        for (auto& plugin : *plugins) {
            auto static_layer =
                std::dynamic_pointer_cast<StaticLayer>(plugin);
            if (static_layer == nullptr) {
                continue;
            }
            static_layer->loadOccupancyGrid(occupancy_grid_);
            fed_to_static_layer = true;
            AINFO << "Applied loaded map to StaticLayer";
            break;
        }
    }

    if (!fed_to_static_layer) {
        Costmap2D loaded_grid(occupancy_grid_);
        *layered_costmap_->getCostmap() = loaded_grid;
        AINFO << "Applied loaded map directly to master costmap";
    }

    if (plugins != nullptr && !plugins->empty()) {
        layered_costmap_->updateMap(0.0, 0.0, 0.0);
    }
    ready_ = true;
}

void Costmap2DWrapper::setPluginsActive(bool active) {
    if (!layered_costmap_) {
        return;
    }
    auto* plugins = layered_costmap_->getPlugins();
    if (plugins == nullptr) {
        return;
    }

    for (auto& plugin : *plugins) {
        if (active) {
            if (auto static_layer =
                    std::dynamic_pointer_cast<StaticLayer>(plugin)) {
                static_layer->activate();
            } else if (auto obstacle_layer =
                           std::dynamic_pointer_cast<ObstacleLayer>(plugin)) {
                obstacle_layer->activate();
            } else if (auto range_layer =
                           std::dynamic_pointer_cast<RangeSensorLayer>(
                               plugin)) {
                range_layer->activate();
            }
        } else {
            if (auto static_layer =
                    std::dynamic_pointer_cast<StaticLayer>(plugin)) {
                static_layer->deactivate();
            } else if (auto obstacle_layer =
                           std::dynamic_pointer_cast<ObstacleLayer>(plugin)) {
                obstacle_layer->deactivate();
            } else if (auto range_layer =
                           std::dynamic_pointer_cast<RangeSensorLayer>(
                               plugin)) {
                range_layer->deactivate();
            }
        }
    }
}

void Costmap2DWrapper::updateMap() {
    if (!layered_costmap_) {
        return;
    }

    double robot_x = 0.0;
    double robot_y = 0.0;
    double robot_yaw = 0.0;
    commsgs::geometry_msgs::PoseStamped robot_pose;
    if (getRobotPose(robot_pose)) {
        robot_x = robot_pose.pose.position.x;
        robot_y = robot_pose.pose.position.y;
        robot_yaw = autonomy::transform::tf2::getYaw(robot_pose.pose.orientation);
    }

    layered_costmap_->updateMap(robot_x, robot_y, robot_yaw);
    if (!ready_) {
        ready_ = true;
    }
}

void Costmap2DWrapper::Start() {
    // 检查是否启用
    if (!options_.enabled()) {
        AINFO << "Costmap2D is disabled, skipping Start().";
        return;
    }

    // 如果已经有更新线程在运行，则不重复启动
    if (map_update_thread_ && !map_update_thread_shutdown_) {
        AINFO << "Costmap2D update thread already running, Skip Start().";
        return;
    }

    // 如果配置了 map_file 且尚未加载地图，则加载
    // 注意：如果已经通过 MapEngine::Load() 加载过，这里不会重复加载
    if (!options_.map_file().empty() && !map_loaded_) {
        std::string filename = options_.map_file();

        // 如果路径是相对路径，使用地图数据目录拼接完整路径
        if (filename[0] != '/') {
            filename =
                ::autonomy::map::utils::GetMapDataFilesDirectory() + filename;
        }

        if (!loadMap(filename)) {
            AERROR << "Failed to load map from file: " << filename;
            // 即使加载失败也继续，可能后续会通过其他方式更新地图
        } else {
            AINFO << "Map loaded successfully from: " << filename;
        }
    }

    setPluginsActive(true);
    stopped_ = false;

    if (plugin_names_.empty() && !ready_) {
        ready_ = true;
        AINFO << "Costmap ready (no layer plugins)";
    }

    // 确定发布频率：优先使用 map_publish_frequency_，否则使用
    // update_frequency，最后使用默认值 1.0 Hz
    double publish_frequency =
        map_publish_frequency_ > 0.0
            ? map_publish_frequency_
            : (options_.update_frequency() > 0.0 ? options_.update_frequency()
                                                 : 1.0);

    int publish_interval_ms = static_cast<int>(1000.0 / publish_frequency);
    if (publish_interval_ms <= 0) {
        publish_interval_ms = 1000;  // 默认 1 秒
    }

    AINFO << "Starting Costmap2D with publish frequency: " << publish_frequency
          << " Hz";

    // 启动后台线程进行周期性发布，而不是阻塞当前线程
    stop_updates_ = false;
    map_update_thread_shutdown_ = false;

    map_update_thread_ =
        std::make_unique<std::thread>([this, publish_interval_ms]() {
            AINFO << "Costmap2D update thread started.";
            while (!stop_updates_) {
                try {
                    updateMap();
                    if (!ready_) {
                        AINFO << "Costmap2D is now ready after first update.";
                    }
                } catch (const std::exception& e) {
                    AERROR << "Failed to update costmap: " << e.what();
                }

                publishMap();
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(publish_interval_ms));
            }
            map_update_thread_shutdown_ = true;
            AINFO << "Costmap2D update thread stopped.";
        });
}

void Costmap2DWrapper::Stop() {
    stop_updates_ = true;

    // 等待更新线程退出
    if (map_update_thread_ && map_update_thread_->joinable()) {
        map_update_thread_->join();
        map_update_thread_.reset();
    }

    // layered_costmap_ is set only if on_configure has been called
    if (layered_costmap_) {
        std::vector<std::shared_ptr<Layer>>* plugins =
            layered_costmap_->getPlugins();
        std::vector<std::shared_ptr<Layer>>* filters =
            layered_costmap_->getFilters();

        setPluginsActive(false);
    }
    initialized_ = false;
    stopped_ = true;
}

void Costmap2DWrapper::Pause() {
    stop_updates_ = true;
    initialized_ = false;
}

void Costmap2DWrapper::Resume() {
    stop_updates_ = false;
    setPluginsActive(true);
    while (!ready_ && !stop_updates_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    initialized_ = ready_;
}

void Costmap2DWrapper::resetLayers() {
    Costmap2D* top = layered_costmap_->getCostmap();
    top->resetMap(0, 0, top->getSizeInCellsX(), top->getSizeInCellsY());

    // Reset each of the plugins
    std::vector<std::shared_ptr<Layer>>* plugins =
        layered_costmap_->getPlugins();
    std::vector<std::shared_ptr<Layer>>* filters =
        layered_costmap_->getFilters();
    for (auto plugin = plugins->begin(); plugin != plugins->end(); ++plugin) {
        (*plugin)->reset();
    }
    for (auto filter = filters->begin(); filter != filters->end(); ++filter) {
        (*filter)->reset();
    }
}

void Costmap2DWrapper::setRobotFootprint(
    const std::vector<commsgs::geometry_msgs::Point>& points) {
    unpadded_footprint_ = points;
    padded_footprint_ = points;
    padFootprint(padded_footprint_, footprint_padding_);
    layered_costmap_->setFootprint(padded_footprint_);
}

void Costmap2DWrapper::setRobotFootprintPolygon(
    const commsgs::geometry_msgs::Polygon::SharedPtr footprint) {
    setRobotFootprint(toPointVector(footprint));
}

void Costmap2DWrapper::getOrientedFootprint(
    std::vector<commsgs::geometry_msgs::Point>& oriented_footprint) {
    commsgs::geometry_msgs::PoseStamped global_pose;
    if (!getRobotPose(global_pose)) {
        oriented_footprint = padded_footprint_;
        return;
    }

    const double yaw =
        autonomy::transform::tf2::getYaw(global_pose.pose.orientation);
    transformFootprint(global_pose.pose.position.x, global_pose.pose.position.y,
                       yaw, padded_footprint_, oriented_footprint);
}

void Costmap2DWrapper::mapUpdateLoop(double frequency) {}

bool Costmap2DWrapper::getRobotPose(
    commsgs::geometry_msgs::PoseStamped& global_pose) {
    (void)global_pose;
    // TF lookup is handled by ControllerServer / planners via transform::Buffer.
    return false;
}

bool Costmap2DWrapper::transformPoseToGlobalFrame(
    const commsgs::geometry_msgs::PoseStamped& input_pose,
    commsgs::geometry_msgs::PoseStamped& transformed_pose) {
    if (input_pose.header.frame_id.empty() ||
        input_pose.header.frame_id == global_frame_) {
        transformed_pose = input_pose;
        transformed_pose.header.frame_id = global_frame_;
        return true;
    }

    auto* tf_buffer = autonomy::transform::Buffer::Instance();
    if (!tf_buffer) {
        AERROR << "TF buffer is null, cannot transform pose to global frame";
        return false;
    }

    try {
        transformed_pose = tf_buffer->transform(
            input_pose, global_frame_,
            static_cast<float>(transform_tolerance_));
        return true;
    } catch (const autonomy::transform::tf2::TransformException& ex) {
        AERROR << "Failed to transform pose from " << input_pose.header.frame_id
               << " to " << global_frame_ << ": " << ex.what();
    } catch (const std::exception& ex) {
        AERROR << "Failed to transform pose from " << input_pose.header.frame_id
               << " to " << global_frame_ << ": " << ex.what();
    }
    return false;
}

bool Costmap2DWrapper::loadMap(const std::string& filename) {
    if (loadMapFromYaml(filename, occupancy_grid_) !=
        LOAD_MAP_STATUS::LOAD_MAP_SUCCESS) {
        AERROR << "Load yaml file error.";
        return false;
    }

    // 根据加载的地图尺寸更新 layered_costmap_
    if (layered_costmap_ && occupancy_grid_.info.width > 0 &&
        occupancy_grid_.info.height > 0) {
        unsigned int size_x = occupancy_grid_.info.width;
        unsigned int size_y = occupancy_grid_.info.height;
        double resolution = occupancy_grid_.info.resolution;
        double origin_x = occupancy_grid_.info.origin.position.x;
        double origin_y = occupancy_grid_.info.origin.position.y;

        // 更新成员变量
        resolution_ = resolution;
        map_width_meters_ = size_x * resolution;
        map_height_meters_ = size_y * resolution;
        origin_x_ = origin_x;
        origin_y_ = origin_y;

        // 调整 layered_costmap 尺寸
        layered_costmap_->resizeMap(size_x, size_y, resolution, origin_x,
                                    origin_y);

        AINFO << "Map loaded and resized: " << size_x << "x" << size_y << " @ "
              << resolution << " m/cell, origin: (" << origin_x << ", "
              << origin_y << ")";
        applyLoadedOccupancyGrid();
    }

    map_loaded_ = true;
    return true;
}

bool Costmap2DWrapper::applyOccupancyGrid(
    const commsgs::map_msgs::OccupancyGrid& grid) {
    if (!utils::validateMsg(grid)) {
        AERROR << "Costmap2DWrapper: rejected malformed OccupancyGrid";
        return false;
    }

    occupancy_grid_ = grid;

    if (layered_costmap_ && occupancy_grid_.info.width > 0 &&
        occupancy_grid_.info.height > 0) {
        const unsigned int size_x = occupancy_grid_.info.width;
        const unsigned int size_y = occupancy_grid_.info.height;
        const double resolution = occupancy_grid_.info.resolution;
        const double origin_x = occupancy_grid_.info.origin.position.x;
        const double origin_y = occupancy_grid_.info.origin.position.y;

        resolution_ = resolution;
        map_width_meters_ = size_x * resolution;
        map_height_meters_ = size_y * resolution;
        origin_x_ = origin_x;
        origin_y_ = origin_y;

        layered_costmap_->resizeMap(size_x, size_y, resolution, origin_x,
                                    origin_y);
        applyLoadedOccupancyGrid();
    }

    map_loaded_ = true;
    ready_ = true;
    AINFO << "Costmap2DWrapper: applied external OccupancyGrid";
    return true;
}

void Costmap2DWrapper::publishMap() {
    // 检查是否有有效的 costmap
    if (!layered_costmap_ || !layered_costmap_->getCostmap()) {
        static int not_initialized_warn_count = 0;
        if (not_initialized_warn_count < 5 ||
            not_initialized_warn_count % 100 == 0) {
            AWARN << "Costmap not initialized, cannot publish map.";
        }
        ++not_initialized_warn_count;
        return;
    }

    Costmap2D* costmap = layered_costmap_->getCostmap();

    // 获取 costmap 的基本信息
    unsigned int size_x = costmap->getSizeInCellsX();
    unsigned int size_y = costmap->getSizeInCellsY();
    double resolution = costmap->getResolution();
    double origin_x = costmap->getOriginX();
    double origin_y = costmap->getOriginY();

    // 如果 costmap
    // 为空，不发布（只在前几次或每隔一段时间打印一次日志，避免刷屏）
    if (size_x == 0 || size_y == 0) {
        static int empty_warn_count = 0;
        if (empty_warn_count < 5 || empty_warn_count % 100 == 0) {
            AWARN << "Costmap is empty, cannot publish map.";
        }
        ++empty_warn_count;
        return;
    }

    // 更新 OccupancyGrid 的 header
    occupancy_grid_.header.frame_id = global_frame_;
    occupancy_grid_.header.stamp = commsgs::builtin_interfaces::Time::Now();

    // 更新地图元数据
    occupancy_grid_.info.width = size_x;
    occupancy_grid_.info.height = size_y;
    occupancy_grid_.info.resolution = resolution;
    occupancy_grid_.info.origin.position.x = origin_x;
    occupancy_grid_.info.origin.position.y = origin_y;
    occupancy_grid_.info.origin.position.z = 0.0;
    // 设置方向为 0（无旋转）
    occupancy_grid_.info.origin.orientation.x = 0.0;
    occupancy_grid_.info.origin.orientation.y = 0.0;
    occupancy_grid_.info.origin.orientation.z = 0.0;
    occupancy_grid_.info.origin.orientation.w = 1.0;

    // 获取 costmap 的原始数据
    const unsigned char* costmap_data = costmap->getCharMap();
    unsigned int total_size = size_x * size_y;

    // 调整 data 数组大小
    occupancy_grid_.data.clear();
    occupancy_grid_.data.reserve(total_size);

    // 将 costmap 值转换为 OccupancyGrid 值
    // Costmap: 0 (FREE_SPACE) -> 254 (LETHAL_OBSTACLE), 255 (NO_INFORMATION)
    // OccupancyGrid: -1 (UNKNOWN), 0 (FREE), 1-100 (OCCUPIED)
    for (unsigned int i = 0; i < total_size; ++i) {
        unsigned char cost = costmap_data[i];
        int8_t occ_value;

        if (cost == NO_INFORMATION) {
            // 未知区域
            occ_value = utils::OCC_GRID_UNKNOWN;
        } else if (cost == FREE_SPACE) {
            // 自由空间
            occ_value = utils::OCC_GRID_FREE;
        } else if (cost >= LETHAL_OBSTACLE) {
            // 致命障碍物
            occ_value = utils::OCC_GRID_OCCUPIED;
        } else {
            // 线性映射：从 [FREE_SPACE+1, LETHAL_OBSTACLE-1] 映射到 [1, 99]
            // 这样可以保留 costmap 中的中间值信息
            double normalized =
                static_cast<double>(cost - FREE_SPACE) /
                static_cast<double>(LETHAL_OBSTACLE - FREE_SPACE);
            occ_value = static_cast<int8_t>(
                std::round(normalized * (utils::OCC_GRID_OCCUPIED - 1) + 1));
        }

        occupancy_grid_.data.push_back(occ_value);
    }

    // 注意：这里只是更新了 occupancy_grid_ 成员变量
    // 实际的发布需要通过 autolink Writer 或 ROS 2 publisher 来实现
    // 如果需要在 autolink 中发布，可以通过 MapInterface 的接口
    // 只在需要时输出详细信息（避免频繁日志）
    static int publish_count = 0;
    if (++publish_count % 10 == 0) {
        AINFO << "Map updated: " << size_x << "x" << size_y
              << " resolution: " << resolution
              << " m/cell, frame: " << global_frame_;
    }
}

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy
