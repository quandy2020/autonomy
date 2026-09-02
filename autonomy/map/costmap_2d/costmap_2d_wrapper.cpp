#include <automsgs/msgs/sensor_msgs/range.pb.h>
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
#include <unordered_set>

#include "autolink/node/node.hpp"
#include "autolink/node/reader_base.hpp"
#include "autonomy/common/logging.hpp"

namespace {
bool occupancyGridContentEqual(
    const automsgs::msgs::map_msgs::OccupancyGrid& a,
    const automsgs::msgs::map_msgs::OccupancyGrid& b) {
    if (a.info().width() != b.info().width() || a.info().height() != b.info().height() ||
        a.info().resolution() != b.info().resolution()) {
        return false;
    }
    const auto& oa = a.info().origin().position();
    const auto& ob = b.info().origin().position();
    if (oa.x() != ob.x() || oa.y() != ob.y() || oa.z() != ob.z()) {
        return false;
    }
    return std::equal(a.data().begin(), a.data().end(), b.data().begin());
}
}  // namespace
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
        std::unordered_set<std::string> seen_plugins;
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
            std::string plugin_name =
                (colon_pos != std::string::npos)
                    ? plugin_str.substr(0, colon_pos)
                    : plugin_str;
            if (!seen_plugins.insert(plugin_name).second) {
                AWARN << "Duplicate costmap layer plugin ignored: "
                      << plugin_name;
                continue;
            }
            if (colon_pos != std::string::npos) {
                // Format: "name:type"
                plugin_names_.push_back(plugin_name);
                plugin_types_.push_back(plugin_str.substr(colon_pos + 1));
            } else {
                // Format: just "name", map to type
                plugin_names_.push_back(plugin_name);
                plugin_types_.push_back(GetPluginType(plugin_name));
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
        std::vector<automsgs::msgs::geometry_msgs::Point> new_footprint;
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
        AINFO << "Loaded layer plugin: " << plugin_name
              << " (type = " << plugin_type << ")";
    }

    if (plugin_names_.empty()) {
        AINFO << "Costmap has no layer plugins (base grid only)";
    }
}

void Costmap2DWrapper::applyLoadedOccupancyGrid(
    const automsgs::msgs::map_msgs::OccupancyGrid& grid) {
    if (!layered_costmap_ || grid.info().width() == 0 || grid.info().height() == 0) {
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
            static_layer->loadOccupancyGrid(grid);
            fed_to_static_layer = true;
            ADEBUG << "Applied loaded map to StaticLayer";
            break;
        }
    }

    if (!fed_to_static_layer) {
        // Rolling costmaps (no static_layer plugin) must not be overwritten by /map.
        ADEBUG << "No StaticLayer plugin; ignore external OccupancyGrid";
        return;
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
    if (global_frame_ == robot_base_frame_) {
        // Local rolling costmap in the robot frame: origin stays at (0,0,0).
    } else {
        // Do not fall back to (0,0): default costmap origin is (0,0) with size
        // [0,width]×[0,height], while the robot (and static map origin) often sit
        // in negative map coordinates — that falsely trips "out of bounds".
        automsgs::msgs::geometry_msgs::PoseStamped robot_pose;
        if (!getRobotPose(robot_pose)) {
            return;
        }
        robot_x = robot_pose.pose().position().x();
        robot_y = robot_pose.pose().position().y();
        robot_yaw =
            autonomy::transform::tf2::getYaw(robot_pose.pose().orientation());
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
    sensor_readers_.clear();
    sensor_readers_attached_ = false;

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
    const std::vector<automsgs::msgs::geometry_msgs::Point>& points) {
    unpadded_footprint_ = points;
    padded_footprint_ = points;
    padFootprint(padded_footprint_, footprint_padding_);
    layered_costmap_->setFootprint(padded_footprint_);
}

void Costmap2DWrapper::setRobotFootprintPolygon(
    const std::shared_ptr<automsgs::msgs::geometry_msgs::Polygon> footprint) {
    setRobotFootprint(toPointVector(footprint));
}

void Costmap2DWrapper::getOrientedFootprint(
    std::vector<automsgs::msgs::geometry_msgs::Point>& oriented_footprint) {
    automsgs::msgs::geometry_msgs::PoseStamped global_pose;
    if (!getRobotPose(global_pose)) {
        oriented_footprint = padded_footprint_;
        return;
    }

    const double yaw =
        autonomy::transform::tf2::getYaw(global_pose.pose().orientation());
    transformFootprint(global_pose.pose().position().x(), global_pose.pose().position().y(),
                       yaw, padded_footprint_, oriented_footprint);
}

void Costmap2DWrapper::mapUpdateLoop(double frequency) {}

void Costmap2DWrapper::setGlobalFrameID(const std::string& global_frame) {
    if (global_frame.empty() || global_frame == global_frame_) {
        return;
    }
    global_frame_ = global_frame;
    if (layered_costmap_) {
        layered_costmap_->setGlobalFrameID(global_frame);
    }
}

void Costmap2DWrapper::setRobotBaseFrameID(
    const std::string& robot_base_frame) {
    if (robot_base_frame.empty() || robot_base_frame == robot_base_frame_) {
        return;
    }
    robot_base_frame_ = robot_base_frame;
}

bool Costmap2DWrapper::getRobotPose(
    automsgs::msgs::geometry_msgs::PoseStamped& global_pose) {
    auto* tf_buffer = autonomy::transform::Buffer::Instance();
    if (!tf_buffer) {
        return false;
    }
    const float timeout =
        transform_tolerance_ > 0.0
            ? static_cast<float>(transform_tolerance_)
            : 0.1f;
    try {
        std::string err;
        if (!tf_buffer->canTransform(global_frame_, robot_base_frame_,
                                     automsgs::msgs::builtin_interfaces::ZeroTime(),
                                     timeout, &err)) {
            AERROR << "getRobotPose: canTransform(" << global_frame_ << ", "
                   << robot_base_frame_ << ") failed: " << err;
            return false;
        }
        const geometry_msgs::TransformStamped gt =
            static_cast<autonomy::transform::tf2::BufferCore&>(*tf_buffer)
                .lookupTransform(global_frame_, robot_base_frame_, 0ULL);
        global_pose.mutable_header()->set_frame_id(global_frame_);
        *global_pose.mutable_header()->mutable_stamp() = automsgs::msgs::builtin_interfaces::TimeNow();
        global_pose.mutable_pose()->mutable_position()->set_x(gt.transform.translation.x);
        global_pose.mutable_pose()->mutable_position()->set_y(gt.transform.translation.y);
        global_pose.mutable_pose()->mutable_position()->set_z(gt.transform.translation.z);
        global_pose.mutable_pose()->mutable_orientation()->set_x(gt.transform.rotation.x);
        global_pose.mutable_pose()->mutable_orientation()->set_y(gt.transform.rotation.y);
        global_pose.mutable_pose()->mutable_orientation()->set_z(gt.transform.rotation.z);
        global_pose.mutable_pose()->mutable_orientation()->set_w(gt.transform.rotation.w);
        return true;
    } catch (const autonomy::transform::tf2::TransformException& ex) {
        AERROR << "getRobotPose: " << ex.what();
    } catch (const std::exception& ex) {
        AERROR << "getRobotPose: " << ex.what();
    }
    return false;
}

bool Costmap2DWrapper::transformPoseToGlobalFrame(
    const automsgs::msgs::geometry_msgs::PoseStamped& input_pose,
    automsgs::msgs::geometry_msgs::PoseStamped& transformed_pose) {
    if (input_pose.header().frame_id().empty() ||
        input_pose.header().frame_id() == global_frame_) {
        transformed_pose = input_pose;
        transformed_pose.mutable_header()->set_frame_id(global_frame_);
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
        AERROR << "Failed to transform pose from " << input_pose.header().frame_id()
               << " to " << global_frame_ << ": " << ex.what();
    } catch (const std::exception& ex) {
        AERROR << "Failed to transform pose from " << input_pose.header().frame_id()
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
    if (layered_costmap_ && occupancy_grid_.info().width() > 0 &&
        occupancy_grid_.info().height() > 0) {
        unsigned int size_x = occupancy_grid_.info().width();
        unsigned int size_y = occupancy_grid_.info().height();
        double resolution = occupancy_grid_.info().resolution();
        double origin_x = occupancy_grid_.info().origin().position().x();
        double origin_y = occupancy_grid_.info().origin().position().y();

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
        applyLoadedOccupancyGrid(occupancy_grid_);
    }

    map_loaded_ = true;
    return true;
}

bool Costmap2DWrapper::applyOccupancyGrid(
    const automsgs::msgs::map_msgs::OccupancyGrid& grid) {
    if (!utils::validateMsg(grid)) {
        AERROR << "Costmap2DWrapper: rejected malformed OccupancyGrid";
        return false;
    }

    if (map_loaded_ && occupancyGridContentEqual(occupancy_grid_, grid)) {
        ADEBUG << "Costmap2DWrapper: static map unchanged, skip reapply";
        return true;
    }

    const automsgs::msgs::map_msgs::OccupancyGrid grid_copy = grid;
    occupancy_grid_ = grid_copy;

    if (layered_costmap_ && grid_copy.info().width() > 0 &&
        grid_copy.info().height() > 0) {
        // Do not resize/clear the master costmap here. A pre-clear races with
        // planners and briefly marks the world free → wall-crossing paths.
        // StaticLayer::processMap resizes under LayeredCostmap::updateMap's
        // lock, then copies lethal cells in the same update cycle.
        applyLoadedOccupancyGrid(grid_copy);

        if (Costmap2D* costmap = layered_costmap_->getCostmap()) {
            resolution_ = costmap->getResolution();
            map_width_meters_ = costmap->getSizeInMetersX();
            map_height_meters_ = costmap->getSizeInMetersY();
            origin_x_ = costmap->getOriginX();
            origin_y_ = costmap->getOriginY();
        }
    }

    map_loaded_ = true;
    ready_ = true;
    ADEBUG << "Costmap2DWrapper: applied external OccupancyGrid";
    return true;
}

void Costmap2DWrapper::SetMapPublishCallback(MapPublishCallback callback) {
    std::lock_guard<std::mutex> lock(map_publish_mutex_);
    map_publish_callback_ = std::move(callback);
}

void Costmap2DWrapper::publishMap() {
    // Build a costmap snapshot only. Do not mutate occupancy_grid_, which stores
    // the static map from MapServer (race with applyOccupancyGrid caused malformed
    // maps and heap corruption).
    automsgs::msgs::map_msgs::OccupancyGrid snapshot;
    if (!snapshotOccupancyGrid(snapshot)) {
        return;
    }
    MapPublishCallback callback;
    {
        std::lock_guard<std::mutex> lock(map_publish_mutex_);
        callback = map_publish_callback_;
    }
    if (callback) {
        callback(snapshot);
    }
}

void Costmap2DWrapper::AttachSensorReaders(
    const std::shared_ptr<autolink::Node>& node) {
    if (!node || sensor_readers_attached_ || !options_.has_obstacle_layer()) {
        return;
    }
    const auto& sources = options_.obstacle_layer().sensor_sources();
    if (sources.empty()) {
        return;
    }

    Costmap2DWrapper* self = this;
    for (const auto& entry : sources) {
        const auto& src = entry.second;
        const std::string topic =
            src.topic().empty() ? entry.first : src.topic();
        const std::string data_type =
            src.data_type().empty() ? "PointCloud2" : src.data_type();

        if (data_type == "LaserScan") {
            auto reader =
                node->CreateReader<automsgs::msgs::sensor_msgs::LaserScan>(
                    topic,
                    [self](const std::shared_ptr<
                           automsgs::msgs::sensor_msgs::LaserScan>& msg) {
                        if (msg) {
                            self->feedLaserScan(*msg);
                        }
                    });
            if (reader) {
                sensor_readers_.push_back(reader);
                AINFO << "Costmap2D: LaserScan on " << topic;
            } else {
                AWARN << "Costmap2D: failed to subscribe LaserScan "
                      << topic;
            }
        } else if (data_type == "PointCloud2") {
            auto reader =
                node->CreateReader<automsgs::msgs::sensor_msgs::PointCloud2>(
                    topic,
                    [self](const std::shared_ptr<
                           automsgs::msgs::sensor_msgs::PointCloud2>& msg) {
                        if (msg) {
                            self->feedPointCloud2(*msg);
                        }
                    });
            if (reader) {
                sensor_readers_.push_back(reader);
                AINFO << "Costmap2D: PointCloud2 on " << topic;
            } else {
                AWARN << "Costmap2D: failed to subscribe PointCloud2 "
                      << topic;
            }
        } else {
            AWARN << "Costmap2D: unsupported sensor data_type " << data_type
                  << " on " << topic;
        }
    }

    sensor_readers_attached_ = !sensor_readers_.empty();
}

void Costmap2DWrapper::feedLaserScan(
    const automsgs::msgs::sensor_msgs::LaserScan& scan) {
    if (!layered_costmap_) {
        return;
    }
    auto* plugins = layered_costmap_->getPlugins();
    if (plugins == nullptr) {
        return;
    }
    for (auto& plugin : *plugins) {
        if (auto obstacle_layer =
                std::dynamic_pointer_cast<ObstacleLayer>(plugin)) {
            obstacle_layer->feedLaserScan(scan);
        }
    }
}

void Costmap2DWrapper::feedPointCloud2(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud) {
    if (!layered_costmap_) {
        return;
    }
    auto* plugins = layered_costmap_->getPlugins();
    if (plugins == nullptr) {
        return;
    }
    for (auto& plugin : *plugins) {
        if (auto obstacle_layer =
                std::dynamic_pointer_cast<ObstacleLayer>(plugin)) {
            obstacle_layer->feedPointCloud2(cloud);
        }
    }
}

void Costmap2DWrapper::feedRange(const automsgs::msgs::sensor_msgs::Range& range) {
    if (!layered_costmap_) {
        return;
    }
    auto* plugins = layered_costmap_->getPlugins();
    if (plugins == nullptr) {
        return;
    }
    for (auto& plugin : *plugins) {
        if (auto range_layer =
                std::dynamic_pointer_cast<RangeSensorLayer>(plugin)) {
            range_layer->feedRange(range);
        }
    }
}

bool Costmap2DWrapper::snapshotOccupancyGrid(
    automsgs::msgs::map_msgs::OccupancyGrid& grid) {
    if (!layered_costmap_ || !layered_costmap_->getCostmap()) {
        return false;
    }

    Costmap2D* costmap = layered_costmap_->getCostmap();
    const unsigned int size_x = costmap->getSizeInCellsX();
    const unsigned int size_y = costmap->getSizeInCellsY();
    if (size_x == 0 || size_y == 0) {
        return false;
    }

    const double resolution = costmap->getResolution();
    grid.mutable_header()->set_frame_id( global_frame_);
    *grid.mutable_header()->mutable_stamp() = automsgs::msgs::builtin_interfaces::TimeNow();
    grid.mutable_info()->set_width(size_x);
    grid.mutable_info()->set_height(size_y);
    grid.mutable_info()->set_resolution(resolution);
    grid.mutable_info()->mutable_origin()->mutable_position()->set_x(costmap->getOriginX());
    grid.mutable_info()->mutable_origin()->mutable_position()->set_y(costmap->getOriginY());
    grid.mutable_info()->mutable_origin()->mutable_position()->set_z(0.0);
    grid.mutable_info()->mutable_origin()->mutable_orientation()->set_w(1.0);

    const unsigned char* costmap_data = costmap->getCharMap();
    const unsigned int total_size = size_x * size_y;
    grid.mutable_data()->Clear();
    grid.mutable_data()->Reserve(total_size);

    for (unsigned int i = 0; i < total_size; ++i) {
        const unsigned char cost = costmap_data[i];
        int8_t occ_value;
        if (cost == NO_INFORMATION) {
            occ_value = utils::OCC_GRID_UNKNOWN;
        } else if (cost == LETHAL_OBSTACLE) {
            occ_value = utils::OCC_GRID_OCCUPIED;
        } else if (cost == INSCRIBED_INFLATED_OBSTACLE) {
            // Keep Nav2-compatible reserved value for inscribed inflated
            // obstacles so RViz/Foxglove palettes look familiar.
            occ_value = static_cast<int8_t>(99);
        } else {
            // Nav2-compatible compression for 0..252 -> 0..98.
            occ_value = static_cast<int8_t>(
                std::lround(static_cast<double>(cost) * 98.0 / 252.0));
        }
        grid.mutable_data()->Add(occ_value);
    }
    return true;
}

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy
