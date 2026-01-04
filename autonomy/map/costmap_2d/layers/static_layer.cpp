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

#include "autonomy/map/costmap_2d/layers/static_layer.hpp"

#include <algorithm>
#include <string>

#include "autolink/autolink.hpp"
#include "autolink/class_loader/class_loader_register_macro.hpp"
#include "autolink/common/log.hpp"
#include "autonomy/map/costmap_2d/utils/validate_messages.hpp"
#include "autonomy/map/proto/map_2d_option.pb.h"
#include "autonomy/transform/tf2/convert.h"

namespace autonomy {
namespace map {
namespace costmap_2d {

StaticLayer::StaticLayer() : map_buffer_(nullptr) {}

StaticLayer::~StaticLayer() {}

void StaticLayer::onInitialize() {
    global_frame_ = layered_costmap_->getGlobalFrameID();

    getParameters();

    // 检查 node_ 是否有效
    if (!node_) {
        AWARN << "StaticLayer: node_ is null, skipping topic subscription. "
              << "Map updates will not be received from topic: " << map_topic_;
        return;
    }

    map_sub_ = node_->CreateReader<commsgs::map_msgs::OccupancyGrid>(
        map_topic_,
        std::bind(&StaticLayer::incomingMap, this, std::placeholders::_1));

    if (map_sub_) {
        AINFO << "StaticLayer: Subscribed to map topic: " << map_topic_;
    } else {
        AERROR << "StaticLayer: Failed to subscribe to map topic: "
               << map_topic_;
    }

    if (subscribe_to_updates_) {
        AINFO << "Subscribing to updates";
        map_update_sub_ =
            node_->CreateReader<commsgs::map_msgs::OccupancyGridUpdate>(
                map_topic_ + "_updates",
                std::bind(&StaticLayer::incomingUpdate, this,
                          std::placeholders::_1));
    }
}

void StaticLayer::activate() {}

void StaticLayer::deactivate() {}

void StaticLayer::reset() {
    has_updated_data_ = true;
    current_ = false;
}

void StaticLayer::getParameters() {
    // Get options from parent layer
    const proto::Costmap2DOptions* options = getOptions();
    if (!options) {
        AERROR << "StaticLayer: options not available, using defaults";
        // Set defaults
        enabled_ = true;
        subscribe_to_updates_ = false;
        footprint_clearing_enabled_ = false;
        map_topic_ = "map";
        transform_tolerance_ = 0;
        map_subscribe_transient_local_ = true;
        track_unknown_space_ = false;
        use_maximum_ = false;
        lethal_threshold_ = 100;
        unknown_cost_value_ = 0xff;
        trinary_costmap_ = true;
        map_received_ = false;
        map_received_in_update_bounds_ = false;
        return;
    }

    // Get static_layer configuration from options
    const auto& static_layer_config = options->static_layer();

    // Read parameters from static_layer_config
    enabled_ = static_layer_config.enabled();
    subscribe_to_updates_ = static_layer_config.subscribe_to_updates();
    footprint_clearing_enabled_ =
        static_layer_config.footprint_clearing_enabled();

    // Map topic: use static_layer.map_topic if set, otherwise default to "map"
    if (!static_layer_config.map_topic().empty()) {
        map_topic_ = static_layer_config.map_topic();
    } else {
        map_topic_ = "map";  // default
    }

    // Transform tolerance
    double temp_tf_tol = static_layer_config.transform_tolerance();
    if (temp_tf_tol <= 0.0) {
        temp_tf_tol = 0.0;  // default
    }
    transform_tolerance_ = static_cast<transform::tf2::Duration>(
        static_cast<uint64_t>(temp_tf_tol * 1e9));  // convert to nanoseconds

    // Map subscribe transient local (default to true)
    map_subscribe_transient_local_ = true;

    // Other parameters from costmap options (if available)
    track_unknown_space_ = false;  // default
    use_maximum_ = false;          // default
    lethal_threshold_ = 100;       // default
    unknown_cost_value_ = 0xff;    // default
    trinary_costmap_ = true;       // default

    map_received_ = false;
    map_received_in_update_bounds_ = false;
}

void StaticLayer::processMap(const commsgs::map_msgs::OccupancyGrid& new_map) {
    AINFO << "StaticLayer: Process map";

    unsigned int size_x = new_map.info.width;
    unsigned int size_y = new_map.info.height;

    ADEBUG << "StaticLayer: Received a " << size_x << " X " << size_y
           << " map at " << new_map.info.resolution << " m/pix";

    // resize costmap if size, resolution or origin do not match
    Costmap2D* master = layered_costmap_->getCostmap();
    if (!layered_costmap_->isRolling() &&
        (master->getSizeInCellsX() != size_x ||
         master->getSizeInCellsY() != size_y ||
         master->getResolution() != new_map.info.resolution ||
         master->getOriginX() != new_map.info.origin.position.x ||
         master->getOriginY() != new_map.info.origin.position.y ||
         !layered_costmap_->isSizeLocked())) {
        // Update the size of the layered costmap (and all layers, including
        // this one)
        AINFO << "StaticLayer: Resizing costmap to " << size_x << " X "
              << size_y << " at " << new_map.info.resolution << " m/pix";
        layered_costmap_->resizeMap(size_x, size_y, new_map.info.resolution,
                                    new_map.info.origin.position.x,
                                    new_map.info.origin.position.y, true);
    } else if (size_x_ != size_x || size_y_ != size_y ||  // NOLINT
               resolution_ != new_map.info.resolution ||
               origin_x_ != new_map.info.origin.position.x ||
               origin_y_ != new_map.info.origin.position.y) {
        // only update the size of the costmap stored locally in this layer
        AINFO << "StaticLayer: Resizing static layer to " << size_x << " X "
              << size_y << " at " << new_map.info.resolution << " m/pix";
        resizeMap(size_x, size_y, new_map.info.resolution,
                  new_map.info.origin.position.x,
                  new_map.info.origin.position.y);
    }

    unsigned int index = 0;

    // we have a new map, update full size of map
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

    // initialize the costmap with static data
    // 统计数据分布用于调试
    int count_free = 0, count_occupied = 0, count_unknown = 0, count_other = 0;
    for (unsigned int i = 0; i < size_y; ++i) {
        for (unsigned int j = 0; j < size_x; ++j) {
            // new_map.data 是 int16_t，需要正确转换
            int16_t raw_value = new_map.data[index];
            unsigned char value;
            if (raw_value < 0) {
                value = 255;  // unknown -> 0xff
            } else if (raw_value > 255) {
                value = 255;
            } else {
                value = static_cast<unsigned char>(raw_value);
            }

            unsigned char cost = interpretValue(value);
            costmap_[index] = cost;

            // 统计
            if (cost == FREE_SPACE)
                count_free++;
            else if (cost == LETHAL_OBSTACLE)
                count_occupied++;
            else if (cost == NO_INFORMATION)
                count_unknown++;
            else
                count_other++;

            ++index;
        }
    }

    ADEBUG << "StaticLayer: processMap data distribution - "
           << "free: " << count_free << ", occupied: " << count_occupied
           << ", unknown: " << count_unknown << ", other: " << count_other
           << " (total: " << (size_x * size_y) << ")";

    map_frame_ = new_map.header.frame_id;

    x_ = y_ = 0;
    width_ = size_x_;
    height_ = size_y_;
    has_updated_data_ = true;

    current_ = true;
}

void StaticLayer::matchSize() {
    // If we are using rolling costmap, the static map size is
    //   unrelated to the size of the layered costmap
    if (!layered_costmap_->isRolling()) {
        Costmap2D* master = layered_costmap_->getCostmap();
        resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(),
                  master->getResolution(), master->getOriginX(),
                  master->getOriginY());
    }
}

unsigned char StaticLayer::interpretValue(unsigned char value) {
    // check if the static value is above the unknown or lethal thresholds
    if (track_unknown_space_ && value == unknown_cost_value_) {
        return NO_INFORMATION;
    } else if (!track_unknown_space_ && value == unknown_cost_value_) {
        return FREE_SPACE;
    } else if (value >= lethal_threshold_) {
        return LETHAL_OBSTACLE;
    } else if (trinary_costmap_) {
        return FREE_SPACE;
    }

    double scale = static_cast<double>(value) / lethal_threshold_;
    return scale * LETHAL_OBSTACLE;
}

void StaticLayer::incomingMap(
    const commsgs::map_msgs::OccupancyGrid::SharedPtr new_map) {
    AINFO << "StaticLayer: Received map (" << new_map->info.width << "x"
          << new_map->info.height << " @ " << new_map->info.resolution
          << " m/cell, data size: " << new_map->data.size() << ")";

    if (!utils::validateMsg(*new_map)) {
        AWARN << "Received map message is malformed. Rejecting. "
              << "width=" << new_map->info.width
              << ", height=" << new_map->info.height
              << ", resolution=" << new_map->info.resolution
              << ", data.size=" << new_map->data.size();
        return;
    }

    if (!map_received_) {
        AINFO << "StaticLayer: First map, calling processMap...";
        processMap(*new_map);
        map_received_ = true;
        return;
    }
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    map_buffer_ = new_map;
}

void StaticLayer::incomingUpdate(
    commsgs::map_msgs::OccupancyGridUpdate::ConstSharedPtr update) {
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    if (update->y < static_cast<int32_t>(y_) ||
        y_ + height_ < update->y + update->height ||
        update->x < static_cast<int32_t>(x_) ||
        x_ + width_ < update->x + update->width) {
        AWARN << "StaticLayer: Map update ignored. Exceeds bounds of static "
                 "layer.\n"
              << "Static layer origin: " << x_ << ", " << y_ << "   bounds: "
              << "Update origin: " << update->x << ", " << update->y
              << "   bounds: " << update->width << " X " << update->height;
        return;
    }

    if (update->header.frame_id != map_frame_) {
        AWARN << "StaticLayer: Map update ignored. Current map is in frame "
              << map_frame_ << " but update was in frame "
              << update->header.frame_id;
        return;
    }

    unsigned int di = 0;
    for (unsigned int y = 0; y < update->height; y++) {
        unsigned int index_base = (update->y + y) * size_x_;
        for (unsigned int x = 0; x < update->width; x++) {
            unsigned int index = index_base + x + update->x;
            costmap_[index] = interpretValue(update->data[di++]);
        }
    }

    has_updated_data_ = true;
}

void StaticLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                               double* min_x, double* min_y, double* max_x,
                               double* max_y) {
    if (!map_received_) {
        map_received_in_update_bounds_ = false;
        return;
    }
    map_received_in_update_bounds_ = true;

    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

    // If there is a new available map, load it.
    if (map_buffer_) {
        processMap(*map_buffer_);
        map_buffer_ = nullptr;
    }

    if (!layered_costmap_->isRolling()) {
        if (!(has_updated_data_ || has_extra_bounds_)) {
            return;
        }
    }

    useExtraBounds(min_x, min_y, max_x, max_y);

    double wx, wy;

    mapToWorld(x_, y_, wx, wy);
    *min_x = std::min(wx, *min_x);
    *min_y = std::min(wy, *min_y);

    mapToWorld(x_ + width_, y_ + height_, wx, wy);
    *max_x = std::max(wx, *max_x);
    *max_y = std::max(wy, *max_y);

    has_updated_data_ = false;

    updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void StaticLayer::updateFootprint(double robot_x, double robot_y,
                                  double robot_yaw, double* min_x,
                                  double* min_y, double* max_x, double* max_y) {
    if (!footprint_clearing_enabled_) {
        return;
    }

    transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(),
                       transformed_footprint_);

    for (unsigned int i = 0; i < transformed_footprint_.size(); i++) {
        touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x,
              min_y, max_x, max_y);
    }
}

void StaticLayer::updateCosts(Costmap2D& master_grid, int min_i, int min_j,
                              int max_i, int max_j) {
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    if (!enabled_) {
        return;
    }
    if (!map_received_in_update_bounds_) {
        static int count = 0;
        // throttle warning down to only 1/10 message rate
        if (++count == 10) {
            AWARN << "Can't update static costmap layer, no map received "
                     "(map_received_="
                  << map_received_
                  << ", in_update_bounds=" << map_received_in_update_bounds_
                  << ")";
            count = 0;
        }
        return;
    }

    // 仅在第一次更新时打印日志
    static bool first_update = true;
    if (first_update) {
        AINFO
            << "StaticLayer::updateCosts: copying data to master grid. size_x="
            << size_x_ << " size_y=" << size_y_ << " range=[" << min_i << ","
            << min_j << "]->[" << max_i << "," << max_j << "]";
        first_update = false;
    }

    if (footprint_clearing_enabled_) {
        setConvexPolygonCost(transformed_footprint_, FREE_SPACE);
    }

    if (!layered_costmap_->isRolling()) {
        // if not rolling, the layered costmap (master_grid) has same
        // coordinates as this layer
        if (!use_maximum_) {
            updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
        } else {
            updateWithMax(master_grid, min_i, min_j, max_i, max_j);
        }
    } else {
        // If rolling window, the master_grid is unlikely to have same
        // coordinates as this layer
        unsigned int mx, my;
        double wx, wy;

        // 注意：当前系统没有实现 TF 变换接口
        // 如果 map_frame_ 和 global_frame_ 相同，则直接使用坐标
        // 否则使用恒等变换（假设两个坐标系对齐）
        if (map_frame_ != global_frame_) {
            // 仅在首次警告时输出日志
            static bool warned = false;
            if (!warned) {
                AWARN
                    << "StaticLayer: Rolling window mode with different frames "
                    << "(map_frame: " << map_frame_
                    << ", global_frame: " << global_frame_ << "). "
                    << "Using identity transform (TF not implemented).";
                warned = true;
            }
        }

        // 直接使用坐标复制（假设坐标系对齐或使用恒等变换）
        for (int i = min_i; i < max_i; ++i) {
            for (int j = min_j; j < max_j; ++j) {
                // Convert master_grid coordinates (i,j) into world coordinates
                layered_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);
                // Set master_grid with cell from static map
                if (worldToMap(wx, wy, mx, my)) {
                    if (!use_maximum_) {
                        master_grid.setCost(i, j, getCost(mx, my));
                    } else {
                        master_grid.setCost(
                            i, j,
                            std::max(getCost(mx, my),
                                     master_grid.getCost(i, j)));
                    }
                }
            }
        }
    }
    current_ = true;
}

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy

// Register the class as a plugin for dynamic library loading
CLASS_LOADER_REGISTER_CLASS(autonomy::map::costmap_2d::StaticLayer,
                            autonomy::map::costmap_2d::Layer)
