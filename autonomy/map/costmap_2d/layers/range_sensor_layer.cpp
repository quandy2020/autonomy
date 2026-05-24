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

#include "autonomy/map/costmap_2d/layers/range_sensor_layer.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <string>

#include "autonomy/common/logging.hpp"
#include "autonomy/common/math/angle.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_math.hpp"
#include "autonomy/map/proto/map_2d_option.pb.h"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/tf2/exceptions.h"

namespace autonomy {
namespace map {
namespace costmap_2d {
namespace {

void TransformPoint(const commsgs::geometry_msgs::Transform& transform,
                    double x, double y, double z, double& out_x, double& out_y,
                    double& out_z) {
    const auto& q = transform.rotation;
    const auto& t = transform.translation;
    const double qx = q.x;
    const double qy = q.y;
    const double qz = q.z;
    const double qw = q.w;
    const double ix = qw * x + qy * z - qz * y;
    const double iy = qw * y + qz * x - qx * z;
    const double iz = qw * z + qx * y - qy * x;
    const double iw = -qx * x - qy * y - qz * z;
    out_x = ix * qw + iw * -qx + iy * -qz - iz * -qy + t.x;
    out_y = iy * qw + iw * -qy + iz * -qx - ix * -qz + t.y;
    out_z = iz * qw + iw * -qz + ix * -qy - iy * -qx + t.z;
}

}  // namespace

RangeSensorLayer::RangeSensorLayer() {
    processRangeMessageFunc_ =
        [this](commsgs::sensor_msgs::Range& msg) { processRangeMsg(msg); };
}

void RangeSensorLayer::onInitialize() {
    enabled_ = true;
    current_ = true;
    was_reset_ = false;
    buffered_readings_ = 0;
    phi_v_ = 1.2;
    inflate_cone_ = 1.0;
    no_readings_timeout_ = 0.0;
    clear_threshold_ = 0.2;
    mark_threshold_ = 0.8;
    clear_on_max_reading_ = false;
    transform_tolerance_ = DurationFromSeconds(0.3);
    last_reading_steady_ = std::chrono::steady_clock::now();
    default_value_ = to_cost(0.5);
    global_frame_ = layered_costmap_->getGlobalFrameID();

    const auto* layer_options = getOptions();
    if (layer_options && layer_options->has_range_sensor_layer()) {
        const auto& range_opts = layer_options->range_sensor_layer();
        enabled_ = range_opts.enabled();
        if (range_opts.phi() > 0.0) {
            phi_v_ = range_opts.phi();
        }
        if (range_opts.inflate_cone() > 0.0) {
            inflate_cone_ = range_opts.inflate_cone();
        }
        no_readings_timeout_ = range_opts.no_readings_timeout();
        clear_threshold_ = range_opts.clear_threshold();
        mark_threshold_ = range_opts.mark_threshold();
        clear_on_max_reading_ = range_opts.clear_on_max_reading();
        if (range_opts.transform_tolerance() > 0.0) {
            transform_tolerance_ =
                DurationFromSeconds(range_opts.transform_tolerance());
        }

        std::string sensor_type_name = range_opts.input_sensor_type();
        std::transform(sensor_type_name.begin(), sensor_type_name.end(),
                       sensor_type_name.begin(),
                       [](unsigned char c) {
                           return static_cast<char>(std::toupper(c));
                       });

        if (sensor_type_name == "VARIABLE") {
            processRangeMessageFunc_ = [this](commsgs::sensor_msgs::Range& msg) {
                processVariableRangeMsg(msg);
            };
        } else if (sensor_type_name == "FIXED") {
            processRangeMessageFunc_ = [this](commsgs::sensor_msgs::Range& msg) {
                processFixedRangeMsg(msg);
            };
        } else {
            processRangeMessageFunc_ = [this](commsgs::sensor_msgs::Range& msg) {
                processRangeMsg(msg);
            };
            if (!sensor_type_name.empty() && sensor_type_name != "ALL") {
                AWARN << "RangeSensorLayer: unknown input_sensor_type '"
                      << range_opts.input_sensor_type() << "', using ALL";
            }
        }

        if (range_opts.topics_size() > 0) {
            std::string topics;
            for (int i = 0; i < range_opts.topics_size(); ++i) {
                if (!topics.empty()) {
                    topics += ", ";
                }
                topics += range_opts.topics(i);
            }
            AINFO << "RangeSensorLayer topics configured: " << topics
                  << " (feed via bufferIncomingRangeMsg)";
        }
    } else if (layer_options && layer_options->has_static_layer()) {
        const double tol = layer_options->static_layer().transform_tolerance();
        if (tol > 0.0) {
            transform_tolerance_ = DurationFromSeconds(tol);
        }
    }

    matchSize();
    resetRange();

    AINFO << "RangeSensorLayer initialized: enabled=" << enabled_
          << " phi=" << phi_v_ << " inflate_cone=" << inflate_cone_
          << " clear_threshold=" << clear_threshold_
          << " mark_threshold=" << mark_threshold_;
}

double RangeSensorLayer::gamma(double theta) {
    if (fabs(theta) > max_angle_) {
        return 0.0;
    } else {
        return 1 - pow(theta / max_angle_, 2);
    }
}

double RangeSensorLayer::delta(double phi) {
    return 1 - (1 + tanh(2 * (phi - phi_v_))) / 2;
}

void RangeSensorLayer::get_deltas(double angle, double* dx, double* dy) {
    double ta = tan(angle);
    if (ta == 0) {
        *dx = 0;
    } else {
        *dx = resolution_ / ta;
    }

    *dx = copysign(*dx, cos(angle));
    *dy = copysign(resolution_, sin(angle));
}

double RangeSensorLayer::sensor_model(double r, double phi, double theta) {
    double lbda = delta(phi) * gamma(theta);
    double delta = resolution_;

    if (phi >= 0.0 && phi < r - 2 * delta * r) {
        return (1 - lbda) * (0.5);
    } else if (phi < r - delta * r) {
        return lbda * 0.5 * pow((phi - (r - 2 * delta * r)) / (delta * r), 2) +
               (1 - lbda) * .5;
    } else if (phi < r + delta * r) {
        double J = (r - phi) / (delta * r);
        return lbda * ((1 - (0.5) * pow(J, 2)) - 0.5) + 0.5;
    } else {
        return 0.5;
    }
}

void RangeSensorLayer::bufferIncomingRangeMsg(
    const commsgs::sensor_msgs::Range::SharedPtr range_message) {
    range_message_mutex_.lock();
    range_msgs_buffer_.push_back(*range_message);
    last_reading_steady_ = std::chrono::steady_clock::now();
    range_message_mutex_.unlock();
}

void RangeSensorLayer::feedRange(const commsgs::sensor_msgs::Range& range) {
    bufferIncomingRangeMsg(std::make_shared<commsgs::sensor_msgs::Range>(range));
}

void RangeSensorLayer::updateCostmap() {
    std::list<commsgs::sensor_msgs::Range> range_msgs_buffer_copy;

    range_message_mutex_.lock();
    range_msgs_buffer_copy =
        std::list<commsgs::sensor_msgs::Range>(range_msgs_buffer_);
    range_msgs_buffer_.clear();
    range_message_mutex_.unlock();

    for (auto& range_msgs_it : range_msgs_buffer_copy) {
        processRangeMessageFunc_(range_msgs_it);
    }
}

void RangeSensorLayer::processRangeMsg(
    commsgs::sensor_msgs::Range& range_message) {
    if (range_message.min_range == range_message.max_range) {
        processFixedRangeMsg(range_message);
    } else {
        processVariableRangeMsg(range_message);
    }
}

void RangeSensorLayer::processFixedRangeMsg(
    commsgs::sensor_msgs::Range& range_message) {
    if (!std::isinf(range_message.range)) {
        // RCLCPP_ERROR(
        //     logger_,
        //     "Fixed distance ranger (min_range == max_range) in frame %s sent
        //     invalid value. " "Only -Inf (== object detected) and Inf (== no
        //     object detected) are valid.",
        //     range_message.header.frame_id.c_str());
        return;
    }

    bool clear_sensor_cone = false;
    if (range_message.range > 0) {  // +inf
        if (!clear_on_max_reading_) {
            return;  // no clearing at all
        }
        clear_sensor_cone = true;
    }

    range_message.range = range_message.min_range;

    updateCostmap(range_message, clear_sensor_cone);
}

void RangeSensorLayer::processVariableRangeMsg(
    commsgs::sensor_msgs::Range& range_message) {
    if (range_message.range < range_message.min_range ||
        range_message.range > range_message.max_range) {
        return;
    }

    bool clear_sensor_cone = false;

    if (range_message.range >= range_message.max_range &&
        clear_on_max_reading_) {
        clear_sensor_cone = true;
    }

    updateCostmap(range_message, clear_sensor_cone);
}

void RangeSensorLayer::updateCostmap(commsgs::sensor_msgs::Range& range_message,
                                     bool clear_sensor_cone) {
    max_angle_ = range_message.field_of_view / 2;

    auto* tf_buffer = autonomy::transform::Buffer::Instance();
    if (!tf_buffer) {
        AWARN << "RangeSensorLayer: TF buffer unavailable";
        return;
    }

    const float timeout = static_cast<float>(
        SecondsFromDuration(transform_tolerance_));
    double ox = 0.0;
    double oy = 0.0;
    double tx = 0.0;
    double ty = 0.0;

    try {
        const auto origin_transform = tf_buffer->lookupTransform(
            global_frame_, range_message.header.frame_id,
            range_message.header.stamp, timeout);
        double oz = 0.0;
        double tz = 0.0;
        TransformPoint(origin_transform.transform, 0.0, 0.0, 0.0, ox, oy, oz);
        TransformPoint(origin_transform.transform, range_message.range, 0.0, 0.0,
                       tx, ty, tz);
        (void)oz;
        (void)tz;
    } catch (const transform::tf2::TransformException& ex) {
        AWARN << "RangeSensorLayer TF error (" << range_message.header.frame_id
              << " -> " << global_frame_ << "): " << ex.what();
        return;
    }

    // calculate target props
    double dx = tx - ox, dy = ty - oy, theta = atan2(dy, dx),
           d = sqrt(dx * dx + dy * dy);

    // Integer Bounds of Update
    int bx0, by0, bx1, by1;

    // Triangle that will be really updated; the other cells within bounds are
    // ignored This triangle is formed by the origin and left and right sides of
    // sonar cone
    int Ox, Oy, Ax, Ay, Bx, By;

    // Bounds includes the origin
    worldToMapNoBounds(ox, oy, Ox, Oy);
    bx1 = bx0 = Ox;
    by1 = by0 = Oy;
    touch(ox, oy, &min_x_, &min_y_, &max_x_, &max_y_);

    // Update Map with Target Point
    unsigned int aa, ab;
    if (worldToMap(tx, ty, aa, ab)) {
        setCost(aa, ab, 233);
        touch(tx, ty, &min_x_, &min_y_, &max_x_, &max_y_);
    }

    double mx, my;

    // Update left side of sonar cone
    mx = ox + cos(theta - max_angle_) * d * 1.2;
    my = oy + sin(theta - max_angle_) * d * 1.2;
    worldToMapNoBounds(mx, my, Ax, Ay);
    bx0 = std::min(bx0, Ax);
    bx1 = std::max(bx1, Ax);
    by0 = std::min(by0, Ay);
    by1 = std::max(by1, Ay);
    touch(mx, my, &min_x_, &min_y_, &max_x_, &max_y_);

    // Update right side of sonar cone
    mx = ox + cos(theta + max_angle_) * d * 1.2;
    my = oy + sin(theta + max_angle_) * d * 1.2;

    worldToMapNoBounds(mx, my, Bx, By);
    bx0 = std::min(bx0, Bx);
    bx1 = std::max(bx1, Bx);
    by0 = std::min(by0, By);
    by1 = std::max(by1, By);
    touch(mx, my, &min_x_, &min_y_, &max_x_, &max_y_);

    // Limit Bounds to Grid
    bx0 = std::max(0, bx0);
    by0 = std::max(0, by0);
    bx1 = std::min(static_cast<int>(size_x_), bx1);
    by1 = std::min(static_cast<int>(size_y_), by1);

    for (unsigned int x = bx0; x <= (unsigned int)bx1; x++) {
        for (unsigned int y = by0; y <= (unsigned int)by1; y++) {
            bool update_xy_cell = true;

            // Unless inflate_cone_ is set to 100 %, we update cells only within
            // the (partially inflated) sensor cone, projected on the costmap as
            // a triangle. 0 % corresponds to just the triangle, but if your
            // sensor fov is very narrow, the covered area can become zero due
            // to cell discretization. See wiki description for more details
            if (inflate_cone_ < 1.0) {
                // Determine barycentric coordinates
                int w0 = orient2d(Ax, Ay, Bx, By, x, y);
                int w1 = orient2d(Bx, By, Ox, Oy, x, y);
                int w2 = orient2d(Ox, Oy, Ax, Ay, x, y);

                // Barycentric coordinates inside area threshold; this is not
                // mathematically sound at all, but it works!
                float bcciath = -static_cast<float>(inflate_cone_) *
                                area(Ax, Ay, Bx, By, Ox, Oy);
                update_xy_cell =
                    w0 >= bcciath && w1 >= bcciath && w2 >= bcciath;
            }

            if (update_xy_cell) {
                double wx, wy;
                mapToWorld(x, y, wx, wy);
                update_cell(ox, oy, theta, range_message.range, wx, wy,
                            clear_sensor_cone);
            }
        }
    }

    buffered_readings_++;
    // last_reading_time_ = clock_->now();
}

void RangeSensorLayer::update_cell(double ox, double oy, double ot, double r,
                                   double nx, double ny, bool clear) {
    unsigned int x, y;
    if (worldToMap(nx, ny, x, y)) {
        double dx = nx - ox, dy = ny - oy;
        double theta = atan2(dy, dx) - ot;
        theta = common::math::NormalizeAngleDifference(theta);
        double phi = sqrt(dx * dx + dy * dy);
        double sensor = 0.0;
        if (!clear) {
            sensor = sensor_model(r, phi, theta);
        }
        double prior = to_prob(getCost(x, y));
        double prob_occ = sensor * prior;
        double prob_not = (1 - sensor) * (1 - prior);
        double new_prob = prob_occ / (prob_occ + prob_not);

        // RCLCPP_DEBUG(
        //   logger_,
        //   "%f %f | %f %f = %f", dx, dy, theta, phi, sensor);
        // RCLCPP_DEBUG(
        //   logger_,
        //   "%f | %f %f | %f", prior, prob_occ, prob_not, new_prob);
        unsigned char c = to_cost(new_prob);
        setCost(x, y, c);
    }
}

void RangeSensorLayer::resetRange() {
    min_x_ = min_y_ = std::numeric_limits<double>::max();
    max_x_ = max_y_ = -std::numeric_limits<double>::max();
}

void RangeSensorLayer::updateBounds(double robot_x, double robot_y,
                                    double robot_yaw, double* min_x,
                                    double* min_y, double* max_x,
                                    double* max_y) {
    robot_yaw = 0 + robot_yaw;  // Avoid error if variable not in use
    if (layered_costmap_->isRolling()) {
        updateOrigin(robot_x - getSizeInMetersX() / 2,
                     robot_y - getSizeInMetersY() / 2);
    }

    updateCostmap();

    *min_x = std::min(*min_x, min_x_);
    *min_y = std::min(*min_y, min_y_);
    *max_x = std::max(*max_x, max_x_);
    *max_y = std::max(*max_y, max_y_);

    resetRange();

    if (!enabled_) {
        current_ = true;
        return;
    }

    if (buffered_readings_ == 0 && no_readings_timeout_ > 0.0) {
        const auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_reading_steady_);
        if (elapsed.count() > no_readings_timeout_) {
            AWARN << "RangeSensorLayer: no readings for " << elapsed.count()
                  << "s (timeout " << no_readings_timeout_ << "s)";
            current_ = false;
        }
    }
}

void RangeSensorLayer::updateCosts(Costmap2D& master_grid, int min_i, int min_j,
                                   int max_i, int max_j) {
    if (!enabled_) {
        return;
    }

    unsigned char* master_array = master_grid.getCharMap();
    unsigned int span = master_grid.getSizeInCellsX();
    unsigned char clear = to_cost(clear_threshold_),
                  mark = to_cost(mark_threshold_);

    for (int j = min_j; j < max_j; j++) {
        unsigned int it = j * span + min_i;
        for (int i = min_i; i < max_i; i++) {
            unsigned char prob = costmap_[it];
            unsigned char current;
            if (prob == NO_INFORMATION) {
                it++;
                continue;
            } else if (prob > mark) {
                current = LETHAL_OBSTACLE;
            } else if (prob < clear) {
                current = FREE_SPACE;
            } else {
                it++;
                continue;
            }

            unsigned char old_cost = master_array[it];

            if (old_cost == NO_INFORMATION || old_cost < current) {
                master_array[it] = current;
            }
            it++;
        }
    }

    buffered_readings_ = 0;

    // if not current due to reset, set current now after clearing
    if (!current_ && was_reset_) {
        was_reset_ = false;
        current_ = true;
    }
}

void RangeSensorLayer::reset() {
    // RCLCPP_DEBUG(logger_, "Reseting range sensor layer...");
    deactivate();
    resetMaps();
    was_reset_ = true;
    activate();
}

void RangeSensorLayer::deactivate() {
    range_msgs_buffer_.clear();
}

void RangeSensorLayer::activate() {
    range_msgs_buffer_.clear();
}

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy