/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autonomy/visualization/converter/vision_msgs_converter.hpp"

#include <cmath>
#include <optional>
#include <string>

#include "autolink/autolink.hpp"
#include "autonomy/visualization/converter/converter_detail.hpp"

namespace autonomy {
namespace visualization {
namespace converter {
namespace impl {

using detail::CreateColor;
using detail::CreatePose;
using detail::ExtractFrameId;
using detail::ExtractTimestamp;
using detail::SetEntityHeader;

// Helper function to convert Pose2D to geometry_msgs::Pose for 2D bounding
// boxes
autonomy::commsgs::proto::geometry_msgs::Pose Pose2DToPose3D(
    const autonomy::commsgs::proto::vision_msgs::Pose2D& pose2d) {
    autonomy::commsgs::proto::geometry_msgs::Pose pose;
    if (pose2d.has_position()) {
        pose.mutable_position()->set_x(pose2d.position().x());
        pose.mutable_position()->set_y(pose2d.position().y());
        pose.mutable_position()->set_z(0.0);
    } else {
        pose.mutable_position()->set_x(0.0);
        pose.mutable_position()->set_y(0.0);
        pose.mutable_position()->set_z(0.0);
    }

    // Convert theta (rotation around Z axis) to quaternion
    double half_theta = pose2d.theta() * 0.5;
    pose.mutable_orientation()->set_x(0.0);
    pose.mutable_orientation()->set_y(0.0);
    pose.mutable_orientation()->set_z(std::sin(half_theta));
    pose.mutable_orientation()->set_w(std::cos(half_theta));

    return pose;
}

// BoundingBox2D conversion
foxglove::schemas::SceneUpdate ToFoxgloveImpl(const autonomy::commsgs::proto::vision_msgs::BoundingBox2D& message) {
    foxglove::schemas::SceneUpdate scene_update;

    // Create scene entity for 2D bounding box
    // Note: 2D bounding boxes are typically visualized in image space, but
    // we'll project them to 3D at z=0 for visualization
    foxglove::schemas::SceneEntity entity;
    SetEntityHeader(entity, std::nullopt,
                    std::nullopt);  // BoundingBox2D has no header

    entity.id = "bbox2d";
    entity.frame_locked = false;

    // Convert 2D pose to 3D pose
    if (message.has_center()) {
        autonomy::commsgs::proto::geometry_msgs::Pose pose3d = Pose2DToPose3D(message.center());

        // Create a cube to represent the 2D bounding box (projected to 3D)
        foxglove::schemas::CubePrimitive cube;
        cube.pose = CreatePose(pose3d);

        cube.size = foxglove::schemas::Vector3();
        cube.size->x = message.size_x();
        cube.size->y = message.size_y();
        cube.size->z = 0.01;  // Small thickness for 2D visualization

        // Default color for 2D bounding boxes (cyan)
        cube.color = CreateColor(0.0, 1.0, 1.0, 0.5);

        entity.cubes.push_back(cube);
    }

    if (!entity.cubes.empty()) {
        scene_update.entities.push_back(entity);
    }

    AINFO << "Converted BoundingBox2D to SceneUpdate";

    return scene_update;
}

// BoundingBox2DArray conversion
foxglove::schemas::SceneUpdate ToFoxgloveImpl(
    const autonomy::commsgs::proto::vision_msgs::BoundingBox2DArray& message) {
    foxglove::schemas::SceneUpdate scene_update;

    if (message.boxes_size() == 0) {
        AWARN << "BoundingBox2DArray is empty, returning empty SceneUpdate";
        return scene_update;
    }

    foxglove::schemas::SceneEntity entity;
    SetEntityHeader(entity, ExtractTimestamp(message), ExtractFrameId(message));

    entity.id = "bbox2d_array";
    entity.frame_locked = false;

    // Convert each 2D bounding box
    for (int i = 0; i < message.boxes_size(); ++i) {
        const auto& bbox = message.boxes(i);
        if (bbox.has_center()) {
            autonomy::commsgs::proto::geometry_msgs::Pose pose3d = Pose2DToPose3D(bbox.center());

            foxglove::schemas::CubePrimitive cube;
            cube.pose = CreatePose(pose3d);

            cube.size = foxglove::schemas::Vector3();
            cube.size->x = bbox.size_x();
            cube.size->y = bbox.size_y();
            cube.size->z = 0.01;

            // Use different colors for different boxes
            double hue = static_cast<double>(i) / message.boxes_size();
            cube.color = CreateColor(0.0, 1.0, 1.0, 0.5);

            entity.cubes.push_back(cube);
        }
    }

    if (!entity.cubes.empty()) {
        scene_update.entities.push_back(entity);
    }

    AINFO << "Converted BoundingBox2DArray with " << message.boxes_size() << " boxes to SceneUpdate";

    return scene_update;
}

// BoundingBox3D conversion
foxglove::schemas::SceneUpdate ToFoxgloveImpl(const autonomy::commsgs::proto::vision_msgs::BoundingBox3D& message) {
    foxglove::schemas::SceneUpdate scene_update;

    foxglove::schemas::SceneEntity entity;
    SetEntityHeader(entity, std::nullopt,
                    std::nullopt);  // BoundingBox3D has no header

    entity.id = "bbox3d";
    entity.frame_locked = false;

    // Create a cube to represent the 3D bounding box
    if (message.has_center() && message.has_size()) {
        foxglove::schemas::CubePrimitive cube;
        cube.pose = CreatePose(message.center());

        cube.size = foxglove::schemas::Vector3();
        cube.size->x = message.size().x();
        cube.size->y = message.size().y();
        cube.size->z = message.size().z();

        // Default color for 3D bounding boxes (magenta)
        cube.color = CreateColor(1.0, 0.0, 1.0, 0.5);

        entity.cubes.push_back(cube);
    }

    if (!entity.cubes.empty()) {
        scene_update.entities.push_back(entity);
    }

    AINFO << "Converted BoundingBox3D to SceneUpdate";

    return scene_update;
}

// BoundingBox3DArray conversion
foxglove::schemas::SceneUpdate ToFoxgloveImpl(
    const autonomy::commsgs::proto::vision_msgs::BoundingBox3DArray& message) {
    foxglove::schemas::SceneUpdate scene_update;

    if (message.boxes_size() == 0) {
        AWARN << "BoundingBox3DArray is empty, returning empty SceneUpdate";
        return scene_update;
    }

    foxglove::schemas::SceneEntity entity;
    SetEntityHeader(entity, ExtractTimestamp(message), ExtractFrameId(message));

    entity.id = "bbox3d_array";
    entity.frame_locked = false;

    // Convert each 3D bounding box
    for (int i = 0; i < message.boxes_size(); ++i) {
        const auto& bbox = message.boxes(i);
        if (bbox.has_center() && bbox.has_size()) {
            foxglove::schemas::CubePrimitive cube;
            cube.pose = CreatePose(bbox.center());

            cube.size = foxglove::schemas::Vector3();
            cube.size->x = bbox.size().x();
            cube.size->y = bbox.size().y();
            cube.size->z = bbox.size().z();

            // Use different colors for different boxes
            cube.color = CreateColor(1.0, 0.0, 1.0, 0.5);

            entity.cubes.push_back(cube);
        }
    }

    if (!entity.cubes.empty()) {
        scene_update.entities.push_back(entity);
    }

    AINFO << "Converted BoundingBox3DArray with " << message.boxes_size() << " boxes to SceneUpdate";

    return scene_update;
}

// Detection2D conversion
foxglove::schemas::SceneUpdate ToFoxgloveImpl(const autonomy::commsgs::proto::vision_msgs::Detection2D& message) {
    foxglove::schemas::SceneUpdate scene_update;

    foxglove::schemas::SceneEntity entity;
    SetEntityHeader(entity, ExtractTimestamp(message), ExtractFrameId(message));

    // Use detection ID if available, otherwise use index
    entity.id = message.id().empty() ? "detection2d" : "detection2d_" + message.id();
    entity.frame_locked = false;

    // Visualize the 2D bounding box
    if (message.has_bbox()) {
        auto bbox_entity = ToFoxgloveImpl(message.bbox());
        if (!bbox_entity.entities.empty()) {
            // Merge the bounding box visualization into the detection entity
            if (!bbox_entity.entities[0].cubes.empty()) {
                entity.cubes.insert(entity.cubes.end(), bbox_entity.entities[0].cubes.begin(),
                                    bbox_entity.entities[0].cubes.end());
            }
        }
    }

    // Visualize the best hypothesis pose if available
    if (message.results_size() > 0) {
        const auto& best_hypothesis = message.results(0);
        if (best_hypothesis.has_pose() && best_hypothesis.pose().has_pose()) {
            // Create an arrow to indicate the pose
            foxglove::schemas::ArrowPrimitive arrow;
            arrow.pose = CreatePose(best_hypothesis.pose().pose());

            // Scale arrow based on confidence score
            double scale = best_hypothesis.hypothesis().score();
            arrow.shaft_length = 0.5 * scale;
            arrow.head_length = 0.1 * scale;
            arrow.shaft_diameter = 0.02;
            arrow.head_diameter = 0.05;

            // Color based on confidence (green = high, red = low)
            double confidence = best_hypothesis.hypothesis().score();
            arrow.color = CreateColor(1.0 - confidence, confidence, 0.0, 1.0);

            entity.arrows.push_back(arrow);
        }
    }

    if (!entity.cubes.empty() || !entity.arrows.empty()) {
        scene_update.entities.push_back(entity);
    }

    AINFO << "Converted Detection2D (id=" << message.id() << ") to SceneUpdate with " << message.results_size()
          << " hypotheses";

    return scene_update;
}

// Detection2DArray conversion
foxglove::schemas::SceneUpdate ToFoxgloveImpl(const autonomy::commsgs::proto::vision_msgs::Detection2DArray& message) {
    foxglove::schemas::SceneUpdate scene_update;

    if (message.detections_size() == 0) {
        AWARN << "Detection2DArray is empty, returning empty SceneUpdate";
        return scene_update;
    }

    // Convert each detection
    for (int i = 0; i < message.detections_size(); ++i) {
        auto detection_update = ToFoxgloveImpl(message.detections(i));
        scene_update.entities.insert(scene_update.entities.end(), detection_update.entities.begin(),
                                     detection_update.entities.end());
    }

    AINFO << "Converted Detection2DArray with " << message.detections_size() << " detections to SceneUpdate";

    return scene_update;
}

// Detection3D conversion
foxglove::schemas::SceneUpdate ToFoxgloveImpl(const autonomy::commsgs::proto::vision_msgs::Detection3D& message) {
    foxglove::schemas::SceneUpdate scene_update;

    foxglove::schemas::SceneEntity entity;
    SetEntityHeader(entity, ExtractTimestamp(message), ExtractFrameId(message));

    // Use detection ID if available
    entity.id = message.id().empty() ? "detection3d" : "detection3d_" + message.id();
    entity.frame_locked = false;

    // Visualize the 3D bounding box
    if (message.has_bbox()) {
        auto bbox_entity = ToFoxgloveImpl(message.bbox());
        if (!bbox_entity.entities.empty()) {
            // Merge the bounding box visualization
            if (!bbox_entity.entities[0].cubes.empty()) {
                entity.cubes.insert(entity.cubes.end(), bbox_entity.entities[0].cubes.begin(),
                                    bbox_entity.entities[0].cubes.end());
            }
        }
    }

    // Visualize the best hypothesis pose if available
    if (message.results_size() > 0) {
        const auto& best_hypothesis = message.results(0);
        if (best_hypothesis.has_pose() && best_hypothesis.pose().has_pose()) {
            // Create an arrow to indicate the pose
            foxglove::schemas::ArrowPrimitive arrow;
            arrow.pose = CreatePose(best_hypothesis.pose().pose());

            // Scale arrow based on confidence score
            double scale = best_hypothesis.hypothesis().score();
            arrow.shaft_length = 0.5 * scale;
            arrow.head_length = 0.1 * scale;
            arrow.shaft_diameter = 0.02;
            arrow.head_diameter = 0.05;

            // Color based on confidence (green = high, red = low)
            double confidence = best_hypothesis.hypothesis().score();
            arrow.color = CreateColor(1.0 - confidence, confidence, 0.0, 1.0);

            entity.arrows.push_back(arrow);
        }
    }

    if (!entity.cubes.empty() || !entity.arrows.empty()) {
        scene_update.entities.push_back(entity);
    }

    AINFO << "Converted Detection3D (id=" << message.id() << ") to SceneUpdate with " << message.results_size()
          << " hypotheses";

    return scene_update;
}

// Detection3DArray conversion
foxglove::schemas::SceneUpdate ToFoxgloveImpl(const autonomy::commsgs::proto::vision_msgs::Detection3DArray& message) {
    foxglove::schemas::SceneUpdate scene_update;

    if (message.detections_size() == 0) {
        AWARN << "Detection3DArray is empty, returning empty SceneUpdate";
        return scene_update;
    }

    // Convert each detection
    for (int i = 0; i < message.detections_size(); ++i) {
        auto detection_update = ToFoxgloveImpl(message.detections(i));
        scene_update.entities.insert(scene_update.entities.end(), detection_update.entities.begin(),
                                     detection_update.entities.end());
    }

    AINFO << "Converted Detection3DArray with " << message.detections_size() << " detections to SceneUpdate";

    return scene_update;
}

// Classification conversion
// Classification doesn't have spatial information, so we return an empty
// SceneUpdate In a more advanced implementation, this could display text labels
// or confidence bars
foxglove::schemas::SceneUpdate ToFoxgloveImpl(const autonomy::commsgs::proto::vision_msgs::Classification& message) {
    (void)message;  // Unused for now
    AWARN << "Classification messages have no spatial information, "
          << "returning empty SceneUpdate";
    return foxglove::schemas::SceneUpdate();
}

// ObjectHypothesis conversion
// ObjectHypothesis doesn't have spatial information, so we return an empty
// SceneUpdate
foxglove::schemas::SceneUpdate ToFoxgloveImpl(const autonomy::commsgs::proto::vision_msgs::ObjectHypothesis& message) {
    (void)message;  // Unused for now
    AWARN << "ObjectHypothesis messages have no spatial information, "
          << "returning empty SceneUpdate";
    return foxglove::schemas::SceneUpdate();
}

// ObjectHypothesisWithPose conversion
foxglove::schemas::SceneUpdate ToFoxgloveImpl(
    const autonomy::commsgs::proto::vision_msgs::ObjectHypothesisWithPose& message) {
    foxglove::schemas::SceneUpdate scene_update;

    foxglove::schemas::SceneEntity entity;
    SetEntityHeader(entity, std::nullopt, std::nullopt);

    entity.id = "object_hypothesis_" + message.hypothesis().class_id();
    entity.frame_locked = false;

    // Visualize the pose if available
    if (message.has_pose() && message.pose().has_pose()) {
        // Create an arrow to indicate the pose
        foxglove::schemas::ArrowPrimitive arrow;
        arrow.pose = CreatePose(message.pose().pose());

        // Scale arrow based on confidence score
        double scale = message.hypothesis().score();
        arrow.shaft_length = 0.3 * scale;
        arrow.head_length = 0.1 * scale;
        arrow.shaft_diameter = 0.02;
        arrow.head_diameter = 0.05;

        // Color based on confidence
        double confidence = message.hypothesis().score();
        arrow.color = CreateColor(1.0 - confidence, confidence, 0.0, 1.0);

        entity.arrows.push_back(arrow);
    }

    if (!entity.arrows.empty()) {
        scene_update.entities.push_back(entity);
    }

    AINFO << "Converted ObjectHypothesisWithPose (class_id=" << message.hypothesis().class_id()
          << ", score=" << message.hypothesis().score() << ") to SceneUpdate";

    return scene_update;
}

}  // namespace impl
}  // namespace converter
}  // namespace visualization
}  // namespace autonomy
