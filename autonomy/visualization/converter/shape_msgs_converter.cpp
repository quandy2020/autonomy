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

#include "autonomy/visualization/converter/shape_msgs_converter.hpp"

#include <cmath>
#include <optional>

#include "autolink/autolink.hpp"
#include "autonomy/visualization/converter/converter_detail.hpp"

namespace autonomy {
namespace visualization {
namespace converter {
namespace impl {

using detail::CreateColor;
using detail::SetEntityHeader;

// Plane conversion
// Note: Plane equations (ax + by + cz + d = 0) are typically not directly
// visualizable. We return an empty SceneUpdate for now.
// In the future, this could visualize a plane by creating a large rectangular
// surface aligned with the plane.
foxglove::schemas::SceneUpdate ToFoxgloveImpl(const autonomy::commsgs::proto::shape_msgs::Plane& message) {
    (void)message;  // Unused for now
    AWARN << "Plane messages are not directly visualizable, "
          << "returning empty SceneUpdate";
    return foxglove::schemas::SceneUpdate();
}

// SolidPrimitive conversion
foxglove::schemas::SceneUpdate ToFoxgloveImpl(const autonomy::commsgs::proto::shape_msgs::SolidPrimitive& message) {
    foxglove::schemas::SceneUpdate scene_update;

    foxglove::schemas::SceneEntity entity;
    SetEntityHeader(entity, std::nullopt, std::nullopt);

    entity.id = "solid_primitive";
    entity.frame_locked = false;

    // Default pose at origin (SolidPrimitive is defined centered at 0,0,0)
    foxglove::schemas::Pose default_pose;
    default_pose.position = foxglove::schemas::Vector3();
    default_pose.position->x = 0.0;
    default_pose.position->y = 0.0;
    default_pose.position->z = 0.0;
    default_pose.orientation = foxglove::schemas::Quaternion();
    default_pose.orientation->x = 0.0;
    default_pose.orientation->y = 0.0;
    default_pose.orientation->z = 0.0;
    default_pose.orientation->w = 1.0;

    uint32_t type = message.type();
    const auto& dimensions = message.dimensions();

    // Default color (light blue)
    auto default_color = CreateColor(0.5, 0.8, 1.0, 0.7);

    switch (type) {
        case 1: {  // BOX
            foxglove::schemas::CubePrimitive cube;
            cube.pose = default_pose;

            cube.size = foxglove::schemas::Vector3();
            if (dimensions.size() >= 3) {
                cube.size->x = dimensions[0];  // BOX_X
                cube.size->y = dimensions[1];  // BOX_Y
                cube.size->z = dimensions[2];  // BOX_Z
            } else if (dimensions.size() >= 2) {
                cube.size->x = dimensions[0];
                cube.size->y = dimensions[1];
                cube.size->z = dimensions[1];  // Use Y for Z if Z not provided
            } else if (dimensions.size() >= 1) {
                cube.size->x = dimensions[0];
                cube.size->y = dimensions[0];
                cube.size->z = dimensions[0];  // Use X for all dimensions
            } else {
                cube.size->x = 1.0;
                cube.size->y = 1.0;
                cube.size->z = 1.0;
            }

            cube.color = default_color;
            entity.cubes.push_back(cube);
            break;
        }
        case 2: {  // SPHERE
            foxglove::schemas::SpherePrimitive sphere;
            sphere.pose = default_pose;

            sphere.size = foxglove::schemas::Vector3();
            double radius = (dimensions.size() >= 1) ? dimensions[0] : 0.5;
            sphere.size->x = radius * 2.0;
            sphere.size->y = radius * 2.0;
            sphere.size->z = radius * 2.0;

            sphere.color = default_color;
            entity.spheres.push_back(sphere);
            break;
        }
        case 3: {  // CYLINDER
            foxglove::schemas::CylinderPrimitive cylinder;
            cylinder.pose = default_pose;

            cylinder.size = foxglove::schemas::Vector3();
            if (dimensions.size() >= 2) {
                double height = dimensions[0];  // CYLINDER_HEIGHT
                double radius = dimensions[1];  // CYLINDER_RADIUS
                cylinder.size->x = height;
                cylinder.size->y = radius * 2.0;
                cylinder.size->z = radius * 2.0;
            } else if (dimensions.size() >= 1) {
                double radius = dimensions[0];
                cylinder.size->x = radius * 2.0;
                cylinder.size->y = radius * 2.0;
                cylinder.size->z = radius * 2.0;
            } else {
                cylinder.size->x = 1.0;
                cylinder.size->y = 0.5;
                cylinder.size->z = 0.5;
            }

            cylinder.color = default_color;
            entity.cylinders.push_back(cylinder);
            break;
        }
        case 4: {  // CONE
            foxglove::schemas::CylinderPrimitive cone;
            cone.pose = default_pose;

            cone.size = foxglove::schemas::Vector3();
            if (dimensions.size() >= 2) {
                double height = dimensions[0];  // CONE_HEIGHT
                double radius = dimensions[1];  // CONE_RADIUS
                cone.size->x = height;
                cone.size->y = radius * 2.0;
                cone.size->z = radius * 2.0;
                cone.top_scale = 0.0;  // Cone has a point at the top
                cone.bottom_scale = 1.0;
            } else {
                cone.size->x = 1.0;
                cone.size->y = 0.5;
                cone.size->z = 0.5;
                cone.top_scale = 0.0;
                cone.bottom_scale = 1.0;
            }

            cone.color = default_color;
            entity.cylinders.push_back(cone);
            break;
        }
        case 5: {  // PRISM
            // PRISM requires polygon information which is in the polygon field
            // For now, create a cylinder as approximation
            // In the future, this could be converted to a mesh
            AWARN << "PRISM type not fully supported, using cylinder "
                     "approximation";
            foxglove::schemas::CylinderPrimitive prism;
            prism.pose = default_pose;

            prism.size = foxglove::schemas::Vector3();
            if (dimensions.size() >= 1) {
                double height = dimensions[0];  // PRISM_HEIGHT
                prism.size->x = height;
                prism.size->y = 1.0;
                prism.size->z = 1.0;
            } else {
                prism.size->x = 1.0;
                prism.size->y = 1.0;
                prism.size->z = 1.0;
            }

            prism.color = default_color;
            entity.cylinders.push_back(prism);
            break;
        }
        default:
            AWARN << "Unknown SolidPrimitive type: " << type << ", returning empty SceneUpdate";
            return scene_update;
    }

    if (!entity.cubes.empty() || !entity.spheres.empty() || !entity.cylinders.empty()) {
        scene_update.entities.push_back(entity);
    }

    AINFO << "Converted SolidPrimitive (type=" << type << ") to SceneUpdate";

    return scene_update;
}

// Mesh conversion
// Note: Foxglove may not have direct Mesh primitive support.
// For now, we convert mesh to lines/points for visualization.
// In the future, this could be converted to a proper mesh representation.
foxglove::schemas::SceneUpdate ToFoxgloveImpl(const autonomy::commsgs::proto::shape_msgs::Mesh& message) {
    foxglove::schemas::SceneUpdate scene_update;

    if (message.triangles_size() == 0 || message.vertices_size() == 0) {
        AWARN << "Mesh is empty, returning empty SceneUpdate";
        return scene_update;
    }

    foxglove::schemas::SceneEntity entity;
    SetEntityHeader(entity, std::nullopt, std::nullopt);

    entity.id = "mesh";
    entity.frame_locked = false;

    // Convert mesh edges to lines for wireframe visualization
    // Each triangle has 3 edges, we'll visualize all edges
    foxglove::schemas::LinePrimitive line;
    line.type = foxglove::schemas::LinePrimitive::LineType::LINE_LIST;
    line.thickness = 0.01;
    line.scale_invariant = false;
    line.color = CreateColor(0.8, 0.8, 0.8, 0.8);

    // Add triangle edges as line segments
    // LINE_LIST requires pairs of points for each line segment
    for (const auto& triangle : message.triangles()) {
        if (triangle.vertex_indices_size() >= 3) {
            uint32_t idx0 = triangle.vertex_indices(0);
            uint32_t idx1 = triangle.vertex_indices(1);
            uint32_t idx2 = triangle.vertex_indices(2);

            // Check bounds
            if (idx0 < message.vertices_size() && idx1 < message.vertices_size() && idx2 < message.vertices_size()) {
                const auto& v0 = message.vertices(idx0);
                const auto& v1 = message.vertices(idx1);
                const auto& v2 = message.vertices(idx2);

                // Edge 0-1: add both endpoints
                foxglove::schemas::Point3 p0;
                p0.x = v0.x();
                p0.y = v0.y();
                p0.z = v0.z();
                line.points.push_back(p0);

                foxglove::schemas::Point3 p1;
                p1.x = v1.x();
                p1.y = v1.y();
                p1.z = v1.z();
                line.points.push_back(p1);

                // Edge 1-2: add both endpoints
                line.points.push_back(p1);
                foxglove::schemas::Point3 p2;
                p2.x = v2.x();
                p2.y = v2.y();
                p2.z = v2.z();
                line.points.push_back(p2);

                // Edge 2-0: add both endpoints
                line.points.push_back(p2);
                line.points.push_back(p0);
            }
        }
    }

    if (!line.points.empty()) {
        entity.lines.push_back(line);
    }

    if (!entity.lines.empty()) {
        scene_update.entities.push_back(entity);
    }

    AINFO << "Converted Mesh to SceneUpdate: " << message.vertices_size() << " vertices, " << message.triangles_size()
          << " triangles";

    return scene_update;
}

// MeshTriangle conversion
// Note: A single MeshTriangle doesn't have enough information for standalone
// visualization (it only has indices, not vertices). We return an empty
// SceneUpdate. Use Mesh message instead.
foxglove::schemas::SceneUpdate ToFoxgloveImpl(const autonomy::commsgs::proto::shape_msgs::MeshTriangle& message) {
    (void)message;  // Unused
    AWARN << "MeshTriangle requires vertex data from Mesh message, "
          << "returning empty SceneUpdate. Use Mesh message instead.";
    return foxglove::schemas::SceneUpdate();
}

}  // namespace impl
}  // namespace converter
}  // namespace visualization
}  // namespace autonomy
