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

#pragma once 

#include <vector>
#include <string>

#include "autonomy/commsgs/proto/shape_msgs.pb.h"

#include "autonomy/common/port.hpp"
#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"

namespace autonomy {
namespace commsgs {
namespace builtin_interfaces {

struct Plane
{
    // Representation of a plane, using the plane equation ax + by + cz + d = 0.
    //
    // a := coef[0]
    // b := coef[1]
    // c := coef[2]
    // d := coef[3]
    // float64[4] coef
    std::vector<double> coef;
};

// Defines box, sphere, cylinder, cone and prism.
// All shapes are defined to have their bounding boxes centered around 0,0,0.

// uint8 BOX=1
// uint8 SPHERE=2
// uint8 CYLINDER=3
// uint8 CONE=4
// uint8 PRISM=5

struct SolidPrimitive
{
    // The type of the shape
    uint32 type;
    
    // The dimensions of the shape
    // float64[<=3] dimensions  // At no point will dimensions have a length > 3.
    std::vector<double> dimensions;
    
    // The meaning of the shape dimensions: each constant defines the index in the 'dimensions' array.
    
    // For type BOX, the X, Y, and Z dimensions are the length of the corresponding sides of the box.
    // uint32 BOX_X=0
    // uint32 BOX_Y=1
    // uint32 BOX_Z=2
    
    // // For the SPHERE type, only one component is used, and it gives the radius of the sphere.
    // uint8 SPHERE_RADIUS=0
    
    // // For the CYLINDER and CONE types, the center line is oriented along the Z axis.
    // // Therefore the CYLINDER_HEIGHT (CONE_HEIGHT) component of dimensions gives the
    // // height of the cylinder (cone).
    // // The CYLINDER_RADIUS (CONE_RADIUS) component of dimensions gives the radius of
    // // the base of the cylinder (cone).
    // // Cone and cylinder primitives are defined to be circular. The tip of the cone
    // // is pointing up, along +Z axis.
    
    // uint8 CYLINDER_HEIGHT=0
    // uint8 CYLINDER_RADIUS=1
    
    // uint8 CONE_HEIGHT=0
    // uint8 CONE_RADIUS=1
    
    // // For the type PRISM, the center line is oriented along Z axis.
    // // The PRISM_HEIGHT component of dimensions gives the
    // // height of the prism.
    // // The polygon defines the Z axis centered base of the prism.
    // // The prism is constructed by extruding the base in +Z and -Z
    // // directions by half of the PRISM_HEIGHT
    // // Only x and y fields of the points are used in the polygon.
    // // Points of the polygon are ordered counter-clockwise.
    
    // uint8 PRISM_HEIGHT=0
    geometry_msgs::Polygon polygon;
};


// Definition of a triangle's vertices.
struct MeshTriangle
{
    // uint32[3] vertex_indices
    std::vector<uint32> vertex_indices; 
};

// Definition of a mesh.
struct Mesh
{
    // List of triangles; the index values refer to positions in vertices[].
    std::vector<MeshTriangle> triangles;

    // The actual vertices that make up the mesh.
    std::vector<geometry_msgs::Point> vertices;
};

}  // namespace builtin_interfaces
}  // namespace commsgs
}  // namespace autonomy