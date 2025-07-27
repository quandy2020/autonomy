/*
 * Copyright 2024 The OpenRobotic Beginner Authors
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

#include <string>
#include <vector>

#include "autonomy/common/port.hpp"
namespace autonomy {
namespace common {

struct PlyPoint 
{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float nx = 0.0f;
  float ny = 0.0f;
  float nz = 0.0f;
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
};

struct PlyMeshVertex 
{
  PlyMeshVertex() : x(0), y(0), z(0) {}
  PlyMeshVertex(const float x, const float y, const float z)
      : x(x), y(y), z(z) {}

  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct PlyMeshFace 
{
  PlyMeshFace() : vertex_idx1(0), vertex_idx2(0), vertex_idx3(0) {}
  PlyMeshFace(const size_t vertex_idx1,
              const size_t vertex_idx2,
              const size_t vertex_idx3)
      : vertex_idx1(vertex_idx1),
        vertex_idx2(vertex_idx2),
        vertex_idx3(vertex_idx3) {}

  size_t vertex_idx1 = 0;
  size_t vertex_idx2 = 0;
  size_t vertex_idx3 = 0;
};

struct PlyMesh {
  std::vector<PlyMeshVertex> vertices;
  std::vector<PlyMeshFace> faces;
};

// Read PLY point cloud from text or binary file.
std::vector<PlyPoint> ReadPly(const std::string& path);

// Write PLY point cloud to text or binary file.
void WriteTextPlyPoints(const std::string& path,
                        const std::vector<PlyPoint>& points,
                        bool write_normal = true,
                        bool write_rgb = true);
void WriteBinaryPlyPoints(const std::string& path,
                          const std::vector<PlyPoint>& points,
                          bool write_normal = true,
                          bool write_rgb = true);

// Write PLY mesh to text or binary file.
void WriteTextPlyMesh(const std::string& path, const PlyMesh& mesh);
void WriteBinaryPlyMesh(const std::string& path, const PlyMesh& mesh);

}  // namespace common
}  // namespace autonomy

