/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/primitive_mesh.hpp"

#include <QtMath>

namespace autoviz {
namespace display {
namespace {

constexpr float kTwoPi = 6.28318530718f;

void AddTriangle(ObjMesh* mesh, int a, int b, int c) {
  mesh->triangles.push_back({a, b, c});
}

}  // namespace

ObjMesh buildCylinderMesh(float radius, float length, int slices) {
  ObjMesh mesh;
  if (radius <= 0.f || length <= 0.f || slices < 3) {
    return mesh;
  }

  const float half = length * 0.5f;
  const int bottom_center = 0;
  const int top_center = 1;
  mesh.vertices.push_back(QVector3D(0.f, 0.f, -half));
  mesh.vertices.push_back(QVector3D(0.f, 0.f, half));

  for (int i = 0; i < slices; ++i) {
    const float angle = kTwoPi * static_cast<float>(i) / static_cast<float>(slices);
    const float x = radius * std::cos(angle);
    const float y = radius * std::sin(angle);
    mesh.vertices.push_back(QVector3D(x, y, -half));
    mesh.vertices.push_back(QVector3D(x, y, half));
  }

  for (int i = 0; i < slices; ++i) {
    const int next = (i + 1) % slices;
    const int bottom_i = 2 + i * 2;
    const int top_i = bottom_i + 1;
    const int bottom_next = 2 + next * 2;
    const int top_next = bottom_next + 1;
    AddTriangle(&mesh, bottom_i, top_i, top_next);
    AddTriangle(&mesh, bottom_i, top_next, bottom_next);
    AddTriangle(&mesh, bottom_center, bottom_next, bottom_i);
    AddTriangle(&mesh, top_center, top_i, top_next);
  }

  return mesh;
}

ObjMesh buildSphereMesh(float radius, int slices, int stacks) {
  ObjMesh mesh;
  if (radius <= 0.f || slices < 3 || stacks < 2) {
    return mesh;
  }

  for (int stack = 0; stack <= stacks; ++stack) {
    const float v = static_cast<float>(stack) / static_cast<float>(stacks);
    const float phi = v * static_cast<float>(M_PI);
    const float sin_phi = std::sin(phi);
    const float cos_phi = std::cos(phi);
    for (int slice = 0; slice <= slices; ++slice) {
      const float u = static_cast<float>(slice) / static_cast<float>(slices);
      const float theta = u * kTwoPi;
      const float x = radius * sin_phi * std::cos(theta);
      const float y = radius * sin_phi * std::sin(theta);
      const float z = radius * cos_phi;
      mesh.vertices.emplace_back(x, y, z);
    }
  }

  const int row_stride = slices + 1;
  for (int stack = 0; stack < stacks; ++stack) {
    for (int slice = 0; slice < slices; ++slice) {
      const int current = stack * row_stride + slice;
      const int next = current + row_stride;
      AddTriangle(&mesh, current, next, current + 1);
      AddTriangle(&mesh, current + 1, next, next + 1);
    }
  }

  return mesh;
}

ObjMesh buildConeMesh(float radius, float height, int slices) {
  ObjMesh mesh;
  if (radius <= 0.f || height <= 0.f || slices < 3) {
    return mesh;
  }

  const int apex = 0;
  mesh.vertices.push_back(QVector3D(0.f, 0.f, height));

  for (int i = 0; i < slices; ++i) {
    const float angle = kTwoPi * static_cast<float>(i) / static_cast<float>(slices);
    mesh.vertices.push_back(
        QVector3D(radius * std::cos(angle), radius * std::sin(angle), 0.f));
  }

  for (int i = 0; i < slices; ++i) {
    const int next = (i + 1) % slices;
    AddTriangle(&mesh, apex, 1 + i, 1 + next);
  }

  return mesh;
}

ObjMesh buildCapsuleMesh(float radius, float length, int slices) {
  ObjMesh mesh = buildSphereMesh(radius, slices, slices);
  if (mesh.vertices.empty() || radius <= 0.f || length <= 0.f) {
    return mesh;
  }
  const float scale_z = length / (2.f * radius);
  for (QVector3D& vertex : mesh.vertices) {
    vertex.setZ(vertex.z() * scale_z);
  }
  return mesh;
}

ObjMesh buildCubeMesh() {
  ObjMesh mesh;
  const float h = 0.5f;
  mesh.vertices = {
      {-h, -h, -h}, {h, -h, -h}, {h, h, -h}, {-h, h, -h},
      {-h, -h, h},  {h, -h, h},  {h, h, h},  {-h, h, h},
  };
  const int faces[12][3] = {
      {0, 1, 2}, {0, 2, 3}, {4, 6, 5}, {4, 7, 6}, {0, 4, 5}, {0, 5, 1},
      {2, 6, 7}, {2, 7, 3}, {0, 3, 7}, {0, 7, 4}, {1, 5, 6}, {1, 6, 2},
  };
  for (const auto& face : faces) {
    AddTriangle(&mesh, face[0], face[1], face[2]);
  }
  return mesh;
}

}  // namespace display
}  // namespace autoviz
