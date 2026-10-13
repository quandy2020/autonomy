/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/ogre_procedural_shape.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include <Ogre.h>

#include "autoviz/plugins/displays/common/obj_mesh.hpp"
#include "autoviz/plugins/displays/common/primitive_mesh.hpp"

namespace autoviz {
namespace rendering {
namespace {

constexpr char kAvizMeshGroup[] = "aviz_rendering";

void SetMeshBoundsFromVertices(Ogre::MeshPtr mesh, const display::ObjMesh& obj) {
  if (obj.vertices.empty()) {
    mesh->_setBounds(Ogre::AxisAlignedBox(-1.f, -1.f, -1.f, 1.f, 1.f, 1.f),
                     false);
    mesh->_setBoundingSphereRadius(1.f);
    return;
  }
  float min_x = std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float min_z = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float max_y = std::numeric_limits<float>::lowest();
  float max_z = std::numeric_limits<float>::lowest();
  for (const QVector3D& v : obj.vertices) {
    min_x = std::min(min_x, v.x());
    min_y = std::min(min_y, v.y());
    min_z = std::min(min_z, v.z());
    max_x = std::max(max_x, v.x());
    max_y = std::max(max_y, v.y());
    max_z = std::max(max_z, v.z());
  }
  mesh->_setBounds(Ogre::AxisAlignedBox(min_x, min_y, min_z, max_x, max_y, max_z),
                   false);
  const float cx = 0.5f * (min_x + max_x);
  const float cy = 0.5f * (min_y + max_y);
  const float cz = 0.5f * (min_z + max_z);
  float radius_sq = 0.f;
  for (const QVector3D& v : obj.vertices) {
    const float dx = v.x() - cx;
    const float dy = v.y() - cy;
    const float dz = v.z() - cz;
    radius_sq = std::max(radius_sq, dx * dx + dy * dy + dz * dz);
  }
  mesh->_setBoundingSphereRadius(std::sqrt(radius_sq));
}

void UploadObjMesh(Ogre::MeshPtr mesh, const display::ObjMesh& obj_in) {
  display::ObjMesh obj = obj_in;
  display::ensureMeshNormals(&obj);

  mesh->sharedVertexData = new Ogre::VertexData();
  Ogre::VertexData* vertex_data = mesh->sharedVertexData;
  vertex_data->vertexCount = obj.vertices.size();

  Ogre::VertexDeclaration* decl = vertex_data->vertexDeclaration;
  size_t offset = 0;
  decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
  offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
  // RViz Assimp path always provides normals so Ogre lighting can shade the
  // mesh. Without VES_NORMAL, createMaterialWithLighting looks flat grey.
  decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
  offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

  Ogre::HardwareVertexBufferSharedPtr vbuf =
      Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
          decl->getVertexSize(0), vertex_data->vertexCount,
          Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
  float* vertices = static_cast<float*>(vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));
  for (std::size_t i = 0; i < obj.vertices.size(); ++i) {
    *vertices++ = obj.vertices[i].x();
    *vertices++ = obj.vertices[i].y();
    *vertices++ = obj.vertices[i].z();
    const QVector3D& n = obj.normals[i];
    *vertices++ = n.x();
    *vertices++ = n.y();
    *vertices++ = n.z();
  }
  vbuf->unlock();
  vertex_data->vertexBufferBinding->setBinding(0, vbuf);

  mesh->sharedVertexData->vertexStart = 0;
  mesh->sharedVertexData->vertexCount = obj.vertices.size();

  Ogre::SubMesh* sub = mesh->createSubMesh();
  sub->useSharedVertices = true;
  sub->operationType = Ogre::RenderOperation::OT_TRIANGLE_LIST;
  sub->indexData->indexCount = obj.triangles.size() * 3;
  // RViz assimp_loader: 16-bit when vertexCount < 65536, else 32-bit.
  const bool use_16_bits = vertex_data->vertexCount < (1u << 16);
  Ogre::HardwareIndexBufferSharedPtr ibuf =
      Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
          use_16_bits ? Ogre::HardwareIndexBuffer::IT_16BIT
                      : Ogre::HardwareIndexBuffer::IT_32BIT,
          sub->indexData->indexCount,
          Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
  if (use_16_bits) {
    auto* indices =
        static_cast<uint16_t*>(ibuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));
    std::size_t idx = 0;
    for (const auto& tri : obj.triangles) {
      indices[idx++] = static_cast<uint16_t>(tri[0]);
      indices[idx++] = static_cast<uint16_t>(tri[1]);
      indices[idx++] = static_cast<uint16_t>(tri[2]);
    }
  } else {
    auto* indices =
        static_cast<uint32_t*>(ibuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));
    std::size_t idx = 0;
    for (const auto& tri : obj.triangles) {
      indices[idx++] = static_cast<uint32_t>(tri[0]);
      indices[idx++] = static_cast<uint32_t>(tri[1]);
      indices[idx++] = static_cast<uint32_t>(tri[2]);
    }
  }
  ibuf->unlock();
  sub->indexData->indexBuffer = ibuf;
  sub->indexData->indexStart = 0;

  SetMeshBoundsFromVertices(mesh, obj);
  mesh->load();
}

}  // namespace

void registerObjMesh(const std::string& mesh_name, const display::ObjMesh& obj) {
  if (obj.vertices.empty() || obj.triangles.empty()) {
    return;
  }
  if (Ogre::MeshManager::getSingleton().resourceExists(mesh_name, kAvizMeshGroup)) {
    return;
  }
  Ogre::MeshPtr mesh =
      Ogre::MeshManager::getSingleton().createManual(mesh_name, kAvizMeshGroup);
  UploadObjMesh(mesh, obj);
}

void ensureAvizPrimitiveMeshes() {
  registerObjMesh("aviz_capsule.mesh", display::buildCapsuleMesh(0.5f, 1.f));
  registerObjMesh("aviz_cylinder.mesh", display::buildCylinderMesh(0.5f, 1.f));
  registerObjMesh("aviz_cone.mesh", display::buildConeMesh(0.5f, 1.f));
  registerObjMesh("aviz_sphere.mesh", display::buildSphereMesh(0.5f));
  registerObjMesh("aviz_cube.mesh", display::buildCubeMesh());
}

}  // namespace rendering
}  // namespace autoviz
