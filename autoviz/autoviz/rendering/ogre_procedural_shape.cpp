/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/ogre_procedural_shape.hpp"

#ifdef AUTOVIZ_USE_OGRE

#include <Ogre.h>

#include "autoviz/display/primitive_mesh.hpp"

namespace autoviz {
namespace rendering {
namespace {

constexpr char kRvizMeshGroup[] = "rviz_rendering";

void UploadObjMesh(Ogre::MeshPtr mesh, const display::ObjMesh& obj) {
  mesh->sharedVertexData = new Ogre::VertexData();
  Ogre::VertexData* vertex_data = mesh->sharedVertexData;
  vertex_data->vertexCount = obj.vertices.size();

  Ogre::VertexDeclaration* decl = vertex_data->vertexDeclaration;
  size_t offset = 0;
  decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
  offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

  Ogre::HardwareVertexBufferSharedPtr vbuf =
      Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
          decl->getVertexSize(0), vertex_data->vertexCount,
          Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
  float* vertices = static_cast<float*>(vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));
  for (std::size_t i = 0; i < obj.vertices.size(); ++i) {
    vertices[i * 3 + 0] = obj.vertices[i].x();
    vertices[i * 3 + 1] = obj.vertices[i].y();
    vertices[i * 3 + 2] = obj.vertices[i].z();
  }
  vbuf->unlock();
  vertex_data->vertexBufferBinding->setBinding(0, vbuf);

  mesh->sharedVertexData->vertexStart = 0;
  mesh->sharedVertexData->vertexCount = obj.vertices.size();

  Ogre::SubMesh* sub = mesh->createSubMesh();
  sub->useSharedVertices = true;
  sub->operationType = Ogre::RenderOperation::OT_TRIANGLE_LIST;
  sub->indexData->indexCount = obj.triangles.size() * 3;
  Ogre::HardwareIndexBufferSharedPtr ibuf =
      Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
          Ogre::HardwareIndexBuffer::IT_16BIT, sub->indexData->indexCount,
          Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
  auto* indices = static_cast<uint16_t*>(ibuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));
  std::size_t idx = 0;
  for (const auto& tri : obj.triangles) {
    indices[idx++] = static_cast<uint16_t>(tri[0]);
    indices[idx++] = static_cast<uint16_t>(tri[1]);
    indices[idx++] = static_cast<uint16_t>(tri[2]);
  }
  ibuf->unlock();
  sub->indexData->indexBuffer = ibuf;
  sub->indexData->indexStart = 0;

  mesh->_setBounds(Ogre::AxisAlignedBox(-1.f, -1.f, -1.f, 1.f, 1.f, 1.f),
                   false);
  mesh->_setBoundingSphereRadius(1.f);
  mesh->load();
}

}  // namespace

void registerObjMesh(const std::string& mesh_name, const display::ObjMesh& obj) {
  if (obj.vertices.empty() || obj.triangles.empty()) {
    return;
  }
  if (Ogre::MeshManager::getSingleton().resourceExists(mesh_name, kRvizMeshGroup)) {
    return;
  }
  Ogre::MeshPtr mesh =
      Ogre::MeshManager::getSingleton().createManual(mesh_name, kRvizMeshGroup);
  UploadObjMesh(mesh, obj);
}

void ensureRvizPrimitiveMeshes() {
  registerObjMesh("rviz_capsule.mesh", display::buildCapsuleMesh(0.5f, 1.f));
  registerObjMesh("rviz_cylinder.mesh", display::buildCylinderMesh(0.5f, 1.f));
  registerObjMesh("rviz_cone.mesh", display::buildConeMesh(0.5f, 1.f));
  registerObjMesh("rviz_sphere.mesh", display::buildSphereMesh(0.5f));
  registerObjMesh("rviz_cube.mesh", display::buildCubeMesh());
}

}  // namespace rendering
}  // namespace autoviz

#endif
