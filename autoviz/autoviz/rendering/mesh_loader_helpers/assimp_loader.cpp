/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/mesh_loader_helpers/assimp_loader.hpp"

#ifdef AUTOVIZ_USE_ASSIMP

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <OgreHardwareBufferManager.h>
#include <OgreMeshManager.h>
#include <OgreSubMesh.h>

#include "autoviz/rendering/mesh_resource.hpp"

namespace autoviz {
namespace rendering {
namespace {

constexpr char kMeshGroup[] = "aviz_rendering";

Assimp::Importer* AsImporter(void* ptr) {
  return static_cast<Assimp::Importer*>(ptr);
}

}  // namespace

AssimpLoader::AssimpLoader(MeshResourceResolver* resolver)
    : resolver_(resolver) {}

const aiScene* AssimpLoader::getScene(const std::string& resource_uri) {
  error_.clear();
  if (importer_ == nullptr) {
    importer_ = new Assimp::Importer();
  }
  auto* importer = AsImporter(importer_);
  importer->FreeScene();

  if (resolver_ == nullptr || !resolver_->exists(resource_uri)) {
    error_ = "Could not resolve resource URI: " + resource_uri;
    return nullptr;
  }

  const auto resource = resolver_->fetch(resource_uri);
  if (resource == nullptr || resource->data.empty()) {
    error_ = "Empty resource: " + resource_uri;
    return nullptr;
  }

  const aiScene* scene = importer->ReadFileFromMemory(
      resource->data.data(), resource->data.size(),
      aiProcess_Triangulate | aiProcess_GenNormals | aiProcess_JoinIdenticalVertices |
          aiProcess_SortByPType,
      nullptr);
  if (scene == nullptr) {
    error_ = importer->GetErrorString();
  }
  return scene;
}

Ogre::MeshPtr AssimpLoader::meshFromAssimpScene(const std::string& name,
                                                const aiScene* scene) {
  if (scene == nullptr || scene->mNumMeshes == 0) {
    return {};
  }

  if (Ogre::MeshManager::getSingleton().resourceExists(name, kMeshGroup)) {
    return Ogre::MeshManager::getSingleton().getByName(name, kMeshGroup);
  }

  Ogre::MeshPtr mesh =
      Ogre::MeshManager::getSingleton().createManual(name, kMeshGroup);

  for (unsigned int mi = 0; mi < scene->mNumMeshes; ++mi) {
    const aiMesh* src = scene->mMeshes[mi];
    if (src == nullptr || src->mNumVertices == 0 || src->mNumFaces == 0) {
      continue;
    }

    Ogre::SubMesh* sub = mesh->createSubMesh();
    sub->useSharedVertices = false;
    sub->vertexData = OGRE_NEW Ogre::VertexData();
    sub->vertexData->vertexCount = src->mNumVertices;
    sub->vertexData->vertexStart = 0;

    Ogre::VertexDeclaration* decl = sub->vertexData->vertexDeclaration;
    size_t offset = 0;
    decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
    const bool has_normals = src->HasNormals();
    if (has_normals) {
      decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
      offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
    }

    Ogre::HardwareVertexBufferSharedPtr vbuf =
        Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
            decl->getVertexSize(0), src->mNumVertices,
            Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
    float* dst = static_cast<float*>(vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));
    for (unsigned int i = 0; i < src->mNumVertices; ++i) {
      *dst++ = src->mVertices[i].x;
      *dst++ = src->mVertices[i].y;
      *dst++ = src->mVertices[i].z;
      if (has_normals) {
        *dst++ = src->mNormals[i].x;
        *dst++ = src->mNormals[i].y;
        *dst++ = src->mNormals[i].z;
      }
    }
    vbuf->unlock();
    sub->vertexData->vertexBufferBinding->setBinding(0, vbuf);

    unsigned int index_count = 0;
    for (unsigned int f = 0; f < src->mNumFaces; ++f) {
      if (src->mFaces[f].mNumIndices == 3) {
        index_count += 3;
      }
    }
    sub->indexData->indexCount = index_count;
    sub->indexData->indexStart = 0;
    Ogre::HardwareIndexBufferSharedPtr ibuf =
        Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
            Ogre::HardwareIndexBuffer::IT_16BIT, index_count,
            Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
    auto* idst = static_cast<uint16_t*>(ibuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));
    for (unsigned int f = 0; f < src->mNumFaces; ++f) {
      const aiFace& face = src->mFaces[f];
      if (face.mNumIndices != 3) {
        continue;
      }
      *idst++ = static_cast<uint16_t>(face.mIndices[0]);
      *idst++ = static_cast<uint16_t>(face.mIndices[1]);
      *idst++ = static_cast<uint16_t>(face.mIndices[2]);
    }
    ibuf->unlock();
    sub->indexData->indexBuffer = ibuf;
  }

  mesh->_setBounds(Ogre::AxisAlignedBox(-1, -1, -1, 1, 1, 1));
  mesh->_setBoundingSphereRadius(1.f);
  mesh->load();
  return mesh;
}

}  // namespace rendering
}  // namespace autoviz

#endif  // AUTOVIZ_USE_ASSIMP
