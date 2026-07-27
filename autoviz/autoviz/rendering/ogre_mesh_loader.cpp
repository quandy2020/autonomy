/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/ogre_mesh_loader.hpp"

#ifdef AUTOVIZ_USE_OGRE

#include <functional>

#include <QFileInfo>
#include <QString>

#include <OgreMeshManager.h>
#include <OgreMeshSerializer.h>

#include <glog/logging.h>

#include "autoviz/display/obj_mesh.hpp"
#include "autoviz/display/primitive_mesh.hpp"
#ifdef AUTOVIZ_USE_ASSIMP
#include "autoviz/rendering/mesh_loader_helpers/assimp_loader.hpp"
#endif
#include "autoviz/rendering/mesh_resource.hpp"
#include "autoviz/rendering/ogre_procedural_shape.hpp"

namespace autoviz {
namespace rendering {
namespace {

constexpr char kMeshGroup[] = "rviz_rendering";

std::size_t HashMesh(const display::ObjMesh& mesh) {
  std::size_t hash = mesh.vertices.size();
  for (const QVector3D& v : mesh.vertices) {
    hash ^= std::hash<float>{}(v.x()) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    hash ^= std::hash<float>{}(v.y()) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    hash ^= std::hash<float>{}(v.z()) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
  }
  hash ^= mesh.triangles.size();
  for (const auto& tri : mesh.triangles) {
    hash ^= static_cast<std::size_t>(tri[0]) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    hash ^= static_cast<std::size_t>(tri[1]) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    hash ^= static_cast<std::size_t>(tri[2]) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
  }
  return hash;
}

bool MeshesEqual(const display::ObjMesh& a, const display::ObjMesh& b) {
  if (a.vertices.size() != b.vertices.size() ||
      a.triangles.size() != b.triangles.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.vertices.size(); ++i) {
    if (a.vertices[i] != b.vertices[i]) {
      return false;
    }
  }
  for (std::size_t i = 0; i < a.triangles.size(); ++i) {
    if (a.triangles[i] != b.triangles[i]) {
      return false;
    }
  }
  return true;
}

}  // namespace

void OgreMeshLoader::ensurePrimitiveMeshes() { ensureRvizPrimitiveMeshes(); }

std::string OgreMeshLoader::primitiveMeshName(const display::ObjMesh& mesh) {
  ensurePrimitiveMeshes();
  static const display::ObjMesh kCube = display::buildCubeMesh();
  static const display::ObjMesh kSphere = display::buildSphereMesh(0.5f);
  static const display::ObjMesh kCylinder = display::buildCylinderMesh(0.5f, 1.f);
  static const display::ObjMesh kCone = display::buildConeMesh(0.5f, 1.f);
  if (MeshesEqual(mesh, kCube)) {
    return "rviz_cube.mesh";
  }
  if (MeshesEqual(mesh, kSphere)) {
    return "rviz_sphere.mesh";
  }
  if (MeshesEqual(mesh, kCylinder)) {
    return "rviz_cylinder.mesh";
  }
  if (MeshesEqual(mesh, kCone)) {
    return "rviz_cone.mesh";
  }
  return {};
}

std::string OgreMeshLoader::registerCachedObjMesh(const display::ObjMesh& mesh) {
  if (mesh.vertices.empty() || mesh.triangles.empty()) {
    return {};
  }
  const std::string primitive = primitiveMeshName(mesh);
  if (!primitive.empty()) {
    return primitive;
  }
  const std::string name =
      "AvizMesh/" + std::to_string(HashMesh(mesh)) + ".mesh";
  registerObjMesh(name, mesh);
  return name;
}

std::string OgreMeshLoader::loadAndRegisterMeshFile(const std::string& path) {
  display::ObjMesh mesh;
  if (!display::loadMeshFile(path, &mesh)) {
    return {};
  }
  return registerCachedObjMesh(mesh);
}

Ogre::MeshPtr OgreMeshLoader::loadMeshFromResource(
    const std::string& resource_uri) {
  if (Ogre::MeshManager::getSingleton().resourceExists(resource_uri, kMeshGroup)) {
    return Ogre::MeshManager::getSingleton().getByName(resource_uri, kMeshGroup);
  }

  const QFileInfo model_path(QString::fromStdString(resource_uri));
  const std::string ext = model_path.completeSuffix().toLower().toStdString();
  if (ext == "mesh") {
    const auto resource = MeshResourceResolver::instance().fetch(resource_uri);
    if (resource == nullptr || resource->data.empty()) {
      return {};
    }
    Ogre::MeshSerializer serializer;
    Ogre::DataStreamPtr stream(new Ogre::MemoryDataStream(
        resource->data.data(), resource->data.size()));
    Ogre::MeshPtr mesh =
        Ogre::MeshManager::getSingleton().createManual(resource_uri, kMeshGroup);
    serializer.importMesh(stream, mesh.get());
    stream->close();
    return mesh;
  }

#ifdef AUTOVIZ_USE_ASSIMP
  AssimpLoader loader(&MeshResourceResolver::instance());
  if (!MeshResourceResolver::instance().exists(resource_uri)) {
    throw MeshResourceError("Could not resolve resource URI: " + resource_uri);
  }
  const aiScene* scene = loader.getScene(resource_uri);
  if (scene == nullptr) {
    LOG(ERROR) << "Could not load resource [" << resource_uri
               << "]: " << loader.getErrorMessage();
    return {};
  }
  return loader.meshFromAssimpScene(resource_uri, scene);
#else
  if (ext == "stl" || ext == "obj") {
    const auto path = MeshResourceResolver::instance().resolvePath(resource_uri);
    if (!path.has_value()) {
      throw MeshResourceError("Could not resolve resource URI: " + resource_uri);
    }
    const std::string registered = loadAndRegisterMeshFile(*path);
    if (registered.empty()) {
      throw MeshResourceError("Could not load resource: " + resource_uri);
    }
    return Ogre::MeshManager::getSingleton().getByName(registered, kMeshGroup);
  }
  LOG(ERROR) << "Assimp disabled; cannot load mesh resource: " << resource_uri;
  return {};
#endif
}

}  // namespace rendering
}  // namespace autoviz

#endif
