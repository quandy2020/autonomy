/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file assimp_loader.hpp
 * @brief Assimp → Ogre::Mesh bridge used by @ref OgreMeshLoader.
 */

#pragma once

#ifdef AUTOVIZ_USE_ASSIMP

#include <string>

#include <OgreMesh.h>

struct aiScene;

namespace autoviz {
namespace rendering {

class MeshResourceResolver;

class AssimpLoader {
 public:
  explicit AssimpLoader(MeshResourceResolver* resolver);

  const aiScene* getScene(const std::string& resource_uri);
  const std::string& getErrorMessage() const { return error_; }

  Ogre::MeshPtr meshFromAssimpScene(const std::string& name,
                                    const aiScene* scene);

 private:
  MeshResourceResolver* resolver_ = nullptr;
  std::string error_;
  // Owns Assimp importer memory for the last getScene() call.
  void* importer_ = nullptr;
};

}  // namespace rendering
}  // namespace autoviz

#endif  // AUTOVIZ_USE_ASSIMP
