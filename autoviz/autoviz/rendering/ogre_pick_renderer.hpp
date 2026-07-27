/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#ifdef AUTOVIZ_USE_OGRE

#include <string>

#include <OgreMaterialManager.h>

#include "autoviz/common/pick_handle.hpp"
#include "autoviz/common/pick_registry.hpp"

namespace Ogre {
class Camera;
class SceneManager;
class SceneNode;
class Technique;
class Viewport;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

class OgreSceneHost;

/** rviz_common::SelectionRenderer subset — Ogre RenderTexture pick pass. */
class OgrePickRenderer : public Ogre::MaterialManager::Listener {
 public:
  OgrePickRenderer();
  ~OgrePickRenderer() override;

  OgrePickRenderer(const OgrePickRenderer&) = delete;
  OgrePickRenderer& operator=(const OgrePickRenderer&) = delete;

  void initialize(Ogre::SceneManager* scene_manager);
  void shutdown();

  /** On-demand pick at pixel; uses Pick + Pick1 for OgrePointCloud when needed. */
  common::PickHandle pickAt(Ogre::Viewport* main_viewport, int pixel_x, int pixel_y,
                              int viewport_width, int viewport_height,
                              OgreSceneHost* scene_host,
                              common::PickRegistry* pick_registry) const;

  Ogre::Technique* handleSchemeNotFound(
      unsigned short scheme_index, const Ogre::String& scheme_name,
      Ogre::Material* original_material, unsigned short lod_index,
      const Ogre::Renderable* rend) override;

 private:
  common::PickHandle renderSchemeAndRead(Ogre::Viewport* main_viewport, int x1,
                                         int y1, int x2, int y2,
                                         const std::string& scheme) const;

  void configurePickCamera(Ogre::Viewport* main_viewport, int x1, int y1, int x2,
                           int y2) const;

  static float relativeCoordinate(float coordinate, int dimension);

  Ogre::SceneManager* scene_manager_ = nullptr;
  Ogre::Camera* pick_camera_ = nullptr;
  Ogre::SceneNode* pick_camera_node_ = nullptr;
  Ogre::TexturePtr pick_texture_;
  Ogre::TexturePtr pick1_texture_;

  Ogre::MaterialPtr fallback_pick_material_;
  Ogre::Technique* fallback_pick_cull_technique_ = nullptr;
  Ogre::Technique* fallback_black_cull_technique_ = nullptr;
  Ogre::Technique* fallback_pick_technique_ = nullptr;
  Ogre::Technique* fallback_black_technique_ = nullptr;
};

}  // namespace rendering
}  // namespace autoviz

#endif
