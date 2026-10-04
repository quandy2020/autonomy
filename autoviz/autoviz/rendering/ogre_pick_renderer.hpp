/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file ogre_pick_renderer.hpp
 * @brief Ogre RenderTexture pick pass (rviz_common::SelectionRenderer subset).
 *
 * Implements @c Ogre::MaterialManager::Listener to substitute pick/black
 * techniques for the @c Pick / @c Pick1 schemes, then reads the encoded handle
 * from an offscreen texture — used for @ref OgrePointCloud and Entity picking.
 *
 * @see OgreRenderBackend
 * @see OgreSceneHost
 * @see common::PickRegistry
 */

#pragma once

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

/**
 * @class OgrePickRenderer
 * @brief On-demand GPU pick via Ogre material schemes.
 *
 * Non-copyable. Call @ref initialize() after the scene manager exists and
 * @ref shutdown() before destroying Ogre Root.
 */
class OgrePickRenderer : public Ogre::MaterialManager::Listener {
 public:
  /** @brief Constructs an uninitialized pick renderer. */
  OgrePickRenderer();

  /** @brief Calls @ref shutdown() if still initialized. */
  ~OgrePickRenderer() override;

  OgrePickRenderer(const OgrePickRenderer&) = delete;
  OgrePickRenderer& operator=(const OgrePickRenderer&) = delete;

  /**
   * @brief Creates pick cameras / textures and registers as material listener.
   * @param scene_manager Non-null scene manager.
   */
  void initialize(Ogre::SceneManager* scene_manager);

  /**
   * @brief Releases pick resources and unregisters the material listener.
   */
  void shutdown();

  /**
   * @brief Picks at a pixel using Pick (+ Pick1 for point clouds when needed).
   *
   * @param main_viewport Active viewport to mirror camera from.
   * @param pixel_x Pixel X in viewport coordinates.
   * @param pixel_y Pixel Y.
   * @param viewport_width Viewport width.
   * @param viewport_height Viewport height.
   * @param scene_host Optional host for cloud pick-handle lookup; may be null.
   * @param pick_registry Optional registry; may be null.
   * @return Decoded @ref common::PickHandle or invalid handle.
   */
  common::PickHandle pickAt(Ogre::Viewport* main_viewport, int pixel_x, int pixel_y,
                              int viewport_width, int viewport_height,
                              OgreSceneHost* scene_host,
                              common::PickRegistry* pick_registry) const;

  /**
   * @brief MaterialManager callback: supplies fallback pick/black techniques.
   *
   * @param scheme_index Scheme index from Ogre.
   * @param scheme_name Scheme name (@c Pick, @c Pick1, …).
   * @param original_material Material that lacked the scheme.
   * @param lod_index LOD index.
   * @param rend Renderable requesting the technique.
   * @return Fallback technique, or @c nullptr if unhandled.
   */
  Ogre::Technique* handleSchemeNotFound(
      unsigned short scheme_index, const Ogre::String& scheme_name,
      Ogre::Material* original_material, unsigned short lod_index,
      const Ogre::Renderable* rend) override;

 private:
  /**
   * @brief Renders @p scheme into the pick texture and reads a pixel box.
   */
  common::PickHandle renderSchemeAndRead(Ogre::Viewport* main_viewport, int x1,
                                         int y1, int x2, int y2,
                                         const std::string& scheme) const;

  /**
   * @brief Configures the pick camera frustum to cover the pixel box.
   */
  void configurePickCamera(Ogre::Viewport* main_viewport, int x1, int y1, int x2,
                           int y2) const;

  /**
   * @brief Maps a pixel coordinate into \[0,1\] relative to a dimension.
   */
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

