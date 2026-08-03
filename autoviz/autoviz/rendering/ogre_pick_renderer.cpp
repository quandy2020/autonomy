/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/ogre_pick_renderer.hpp"

#ifdef AUTOVIZ_USE_OGRE

#include <algorithm>
#include <cmath>
#include <cstring>

#include <OgreCamera.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreRenderTexture.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>

#include "autoviz/rendering/ogre_scene_host.hpp"

namespace autoviz {
namespace rendering {
namespace {

constexpr char kRvizResourceGroup[] = "rviz_rendering";

int clampInt(int value, int min_value, int max_value) {
  return std::max(min_value, std::min(value, max_value));
}

common::PickHandle handleFromPixelBox(const Ogre::PixelBox& box) {
  if (box.data == nullptr || box.getWidth() == 0 || box.getHeight() == 0) {
    return common::kInvalidPickHandle;
  }
  const auto* bytes = static_cast<const uint8_t*>(box.data());
  if (box.format == Ogre::PF_R8G8B8A8) {
    return common::pickColorToHandle(bytes[0], bytes[1], bytes[2]);
  }
  if (box.format == Ogre::PF_A8R8G8B8 || box.format == Ogre::PF_X8R8G8B8) {
    return common::pickColorToHandle(bytes[2], bytes[1], bytes[0]);
  }
  return common::kInvalidPickHandle;
}

}  // namespace

OgrePickRenderer::OgrePickRenderer() = default;

OgrePickRenderer::~OgrePickRenderer() { shutdown(); }

void OgrePickRenderer::initialize(Ogre::SceneManager* scene_manager) {
  if (scene_manager == nullptr || scene_manager_ != nullptr) {
    return;
  }
  scene_manager_ = scene_manager;

  fallback_pick_material_ = Ogre::MaterialManager::getSingleton().getByName(
      "rviz/DefaultPickAndDepth", kRvizResourceGroup);
  if (fallback_pick_material_) {
    fallback_pick_material_->load();
    fallback_pick_cull_technique_ =
        fallback_pick_material_->getTechnique("PickCull");
    fallback_black_cull_technique_ =
        fallback_pick_material_->getTechnique("BlackCull");
    fallback_pick_technique_ = fallback_pick_material_->getTechnique("Pick");
    fallback_black_technique_ = fallback_pick_material_->getTechnique("Black");
  }

  static int camera_count = 0;
  const Ogre::String camera_name =
      "AvizPickCamera" + Ogre::StringConverter::toString(camera_count++);
  pick_camera_ = scene_manager_->createCamera(camera_name);
  pick_camera_node_ =
      scene_manager_->getRootSceneNode()->createChildSceneNode(camera_name + "Node");
  pick_camera_node_->attachObject(pick_camera_);

  auto createPickTexture = [](const Ogre::String& name) {
    return Ogre::TextureManager::getSingleton().createManual(
        name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D, 1, 1, 0, Ogre::PF_R8G8B8A8,
        Ogre::TU_STATIC | Ogre::TU_RENDERTARGET);
  };

  pick_texture_ = createPickTexture("AvizPickTexture0");
  pick1_texture_ = createPickTexture("AvizPickTexture1");
  pick_texture_->getBuffer()->getRenderTarget()->setAutoUpdated(false);
  pick1_texture_->getBuffer()->getRenderTarget()->setAutoUpdated(false);
}

void OgrePickRenderer::shutdown() {
  if (scene_manager_ == nullptr) {
    return;
  }
  if (pick_texture_) {
    Ogre::TextureManager::getSingleton().remove(pick_texture_->getName());
    pick_texture_.reset();
  }
  if (pick1_texture_) {
    Ogre::TextureManager::getSingleton().remove(pick1_texture_->getName());
    pick1_texture_.reset();
  }
  if (pick_camera_ != nullptr) {
    if (pick_camera_node_ != nullptr) {
      pick_camera_node_->detachObject(pick_camera_);
      scene_manager_->destroySceneNode(pick_camera_node_);
      pick_camera_node_ = nullptr;
    }
    scene_manager_->destroyCamera(pick_camera_);
    pick_camera_ = nullptr;
  }
  scene_manager_ = nullptr;
}

float OgrePickRenderer::relativeCoordinate(float coordinate, int dimension) {
  return coordinate / static_cast<float>(std::max(1, dimension - 1)) - 0.5f;
}

void OgrePickRenderer::configurePickCamera(Ogre::Viewport* main_viewport, int x1,
                                           int y1, int x2, int y2) const {
  if (main_viewport == nullptr || pick_camera_ == nullptr ||
      pick_camera_node_ == nullptr) {
    return;
  }

  Ogre::Camera* main_camera = main_viewport->getCamera();
  Ogre::Matrix4 proj_matrix = main_camera->getProjectionMatrix();
  Ogre::Matrix4 scale_matrix = Ogre::Matrix4::IDENTITY;
  Ogre::Matrix4 trans_matrix = Ogre::Matrix4::IDENTITY;

  const int width = static_cast<int>(main_viewport->getActualWidth());
  const int height = static_cast<int>(main_viewport->getActualHeight());

  const float x1_rel = relativeCoordinate(static_cast<float>(x1), width);
  const float y1_rel = relativeCoordinate(static_cast<float>(y1), height);
  const float x2_rel = relativeCoordinate(static_cast<float>(x2), width);
  const float y2_rel = relativeCoordinate(static_cast<float>(y2), height);

  scale_matrix[0][0] = 1.f / (x2_rel - x1_rel);
  scale_matrix[1][1] = 1.f / (y2_rel - y1_rel);
  trans_matrix[0][3] -= x1_rel + x2_rel;
  trans_matrix[1][3] += y1_rel + y2_rel;

  pick_camera_->setCustomProjectionMatrix(true,
                                          scale_matrix * trans_matrix * proj_matrix);
  pick_camera_node_->setPosition(main_camera->getDerivedPosition());
  pick_camera_node_->setOrientation(main_camera->getDerivedOrientation());
}

common::PickHandle OgrePickRenderer::renderSchemeAndRead(
    Ogre::Viewport* main_viewport, int x1, int y1, int x2, int y2,
    const std::string& scheme) const {
  if (main_viewport == nullptr || pick_camera_ == nullptr) {
    return common::kInvalidPickHandle;
  }

  Ogre::TexturePtr texture =
      (scheme == "Pick1") ? pick1_texture_ : pick_texture_;
  if (!texture) {
    return common::kInvalidPickHandle;
  }

  configurePickCamera(main_viewport, x1, y1, x2, y2);

  Ogre::HardwarePixelBufferSharedPtr pixel_buffer = texture->getBuffer();
  Ogre::RenderTexture* render_texture = pixel_buffer->getRenderTarget();
  if (render_texture->getNumViewports() == 0) {
    render_texture->removeAllViewports();
    render_texture->addViewport(pick_camera_);
  }

  Ogre::Viewport* render_viewport = render_texture->getViewport(0);
  render_viewport->setClearEveryFrame(true);
  render_viewport->setBackgroundColour(Ogre::ColourValue::Black);
  render_viewport->setOverlaysEnabled(false);
  render_viewport->setMaterialScheme(scheme);
  render_viewport->setVisibilityMask(main_viewport->getVisibilityMask());
  render_viewport->setDimensions(0, 0, 1, 1);

  Ogre::MaterialManager::getSingleton().addListener(
      const_cast<OgrePickRenderer*>(this));
  render_texture->update();
  Ogre::MaterialManager::getSingleton().removeListener(
      const_cast<OgrePickRenderer*>(this));

  Ogre::PixelBox dst_box;
  const auto viewport_w =
      static_cast<unsigned>(render_viewport->getActualWidth());
  const auto viewport_h =
      static_cast<unsigned>(render_viewport->getActualHeight());
  const auto format = pixel_buffer->getFormat();
  const auto size =
      Ogre::PixelUtil::getMemorySize(viewport_w, viewport_h, 1, format);
  auto* data = new uint8_t[size];
  dst_box = Ogre::PixelBox(viewport_w, viewport_h, 1, format, data);
  pixel_buffer->blitToMemory(dst_box);

  const common::PickHandle handle = handleFromPixelBox(dst_box);
  delete[] static_cast<uint8_t*>(dst_box.data());
  return handle;
}

common::PickHandle OgrePickRenderer::pickAt(
    Ogre::Viewport* main_viewport, int pixel_x, int pixel_y, int viewport_width,
    int viewport_height, OgreSceneHost* scene_host,
    common::PickRegistry* pick_registry) const {
  if (main_viewport == nullptr || pick_camera_ == nullptr) {
    return common::kInvalidPickHandle;
  }

  int x1 = clampInt(pixel_x, 0, viewport_width - 2);
  int y1 = clampInt(pixel_y, 0, viewport_height - 2);
  int x2 = x1 + 1;
  int y2 = y1 + 1;

  const common::PickHandle cloud_handle =
      renderSchemeAndRead(main_viewport, x1, y1, x2, y2, "Pick");
  if (cloud_handle == common::kInvalidPickHandle) {
    return common::kInvalidPickHandle;
  }

  if (scene_host == nullptr ||
      !scene_host->isCloudPickHandle(cloud_handle)) {
    return cloud_handle;
  }

  const std::string* display_name =
      scene_host->displayForCloudPickHandle(cloud_handle);
  if (display_name == nullptr) {
    return cloud_handle;
  }

  scene_host->setColorByIndexForAll(true);
  const common::PickHandle point_token =
      renderSchemeAndRead(main_viewport, x1, y1, x2, y2, "Pick1");
  scene_host->setColorByIndexForAll(false);

  if (point_token == common::kInvalidPickHandle || pick_registry == nullptr) {
    return cloud_handle;
  }

  const int point_index = static_cast<int>(point_token) - 1;
  if (point_index < 0) {
    return cloud_handle;
  }

  if (const common::PickHandle resolved =
          pick_registry->lookupByDisplayAndPointIndex(*display_name,
                                                      point_index);
      resolved != common::kInvalidPickHandle) {
    return resolved;
  }
  return cloud_handle;
}

Ogre::Technique* OgrePickRenderer::handleSchemeNotFound(
    unsigned short scheme_index, const Ogre::String& scheme_name,
    Ogre::Material* original_material, unsigned short lod_index,
    const Ogre::Renderable* rend) {
  (void)scheme_index;
  (void)lod_index;

  Ogre::CullingMode culling_mode = Ogre::CULL_CLOCKWISE;
  if (original_material != nullptr) {
    Ogre::Technique* orig_tech = original_material->getTechnique(0);
    if (orig_tech != nullptr && orig_tech->getNumPasses() > 0) {
      culling_mode = orig_tech->getPass(0)->getCullingMode();
    }
  }

  bool has_pick_param = false;
  if (rend != nullptr) {
    has_pick_param =
        rend->getUserObjectBindings().getUserAny("pick_handle").has_value();
  }

  if (culling_mode == Ogre::CULL_CLOCKWISE) {
    if (scheme_name == "Pick") {
      return has_pick_param ? fallback_pick_cull_technique_
                            : fallback_black_cull_technique_;
    }
    if (scheme_name == "Pick1") {
      return fallback_black_cull_technique_;
    }
    return nullptr;
  }

  if (scheme_name == "Pick") {
    return has_pick_param ? fallback_pick_technique_ : fallback_black_technique_;
  }
  if (scheme_name == "Pick1") {
    return fallback_black_technique_;
  }
  return nullptr;
}

}  // namespace rendering
}  // namespace autoviz

#endif
