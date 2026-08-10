/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/ogre_render_backend.hpp"

#ifdef AUTOVIZ_USE_OGRE

#include <algorithm>

#include <Ogre.h>

#include <QImage>
#include <QColor>
#include <QOpenGLContext>

#include "autoviz/rendering/ogre_geometry.hpp"
#include "autoviz/rendering/ogre_materials.hpp"
#include "autoviz/rendering/ogre_material_manager.hpp"
#include "autoviz/rendering/ogre_grid.hpp"
#include "autoviz/rendering/ogre_axes.hpp"
#include "autoviz/rendering/gpu_capabilities.hpp"
#include "autoviz/rendering/gpu_depth_pick.hpp"
#include "autoviz/rendering/ogre_scene_host.hpp"
#include "autoviz/rendering/ogre_pick_renderer.hpp"
#include "autoviz/rendering/render_system.hpp"

namespace autoviz {
namespace rendering {
namespace {

Ogre::Vector3 ToOgre(const QVector3D& v) {
  return Ogre::Vector3(v.x(), v.y(), v.z());
}

void UploadLines(Ogre::ManualObject* object,
                 const std::vector<OgreLineVertex>& lines) {
  if (object == nullptr || lines.empty()) {
    return;
  }
  object->begin("Autoviz/OverlayLine", Ogre::RenderOperation::OT_LINE_LIST,
                "AvizOgre");
  for (const auto& line_vertex : lines) {
    object->position(ToOgre(line_vertex.position));
    object->colour(line_vertex.color.x(), line_vertex.color.y(),
                   line_vertex.color.z(), line_vertex.color.w());
  }
  object->end();
}

void UploadFlatTriangles(Ogre::ManualObject* object,
                         const std::vector<OgreLineVertex>& mesh) {
  if (object == nullptr || mesh.empty()) {
    return;
  }
  object->begin("Autoviz/FlatNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
  for (const OgreLineVertex& vertex : mesh) {
    object->position(ToOgre(vertex.position));
    object->colour(vertex.color.x(), vertex.color.y(), vertex.color.z(),
                   vertex.color.w());
  }
  object->end();
}

void UploadPbrTriangles(Ogre::ManualObject* object,
                        const std::vector<OgrePbrVertex>& mesh) {
  if (object == nullptr || mesh.empty()) {
    return;
  }
  object->begin("AvizPBR", Ogre::RenderOperation::OT_TRIANGLE_LIST);
  for (const OgrePbrVertex& vertex : mesh) {
    object->position(ToOgre(vertex.position));
    object->normal(vertex.normal.x(), vertex.normal.y(), vertex.normal.z());
    object->colour(vertex.albedo.x(), vertex.albedo.y(), vertex.albedo.z(),
                   vertex.albedo.w());
    object->textureCoord(vertex.metallic, vertex.roughness);
  }
  object->end();
}

void UploadPointBillboards(Ogre::SceneManager* scene,
                           Ogre::BillboardSet** billboards,
                           const std::vector<OgreLineVertex>& points,
                           float point_size) {
  if (scene == nullptr || billboards == nullptr) {
    return;
  }
  if (*billboards != nullptr) {
    scene->destroyBillboardSet(*billboards);
    *billboards = nullptr;
  }
  if (points.empty()) {
    return;
  }
  *billboards =
      scene->createBillboardSet("AvizPointBillboards", points.size());
  (*billboards)->setMaterialName("AvizPointSprite");
  const float world_size = std::max(0.004f, point_size * 0.012f);
  (*billboards)->setDefaultDimensions(world_size, world_size);
  for (const OgreLineVertex& point : points) {
    Ogre::Billboard* billboard =
        (*billboards)->createBillboard(ToOgre(point.position));
    billboard->setColour(Ogre::ColourValue(
        point.color.x(), point.color.y(), point.color.z(), point.color.w()));
  }
  scene->getRootSceneNode()->attachObject(*billboards);
}

void ClearDynamicTexturedResources(
    Ogre::SceneManager* scene, std::vector<Ogre::ManualObject*>* objects,
    std::vector<Ogre::String>* texture_names,
    std::vector<Ogre::String>* material_names) {
  if (scene == nullptr || objects == nullptr || texture_names == nullptr ||
      material_names == nullptr) {
    return;
  }
  for (Ogre::ManualObject* object : *objects) {
    if (object != nullptr) {
      scene->destroyManualObject(object);
    }
  }
  objects->clear();
  for (const Ogre::String& name : *texture_names) {
    Ogre::TextureManager::getSingleton().remove(name);
  }
  texture_names->clear();
  for (const Ogre::String& name : *material_names) {
    Ogre::MaterialManager::getSingleton().remove(name);
  }
  material_names->clear();
}

void UploadTexturedBatches(
    Ogre::SceneManager* scene,
    const std::vector<SceneOverlay::TexturedBatch>& batches,
    std::vector<Ogre::ManualObject*>* objects,
    std::vector<Ogre::String>* texture_names,
    std::vector<Ogre::String>* material_names) {
  if (scene == nullptr || objects == nullptr || texture_names == nullptr ||
      material_names == nullptr) {
    return;
  }
  int index = 0;
  for (const auto& batch : batches) {
    if (batch.vertices.empty() || batch.image.isNull()) {
      continue;
    }
    const QImage rgba =
        batch.image.format() == QImage::Format_RGBA8888
            ? batch.image
            : batch.image.convertToFormat(QImage::Format_RGBA8888);
    const Ogre::String base_name =
        "AvizDynTex" + Ogre::StringConverter::toString(index++);
    const Ogre::String tex_name = base_name + "Tex";
    Ogre::TexturePtr texture =
        Ogre::TextureManager::getSingleton().createManual(
            tex_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
            Ogre::TEX_TYPE_2D, rgba.width(), rgba.height(), 0,
            Ogre::PF_R8G8B8A8, Ogre::TU_DYNAMIC);
    Ogre::PixelBox pixel_box(
        static_cast<Ogre::uint32>(rgba.width()),
        static_cast<Ogre::uint32>(rgba.height()), 1, Ogre::PF_R8G8B8A8,
        const_cast<uchar*>(rgba.constBits()));
    texture->getBuffer()->blitFromMemory(pixel_box);

    const Ogre::String mat_name = base_name + "Mat";
    Ogre::MaterialPtr material =
        Ogre::MaterialManager::getSingleton().create(
            mat_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material->getTechnique(0)->getPass(0)->createTextureUnitState(tex_name);
    material->getTechnique(0)->getPass(0)->setLightingEnabled(false);
    material->getTechnique(0)->getPass(0)->setSceneBlending(
        Ogre::SBT_TRANSPARENT_ALPHA);
    material->getTechnique(0)->getPass(0)->setVertexColourTracking(
        Ogre::TVC_DIFFUSE);

    Ogre::ManualObject* object = scene->createManualObject(base_name + "MO");
    object->begin(mat_name, Ogre::RenderOperation::OT_TRIANGLE_LIST);
    for (const auto& vertex : batch.vertices) {
      object->position(vertex.position.x(), vertex.position.y(),
                       vertex.position.z());
      object->textureCoord(vertex.uv.x(), vertex.uv.y());
      object->colour(vertex.color.x(), vertex.color.y(), vertex.color.z(),
                     vertex.color.w());
    }
    object->end();
    scene->getRootSceneNode()->attachObject(object);
    objects->push_back(object);
    texture_names->push_back(tex_name);
    material_names->push_back(mat_name);
  }
}

}  // namespace

namespace {

void UpdatePbrLighting(Ogre::Pass* pass, const Ogre::Vector3& light_dir_world) {
  if (pass == nullptr) {
    return;
  }
  Ogre::GpuProgramParametersSharedPtr fp_params =
      pass->getFragmentProgramParameters();
  if (!fp_params) {
    return;
  }
  fp_params->setNamedConstant("uLightDir", light_dir_world);
}

void UploadPbrTexturedBatches(
    Ogre::SceneManager* scene,
    const std::vector<SceneOverlay::PbrTexturedBatch>& batches,
    std::vector<Ogre::ManualObject*>* objects,
    std::vector<Ogre::String>* texture_names,
    std::vector<Ogre::String>* material_names) {
  if (scene == nullptr || objects == nullptr || texture_names == nullptr ||
      material_names == nullptr ||
      !Ogre::MaterialManager::getSingleton().resourceExists("AvizPBRTextured")) {
    return;
  }
  int index = 0;
  for (const auto& batch : batches) {
    if (batch.vertices.empty() || batch.image.isNull()) {
      continue;
    }
    const QImage rgba =
        batch.image.format() == QImage::Format_RGBA8888
            ? batch.image
            : batch.image.convertToFormat(QImage::Format_RGBA8888);
    const Ogre::String base_name =
        "AvizDynPbrTex" + Ogre::StringConverter::toString(index++);
    const Ogre::String tex_name = base_name + "Tex";
    Ogre::TexturePtr texture =
        Ogre::TextureManager::getSingleton().createManual(
            tex_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
            Ogre::TEX_TYPE_2D, rgba.width(), rgba.height(), 0,
            Ogre::PF_R8G8B8A8, Ogre::TU_DYNAMIC);
    Ogre::PixelBox pixel_box(
        static_cast<Ogre::uint32>(rgba.width()),
        static_cast<Ogre::uint32>(rgba.height()), 1, Ogre::PF_R8G8B8A8,
        const_cast<uchar*>(rgba.constBits()));
    texture->getBuffer()->blitFromMemory(pixel_box);

    const Ogre::String mat_name = base_name + "Mat";
    Ogre::MaterialPtr material =
        Ogre::MaterialManager::getSingleton()
            .getByName("AvizPBRTextured")
            ->clone(mat_name);
    Ogre::Pass* pass = material->getTechnique(0)->getPass(0);
    pass->createTextureUnitState(tex_name);
    UpdatePbrLighting(pass, Ogre::Vector3(0.3f, -0.85f, -0.45f).normalisedCopy());

    Ogre::ManualObject* object = scene->createManualObject(base_name + "MO");
    object->begin(mat_name, Ogre::RenderOperation::OT_TRIANGLE_LIST);
    for (const SceneOverlay::PbrTexturedVertex& vertex : batch.vertices) {
      object->position(vertex.position.x(), vertex.position.y(),
                       vertex.position.z());
      object->normal(vertex.normal.x(), vertex.normal.y(), vertex.normal.z());
      object->textureCoord(0, vertex.uv.x(), vertex.uv.y());
      object->colour(vertex.tint.x(), vertex.tint.y(), vertex.tint.z(),
                     vertex.tint.w());
      object->textureCoord(1, vertex.metallic, vertex.roughness);
    }
    object->end();
    scene->getRootSceneNode()->attachObject(object);
    objects->push_back(object);
    texture_names->push_back(tex_name);
    material_names->push_back(mat_name);
  }
}

}  // namespace

struct OgreRenderBackend::Impl {
  Ogre::SceneManager* scene = nullptr;
  Ogre::Camera* camera = nullptr;
  Ogre::SceneNode* camera_node = nullptr;
  Ogre::RenderWindow* render_window = nullptr;
  std::unique_ptr<OgreGrid> reference_grid;
  std::unique_ptr<OgreAxes> reference_axes;
  Ogre::ManualObject* overlay_lines_object = nullptr;
  Ogre::ManualObject* overlay_flat_object = nullptr;
  Ogre::ManualObject* overlay_pbr_object = nullptr;
  Ogre::BillboardSet* point_billboards = nullptr;
  Ogre::Light* directional_light = nullptr;
  std::vector<Ogre::ManualObject*> textured_objects;
  std::vector<Ogre::String> dynamic_texture_names;
  std::vector<Ogre::String> dynamic_material_names;
  Ogre::ColourValue background_color{48.f / 255.f, 48.f / 255.f, 48.f / 255.f};
  GlPickFramebuffer pick_framebuffer;
  OgrePickRenderer ogre_pick_renderer;
  std::unique_ptr<OgreSceneHost> ogre_scene_host_;
  bool initialized = false;
};

OgreRenderBackend::OgreRenderBackend(QWidget* host) : host_(host) {
  impl_ = std::make_unique<Impl>();
}

OgreRenderBackend::~OgreRenderBackend() { shutdown(); }

bool OgreRenderBackend::initialize() {
  if (impl_->initialized || host_ == nullptr) {
    return impl_->initialized;
  }

  RenderSystem* render_system = RenderSystem::instance();
  if (!render_system->ensureInitialized()) {
    return false;
  }
  if (!GpuCapabilities::instance().hasHardwareGpu()) {
    return false;
  }

  Ogre::Root* root = render_system->ogreRoot();
  // Ogre expects logical widget size + contentScalingFactor (DPR), not device
  // pixels twice. See RenderSystem::makeRenderWindow.
  const double dpr = static_cast<double>(host_->devicePixelRatioF());
  impl_->render_window = render_system->makeRenderWindow(
      static_cast<RenderSystem::WindowHandle>(host_->winId()),
      static_cast<unsigned>(std::max(1, host_->width())),
      static_cast<unsigned>(std::max(1, host_->height())), dpr);
  if (impl_->render_window == nullptr) {
    return false;
  }

  impl_->scene = root->createSceneManager();
  render_system->prepareOverlays(impl_->scene);
  EnsureOgreMaterials(impl_->scene);
  impl_->camera = impl_->scene->createCamera("AvizCamera");
  impl_->camera_node =
      impl_->scene->getRootSceneNode()->createChildSceneNode("AvizCameraNode");
  impl_->camera_node->attachObject(impl_->camera);
  impl_->camera_node->setFixedYawAxis(true, Ogre::Vector3::UNIT_Z);
  impl_->camera->setNearClipDistance(0.01f);
  impl_->camera->setFarClipDistance(500.f);
  impl_->camera->setFOVy(Ogre::Degree(45.f));
  impl_->camera->setAutoAspectRatio(true);

  Ogre::Viewport* viewport =
      impl_->render_window->addViewport(impl_->camera);
  viewport->setBackgroundColour(impl_->background_color);

  impl_->reference_grid =
      std::make_unique<OgreGrid>(impl_->scene, nullptr);
  impl_->reference_axes =
      std::make_unique<OgreAxes>(impl_->scene, nullptr);
  impl_->overlay_lines_object = impl_->scene->createManualObject("AvizOverlayLines");
  impl_->overlay_flat_object =
      impl_->scene->createManualObject("AvizOverlayFlat");
  impl_->overlay_pbr_object = impl_->scene->createManualObject("AvizOverlayPbr");
  impl_->scene->getRootSceneNode()->attachObject(impl_->overlay_lines_object);
  impl_->scene->getRootSceneNode()->attachObject(impl_->overlay_flat_object);
  impl_->scene->getRootSceneNode()->attachObject(impl_->overlay_pbr_object);

  impl_->scene->setAmbientLight(Ogre::ColourValue(0.35f, 0.35f, 0.4f));
  impl_->directional_light = impl_->scene->createLight("AvizDirectionalLight");
  impl_->directional_light->setType(Ogre::Light::LT_DIRECTIONAL);
  impl_->directional_light->setDiffuseColour(Ogre::ColourValue(0.95f, 0.95f, 0.9f));
  impl_->directional_light->setSpecularColour(Ogre::ColourValue(0.2f, 0.2f, 0.2f));
  Ogre::SceneNode* light_node =
      impl_->scene->getRootSceneNode()->createChildSceneNode("AvizLightNode");
  light_node->attachObject(impl_->directional_light);
  light_node->setDirection(Ogre::Vector3(0.3f, -0.85f, -0.45f),
                           Ogre::Node::TS_WORLD);

  impl_->ogre_scene_host_ = std::make_unique<OgreSceneHost>(impl_->scene);
  impl_->ogre_pick_renderer.initialize(impl_->scene);
  impl_->initialized = true;
  return true;
}

void OgreRenderBackend::resize(int width, int height) {
  if (!impl_->initialized || impl_->render_window == nullptr) {
    return;
  }
  impl_->render_window->resize(static_cast<unsigned>(std::max(1, width)),
                               static_cast<unsigned>(std::max(1, height)));
  impl_->render_window->windowMovedOrResized();
}

void OgreRenderBackend::render(bool show_grid,
                               const ReferenceGridSettings& grid_settings,
                               SceneOverlay* overlay,
                               const ViewController& view_controller,
                               float aspect_ratio) {
  if (!initialize()) {
    return;
  }

  const QMatrix4x4 view = view_controller.viewMatrix();
  const QMatrix4x4 projection = view_controller.projectionMatrix(aspect_ratio);
  const QMatrix4x4 view_proj = projection * view;
  const QMatrix4x4 inv = view_proj.inverted();

  const QVector3D cam_pos = inv.map(QVector3D(0.f, 0.f, 0.f));
  const QVector3D look_at = inv.map(QVector3D(0.f, 0.f, -1.f));

  impl_->camera_node->setPosition(ToOgre(cam_pos));
  impl_->camera_node->lookAt(ToOgre(look_at), Ogre::Node::TS_WORLD,
                             Ogre::Vector3::NEGATIVE_UNIT_Z);
  impl_->camera->setAutoAspectRatio(true);
  impl_->camera->setNearClipDistance(
      std::max(0.001f, view_controller.nearClipDistance()));
  impl_->camera->setFarClipDistance(500.f);

  if (impl_->reference_grid != nullptr) {
    impl_->reference_grid->setSettings(grid_settings);
    impl_->reference_grid->setVisible(show_grid);
  }
  if (impl_->reference_axes != nullptr) {
    impl_->reference_axes->setLength(grid_settings.axis_length);
    impl_->reference_axes->setVisible(show_grid && grid_settings.show_axes);
  }

  std::vector<OgreLineVertex> overlay_lines;
  std::vector<OgreLineVertex> overlay_flat;
  std::vector<OgreLineVertex> overlay_points;
  if (overlay != nullptr) {
    AppendSceneOverlayFlatTriangles(*overlay, view, &overlay_flat);
    AppendSceneOverlayLines(*overlay, &overlay_lines);
    AppendSceneOverlayPoints(*overlay, &overlay_points);
  }
  impl_->overlay_lines_object->clear();
  impl_->overlay_flat_object->clear();
  impl_->overlay_pbr_object->clear();
  UploadLines(impl_->overlay_lines_object, overlay_lines);
  UploadFlatTriangles(impl_->overlay_flat_object, overlay_flat);
  std::vector<OgrePbrVertex> pbr_mesh;
  if (overlay != nullptr) {
    BuildPbrMeshFromPbrVertices(overlay->pbrVertices(), &pbr_mesh);
  }
  UploadPbrTriangles(impl_->overlay_pbr_object, pbr_mesh);

  const float point_size = overlay != nullptr ? overlay->pointSize() : 4.f;
  UploadPointBillboards(impl_->scene, &impl_->point_billboards, overlay_points,
                        point_size);

  if (Ogre::MaterialManager::getSingleton().resourceExists("AvizPBR")) {
    Ogre::MaterialPtr pbr_material =
        Ogre::MaterialManager::getSingleton().getByName("AvizPBR");
    if (pbr_material) {
      UpdatePbrLighting(pbr_material->getTechnique(0)->getPass(0),
                        Ogre::Vector3(0.3f, -0.85f, -0.45f).normalisedCopy());
    }
  }

  ClearDynamicTexturedResources(
      impl_->scene, &impl_->textured_objects, &impl_->dynamic_texture_names,
      &impl_->dynamic_material_names);
  if (overlay != nullptr) {
    const std::vector<SceneOverlay::TexturedBatch> textured_batches =
        overlay->expandedTexturedBatches(view);
    if (!textured_batches.empty()) {
      UploadTexturedBatches(impl_->scene, textured_batches,
                            &impl_->textured_objects,
                            &impl_->dynamic_texture_names,
                            &impl_->dynamic_material_names);
    }
    const std::vector<SceneOverlay::PbrTexturedBatch>& pbr_textured_batches =
        overlay->pbrTexturedBatches();
    if (!pbr_textured_batches.empty()) {
      UploadPbrTexturedBatches(impl_->scene, pbr_textured_batches,
                               &impl_->textured_objects,
                               &impl_->dynamic_texture_names,
                               &impl_->dynamic_material_names);
    }
  }

  if (impl_->render_window != nullptr && impl_->render_window->getNumViewports() > 0) {
    impl_->render_window->getViewport(0)->setBackgroundColour(
        impl_->background_color);
  }

  RenderSystem::instance()->ogreRoot()->renderOneFrame();

  if (overlay != nullptr && overlay->hasPickGeometry() &&
      QOpenGLContext::currentContext() != nullptr) {
    impl_->pick_framebuffer.renderPickPass(overlay, view, projection,
                                           host_->width(), host_->height());
  }
}

void OgreRenderBackend::setBackgroundColor(const QColor& color) {
  impl_->background_color =
      Ogre::ColourValue(color.redF(), color.greenF(), color.blueF(), 1.f);
}

bool OgreRenderBackend::pickDepthAt(int pixel_x, int pixel_y, int viewport_width,
                                    int viewport_height,
                                    const QMatrix4x4& view,
                                    const QMatrix4x4& projection,
                                    QVector3D* world_out) const {
  if (world_out == nullptr || !impl_->initialized ||
      !GpuCapabilities::instance().hasHardwareGpu()) {
    return false;
  }
  const GpuDepthPickResult pick = pickWorldPointFromDepthBuffer(
      pixel_x, pixel_y, viewport_width, viewport_height, view, projection);
  if (!pick.hit) {
    return false;
  }
  *world_out = pick.position;
  return true;
}

common::PickHandle OgreRenderBackend::pickHandleAt(
    int pixel_x, int pixel_y, int viewport_width, int viewport_height,
    common::PickRegistry* pick_registry) const {
  if (!impl_->initialized) {
    return common::kInvalidPickHandle;
  }

  if (impl_->render_window != nullptr &&
      impl_->render_window->getNumViewports() > 0) {
    Ogre::Viewport* viewport = impl_->render_window->getViewport(0);
    const common::PickHandle ogre_handle = impl_->ogre_pick_renderer.pickAt(
        viewport, pixel_x, pixel_y, viewport_width, viewport_height,
        impl_->ogre_scene_host_.get(), pick_registry);
    if (ogre_handle != common::kInvalidPickHandle) {
      return ogre_handle;
    }
  }

  if (!impl_->pick_framebuffer.valid() ||
      QOpenGLContext::currentContext() == nullptr) {
    return common::kInvalidPickHandle;
  }
  return impl_->pick_framebuffer.readHandleAt(pixel_x, pixel_y, viewport_width,
                                              viewport_height);
}

void OgreRenderBackend::shutdown() {
  if (!impl_->initialized) {
    return;
  }
  impl_->ogre_scene_host_.reset();
  impl_->ogre_pick_renderer.shutdown();
  impl_->pick_framebuffer.destroy();
  ClearDynamicTexturedResources(
      impl_->scene, &impl_->textured_objects, &impl_->dynamic_texture_names,
      &impl_->dynamic_material_names);
  if (impl_->point_billboards != nullptr) {
    impl_->scene->destroyBillboardSet(impl_->point_billboards);
    impl_->point_billboards = nullptr;
  }
  impl_->reference_grid.reset();
  impl_->reference_axes.reset();
  if (impl_->overlay_lines_object != nullptr) {
    impl_->scene->destroyManualObject(impl_->overlay_lines_object);
    impl_->overlay_lines_object = nullptr;
  }
  if (impl_->overlay_flat_object != nullptr) {
    impl_->scene->destroyManualObject(impl_->overlay_flat_object);
    impl_->overlay_flat_object = nullptr;
  }
  if (impl_->overlay_pbr_object != nullptr) {
    impl_->scene->destroyManualObject(impl_->overlay_pbr_object);
    impl_->overlay_pbr_object = nullptr;
  }
  Ogre::Root* root = RenderSystem::instance()->ogreRoot();
  if (impl_->render_window != nullptr && root != nullptr) {
    root->destroyRenderTarget(impl_->render_window);
    impl_->render_window = nullptr;
  }
  if (impl_->scene != nullptr && root != nullptr) {
    root->destroySceneManager(impl_->scene);
    impl_->scene = nullptr;
  }
  impl_->initialized = false;
}

OgreSceneHost* OgreRenderBackend::ogreSceneHost() {
  return impl_ != nullptr ? impl_->ogre_scene_host_.get() : nullptr;
}

const OgreSceneHost* OgreRenderBackend::ogreSceneHost() const {
  return impl_ != nullptr ? impl_->ogre_scene_host_.get() : nullptr;
}

}  // namespace rendering
}  // namespace autoviz

#else

namespace autoviz {
namespace rendering {

struct OgreRenderBackend::Impl {};

OgreRenderBackend::OgreRenderBackend(QWidget* host) : host_(host) {}
OgreRenderBackend::~OgreRenderBackend() = default;
bool OgreRenderBackend::initialize() { return false; }
void OgreRenderBackend::resize(int, int) {}
void OgreRenderBackend::render(bool, const ReferenceGridSettings&, SceneOverlay*,
                               const ViewController&, float) {}
void OgreRenderBackend::setBackgroundColor(const QColor& /*color*/) {}
bool OgreRenderBackend::pickDepthAt(int, int, int, int, const QMatrix4x4&,
                                   const QMatrix4x4&, QVector3D*) const {
  return false;
}
common::PickHandle OgreRenderBackend::pickHandleAt(int, int, int, int,
                                                   common::PickRegistry*) const {
  return common::kInvalidPickHandle;
}
OgreSceneHost* OgreRenderBackend::ogreSceneHost() { return nullptr; }
const OgreSceneHost* OgreRenderBackend::ogreSceneHost() const { return nullptr; }
void OgreRenderBackend::shutdown() {}

}  // namespace rendering
}  // namespace autoviz

#endif
