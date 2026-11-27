// Copyright 2026 The Openbot Authors
#include <cstdio>
#include <cstdlib>
#include <map>

#include <QApplication>
#include <QWidget>
#include <Ogre.h>

#include "autoviz/rendering/ogre_render_backend.hpp"
#include "autoviz/rendering/scene_overlay.hpp"
#include "autoviz/rendering/view_controller.hpp"

std::map<Ogre::String, Ogre::ResourceHandle> Textures() {
  std::map<Ogre::String, Ogre::ResourceHandle> result;
  auto resources = Ogre::TextureManager::getSingleton().getResourceIterator();
  while (resources.hasMoreElements()) {
    const auto resource = resources.getNext();
    if (resource->getName().find("AvizDynTex") == 0)
      result[resource->getName()] = resource->getHandle();
  }
  return result;
}

int main(int argc, char** argv) {
  if (!std::getenv("DISPLAY")) return 77;
  QApplication app(argc, argv);
  QWidget host;
  host.setAttribute(Qt::WA_DontShowOnScreen);
  host.resize(320, 240);
  host.show();
  app.processEvents();
  autoviz::rendering::OgreRenderBackend backend(&host);
  if (!backend.initialize()) return 77;
  int failures = 0;
  auto check = [&](bool ok, const char* what) {
    if (!ok) { std::fprintf(stderr, "FAIL: %s\n", what); ++failures; }
  };
  autoviz::rendering::ViewController camera;
  autoviz::rendering::ReferenceGridSettings grid;
  QImage image(2048, 2560, QImage::Format_RGBA8888);
  image.fill(QColor(112, 137, 134));
  bool premultiplied = false;
  auto draw = [&] {
    autoviz::rendering::SceneOverlay overlay;
    overlay.addTexturedQuad({0, 1, 0}, {1, 1, 0}, {1, 0, 0}, {0, 0, 0}, image,
        autoviz::rendering::SceneOverlay::TextureFilterMode::kNearest,
        0, false, true, QVector4D(1, 1, 1, 1), premultiplied);
    check(backend.render(false, grid, &overlay, camera, 4.f/3.f), "frame renders");
  };
  draw();
  const auto original = Textures();
  check(original.size() == 1, "one map texture allocated");
  for (int i = 0; i < 30; ++i) draw();
  check(Textures() == original, "unchanged image reuses the same GPU resource for 30 frames");
  for (bool mode : {true, false}) {
    premultiplied = mode;
    draw();
    auto materials = Ogre::MaterialManager::getSingleton().getResourceIterator();
    bool found = false;
    while (materials.hasMoreElements()) {
      const auto resource = materials.getNext();
      if (resource->getName().find("AvizDynTex") != 0) continue;
      auto material = Ogre::MaterialManager::getSingleton().getByName(resource->getName());
      auto* pass = material->getTechnique(0)->getPass(0);
      check(pass->getSourceBlendFactor() == (mode ? Ogre::SBF_ONE : Ogre::SBF_SOURCE_ALPHA) &&
                pass->getDestBlendFactor() == Ogre::SBF_ONE_MINUS_SOURCE_ALPHA,
            "submap and ordinary map use their respective blend modes");
      found = true;
    }
    check(found && Textures() == original, "changing blend mode reuses the texture");
  }
  image.setPixelColor(0, 0, Qt::black);
  draw();
  check(Textures().size() == 1 && Textures() != original,
        "changed pixels replace the texture and release the old one");
  check(backend.render(false, grid, nullptr, camera, 4.f/3.f), "empty frame renders");
  check(Textures().empty(), "removed display releases its texture");
  draw();
  backend.shutdown();
  check(Textures().empty(), "backend shutdown releases cached textures");
  if (!failures) std::puts("PASS GPU texture reuse, pixel invalidation, removal and shutdown");
  return failures ? 1 : 0;
}
