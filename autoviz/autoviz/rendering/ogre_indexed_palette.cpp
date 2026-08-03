/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/ogre_indexed_palette.hpp"

#ifdef AUTOVIZ_USE_OGRE

#include "autoviz/display/point_cloud_utils.hpp"

#include <Ogre.h>

namespace autoviz {
namespace rendering {
namespace {

constexpr char kPaletteTexture[] = "AvizRainbowPalette";
constexpr char kRvizResourceGroup[] = "rviz_rendering";

}  // namespace

const std::array<QColor, 256>& OgreIndexedPalette::rainbowTable() {
  return display::intensityRainbowTable();
}

void OgreIndexedPalette::ensureRainbowPalette() {
  (void)rainbowTable();

  if (Ogre::TextureManager::getSingleton().resourceExists(kPaletteTexture)) {
    return;
  }

  Ogre::TexturePtr texture =
      Ogre::TextureManager::getSingleton().createManual(
          kPaletteTexture,
          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
          Ogre::TEX_TYPE_1D, 256, 1, 0, Ogre::PF_BYTE_RGBA,
          Ogre::TU_STATIC);
  Ogre::HardwarePixelBufferSharedPtr buffer = texture->getBuffer();
  const Ogre::PixelBox& box =
      buffer->lock(Ogre::Box(0, 0, 256, 1, 1, 1), Ogre::HardwareBuffer::HBL_DISCARD);
  auto* pixels = static_cast<Ogre::uint8*>(box.data());
  for (int i = 0; i < 256; ++i) {
    const QColor color = rainbowTable()[static_cast<std::size_t>(i)];
    pixels[i * 4 + 0] = static_cast<Ogre::uint8>(color.red());
    pixels[i * 4 + 1] = static_cast<Ogre::uint8>(color.green());
    pixels[i * 4 + 2] = static_cast<Ogre::uint8>(color.blue());
    pixels[i * 4 + 3] = 255;
  }
  buffer->unlock();

  if (Ogre::MaterialManager::getSingleton().resourceExists("rviz/Indexed8BitImage",
                                                           kRvizResourceGroup)) {
    Ogre::MaterialPtr material =
        Ogre::MaterialManager::getSingleton().getByName("rviz/Indexed8BitImage",
                                                        kRvizResourceGroup);
    if (material && material->getNumTechniques() > 0) {
      Ogre::Pass* pass = material->getTechnique(0)->getPass(0);
      if (pass->getNumTextureUnitStates() >= 2) {
        pass->getTextureUnitState(1)->setTextureName(kPaletteTexture);
      }
    }
  }
}

uint8_t OgreIndexedPalette::intensityToIndex(float intensity, float min_i,
                                             float max_i) {
  return display::intensityToPaletteIndex(intensity, min_i, max_i);
}

QColor OgreIndexedPalette::colorFromIndex(uint8_t index) {
  return display::colorFromIntensityIndex(index);
}

}  // namespace rendering
}  // namespace autoviz

#endif
