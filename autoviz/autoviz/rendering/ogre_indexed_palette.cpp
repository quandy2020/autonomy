/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/ogre_indexed_palette.hpp"

#include "autoviz/plugins/displays/pointcloud/point_cloud_utils.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <vector>

#include <Ogre.h>

namespace autoviz {
namespace rendering {
namespace {

constexpr char kPaletteTexture[] = "AvizRainbowPalette";
constexpr char kAvizResourceGroup[] = "aviz_rendering";

std::atomic<bool> g_palette_ready{false};
std::atomic<bool> g_palette_failed{false};

}  // namespace

const std::array<QColor, 256>& OgreIndexedPalette::rainbowTable() {
  return display::intensityRainbowTable();
}

void OgreIndexedPalette::ensureRainbowPalette() {
  (void)rainbowTable();
  if (g_palette_ready.load(std::memory_order_acquire) ||
      g_palette_failed.load(std::memory_order_acquire)) {
    return;
  }

  try {
    Ogre::TextureManager* textures = Ogre::TextureManager::getSingletonPtr();
    if (textures == nullptr) {
      g_palette_failed.store(true, std::memory_order_release);
      return;
    }

    if (textures->resourceExists(kPaletteTexture)) {
      g_palette_ready.store(true, std::memory_order_release);
      return;
    }

    // Use 256×1 TEX_TYPE_2D + blitFromMemory (same pattern as
    // CreatePointDiscTexture). TEX_TYPE_1D + lock(Box(...)) has returned
    // null PixelBox.data on NVIDIA/GL and SEGV'd writing the rainbow table
    // (wrong 6-arg Box layout was also used historically).
    std::vector<std::uint8_t> pixels(256 * 4);
    const auto& table = rainbowTable();
    for (int i = 0; i < 256; ++i) {
      const QColor color = table[static_cast<std::size_t>(i)];
      pixels[static_cast<std::size_t>(i) * 4 + 0] =
          static_cast<std::uint8_t>(color.red());
      pixels[static_cast<std::size_t>(i) * 4 + 1] =
          static_cast<std::uint8_t>(color.green());
      pixels[static_cast<std::size_t>(i) * 4 + 2] =
          static_cast<std::uint8_t>(color.blue());
      pixels[static_cast<std::size_t>(i) * 4 + 3] = 255;
    }

    Ogre::TexturePtr texture = textures->createManual(
        kPaletteTexture,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D, 256, 1, 0, Ogre::PF_BYTE_RGBA, Ogre::TU_STATIC);
    if (!texture) {
      g_palette_failed.store(true, std::memory_order_release);
      return;
    }
    Ogre::HardwarePixelBufferSharedPtr buffer = texture->getBuffer();
    if (!buffer) {
      g_palette_failed.store(true, std::memory_order_release);
      return;
    }
    Ogre::PixelBox box(256, 1, 1, Ogre::PF_BYTE_RGBA, pixels.data());
    buffer->blitFromMemory(box);

    if (Ogre::MaterialManager::getSingleton().resourceExists(
            "aviz/Indexed8BitImage", kAvizResourceGroup)) {
      Ogre::MaterialPtr material =
          Ogre::MaterialManager::getSingleton().getByName(
              "aviz/Indexed8BitImage", kAvizResourceGroup);
      if (material && material->getNumTechniques() > 0) {
        Ogre::Pass* pass = material->getTechnique(0)->getPass(0);
        if (pass != nullptr && pass->getNumTextureUnitStates() >= 2) {
          pass->getTextureUnitState(1)->setTextureName(kPaletteTexture);
        }
      }
    }

    g_palette_ready.store(true, std::memory_order_release);
  } catch (...) {
    // Palette is optional for FlatSquares vertex colors; never abort the frame.
    g_palette_failed.store(true, std::memory_order_release);
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
