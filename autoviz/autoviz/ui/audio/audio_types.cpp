/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/audio/audio_types.hpp"

namespace autoviz {
namespace audio_panel {

AudioPanelConfig DefaultAudioPanelConfig() {
  AudioPanelConfig config;
  config.volume = 0.8;
  config.window_size_sec = 30.0;
  return config;
}

}  // namespace audio_panel
}  // namespace autoviz
