/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QString>

namespace autoviz {
namespace audio_panel {

struct AudioPanelConfig {
  QString title;
  QString channel;
  QColor waveform_color = QColor(78, 152, 226);
  double volume = 0.8;
  bool mute = false;
  /** Visible time window for live streams (seconds). */
  double window_size_sec = 30.0;
};

AudioPanelConfig DefaultAudioPanelConfig();

}  // namespace audio_panel
}  // namespace autoviz
