/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include "autoviz/ui/audio/audio_types.hpp"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLineEdit;
class QPushButton;
class QSlider;

namespace autoviz {
namespace common {
class VisualizationManager;
}
namespace audio_panel {

class AudioSettingsWidget : public QWidget {
  Q_OBJECT

 public:
  explicit AudioSettingsWidget(common::VisualizationManager* manager,
                               QWidget* parent = nullptr);

  AudioPanelConfig config() const;
  void setConfig(const AudioPanelConfig& config);
  void refreshChannels();

 signals:
  void configChanged();

 private slots:
  void emitConfigChanged();

 private:
  void applyToUi(const AudioPanelConfig& config);
  void syncFromUi();

  common::VisualizationManager* manager_ = nullptr;
  AudioPanelConfig config_;
  QLineEdit* title_edit_ = nullptr;
  QComboBox* channel_combo_ = nullptr;
  QPushButton* color_button_ = nullptr;
  QSlider* volume_slider_ = nullptr;
  QCheckBox* mute_check_ = nullptr;
  QDoubleSpinBox* window_spin_ = nullptr;
  bool suppress_signals_ = false;
};

}  // namespace audio_panel
}  // namespace autoviz
