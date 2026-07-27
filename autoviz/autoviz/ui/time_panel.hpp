/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QString>

#include <QWidget>

class QCheckBox;
class QComboBox;
class QHBoxLayout;
class QLineEdit;
class QPushButton;
class QLabel;

namespace autoviz {
namespace common {
class VisualizationManager;
}

class TimePanel : public QWidget {
  Q_OBJECT

 public:
  explicit TimePanel(common::VisualizationManager* manager,
                     QWidget* parent = nullptr);

  bool experimental() const;
  int syncMode() const;
  QString syncSource() const;
  void setExperimental(bool enabled);
  void setSyncMode(int mode);
  void setSyncSource(const QString& source);
  void setFpsText(const QString& text);

 signals:
  void resetRequested();
  void layoutChanged();

 private slots:
  void pauseToggled(bool checked);
  void syncModeSelected(int index);
  void syncSourceSelected(int index);
  void experimentalToggled(bool checked);
  void update();

 private:
  QLineEdit* makeTimeLabel();
  void fillTimeLabel(QLineEdit* label, double time);
  void refreshSyncSources();
  void notifyLayoutChanged();

  common::VisualizationManager* manager_ = nullptr;
  QWidget* experimental_widget_ = nullptr;
  QWidget* old_widget_ = nullptr;
  QWidget* bottom_row_ = nullptr;
  QString config_sync_source_;
  QCheckBox* experimental_cb_ = nullptr;
  QPushButton* pause_button_ = nullptr;
  QPushButton* reset_button_ = nullptr;
  QComboBox* sync_source_selector_ = nullptr;
  QComboBox* sync_mode_selector_ = nullptr;
  QLineEdit* sim_time_label_ = nullptr;
  QLineEdit* sim_elapsed_label_ = nullptr;
  QLineEdit* wall_time_label_ = nullptr;
  QLineEdit* wall_elapsed_label_ = nullptr;
  QLabel* fps_label_ = nullptr;
};

}  // namespace autoviz
