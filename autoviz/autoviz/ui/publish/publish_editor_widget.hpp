/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include "autoviz/ui/publish/publish_types.hpp"

class QCheckBox;
class QComboBox;
class QLabel;
class QPlainTextEdit;
class QPushButton;
class QScrollArea;

namespace autoviz {
namespace common {
class VisualizationManager;
}

namespace publish_panel {

class PublishEditorWidget : public QWidget {
  Q_OBJECT

 public:
  explicit PublishEditorWidget(common::VisualizationManager* manager,
                               QWidget* parent = nullptr);

  PublishPanelConfig config() const;
  void setConfig(const PublishPanelConfig& config);
  void refreshChannels();

 signals:
  void configChanged();
  void publishRequested();

 private slots:
  void onEditingModeToggled(bool enabled);
  void onChannelChanged(const QString& text);
  void onMessageTypeChanged(const QString& text);
  void onResetTemplate();
  void onPublishClicked();
  void onFieldEdited();

 private:
  void rebuildMessageTypeList();
  void applyEditingModeUi();
  void updatePublishButtonState();
  void updateStatus(const QString& text, bool is_error = false);
  QString messageTypeForChannel(const QString& channel) const;
  void maybeFillTemplateForType(const QString& message_type);
  void emitConfigChanged();

  common::VisualizationManager* manager_ = nullptr;
  PublishPanelConfig config_;
  bool suppress_template_update_ = false;

  QCheckBox* editing_mode_check_ = nullptr;
  QLabel* status_label_ = nullptr;
  QWidget* editor_body_ = nullptr;
  QComboBox* channel_combo_ = nullptr;
  QComboBox* message_type_combo_ = nullptr;
  QPushButton* reset_template_button_ = nullptr;
  QPlainTextEdit* message_edit_ = nullptr;
  QPushButton* publish_button_ = nullptr;
};

}  // namespace publish_panel
}  // namespace autoviz
