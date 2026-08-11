/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include <atomic>

class QKeyEvent;

#include "autoviz/ui/service/service_types.hpp"

class QCheckBox;
class QComboBox;
class QGroupBox;
class QLabel;
class QPlainTextEdit;
class QPushButton;
class QSplitter;
class QTimer;

namespace autoviz {
namespace common {
class VisualizationManager;
}

namespace publish_panel {
class PublishFieldTreeWidget;
}

namespace service_panel {

class ServiceEditorWidget : public QWidget {
  Q_OBJECT

 public:
  explicit ServiceEditorWidget(common::VisualizationManager* manager,
                               QWidget* parent = nullptr);

  ServiceCallPanelConfig config() const;
  void setConfig(const ServiceCallPanelConfig& config);
  void refreshServices();
  void applyLayoutOrientation(bool vertical);

 signals:
  void configChanged();
  void callFinished();

 protected:
  void keyPressEvent(QKeyEvent* event) override;

 private slots:
  void onEditingModeToggled(bool enabled);
  void onServiceChanged(const QString& text);
  void onRequestTypeChanged(const QString& text);
  void onResetTemplate();
  void onFieldEdited();
  void onRequestTreeEdited();
  void onCallClicked();
  void onRefreshServices();

 private:
  QPlainTextEdit* makeJsonEditor(QWidget* parent, const QString& placeholder,
                                 bool read_only = false);
  void applyEditingModeUi();
  void applyButtonStyle();
  void updateCallButtonState();
  void updateStatus(const QString& text, bool is_error = false);
  void maybeFillTemplateForType(const QString& message_type);
  void resolveTypesForService(const QString& service_name);
  void syncRequestJsonFromTree();
  void syncRequestTreeFromJson();
  void syncResponseTreeFromJson(const QString& json);
  void applyRequestServiceRootLabel();
  void emitConfigChanged();
  void finishCall(const ServiceCallPanelConfig& snapshot, bool ok,
                  const QString& response_text, const QString& error_text);

  common::VisualizationManager* manager_ = nullptr;
  ServiceCallPanelConfig config_;
  bool suppress_template_update_ = false;
  bool request_fields_dirty_ = false;
  std::atomic<bool> call_in_progress_{false};

  QWidget* rqt_top_bar_ = nullptr;
  QCheckBox* editing_mode_check_ = nullptr;
  QLabel* status_label_ = nullptr;
  QWidget* advanced_body_ = nullptr;
  QComboBox* service_combo_ = nullptr;
  QComboBox* request_type_combo_ = nullptr;
  QComboBox* response_type_combo_ = nullptr;
  QPushButton* refresh_services_button_ = nullptr;
  QPushButton* reset_template_button_ = nullptr;
  QSplitter* payload_splitter_ = nullptr;
  QGroupBox* request_group_ = nullptr;
  QGroupBox* response_group_ = nullptr;
  publish_panel::PublishFieldTreeWidget* request_tree_ = nullptr;
  publish_panel::PublishFieldTreeWidget* response_tree_ = nullptr;
  QPlainTextEdit* request_edit_ = nullptr;
  QPlainTextEdit* response_edit_ = nullptr;
  QPushButton* call_button_ = nullptr;
  QTimer* service_timer_ = nullptr;
};

}  // namespace service_panel
}  // namespace autoviz
