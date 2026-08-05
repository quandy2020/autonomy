/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include <atomic>

#include "autoviz/ui/service/service_types.hpp"

class QCheckBox;
class QComboBox;
class QLabel;
class QPlainTextEdit;
class QPushButton;
class QSplitter;
class QTimer;

namespace autoviz {
namespace common {
class VisualizationManager;
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

 private slots:
  void onEditingModeToggled(bool enabled);
  void onServiceChanged(const QString& text);
  void onRequestTypeChanged(const QString& text);
  void onResetTemplate();
  void onFieldEdited();
  void onCallClicked();
  void onRefreshServicesTimer();

 private:
  void applyEditingModeUi();
  void applyButtonStyle();
  void updateCallButtonState();
  void updateStatus(const QString& text, bool is_error = false);
  void maybeFillTemplateForType(const QString& message_type);
  void resolveTypesForService(const QString& service_name);
  void emitConfigChanged();
  void finishCall(const ServiceCallPanelConfig& snapshot, bool ok,
                  const QString& response_text, const QString& error_text);

  common::VisualizationManager* manager_ = nullptr;
  ServiceCallPanelConfig config_;
  bool suppress_template_update_ = false;
  std::atomic<bool> call_in_progress_{false};

  QCheckBox* editing_mode_check_ = nullptr;
  QLabel* status_label_ = nullptr;
  QWidget* editor_body_ = nullptr;
  QComboBox* service_combo_ = nullptr;
  QComboBox* request_type_combo_ = nullptr;
  QComboBox* response_type_combo_ = nullptr;
  QPushButton* reset_template_button_ = nullptr;
  QSplitter* payload_splitter_ = nullptr;
  QPlainTextEdit* request_edit_ = nullptr;
  QPlainTextEdit* response_edit_ = nullptr;
  QPushButton* call_button_ = nullptr;
  QTimer* service_timer_ = nullptr;
};

}  // namespace service_panel
}  // namespace autoviz
