/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/grpc/grpc_panel.hpp"

#include <QFocusEvent>
#include <QFrame>
#include <QHBoxLayout>
#include <QScrollArea>
#include <QToolButton>
#include <QVBoxLayout>

#include "autoviz/ui/grpc/grpc_editor_widget.hpp"
#include "autoviz/ui/grpc/grpc_settings_widget.hpp"
#include "autoviz/ui/panel_context_menu.hpp"
#include "autoviz/ui/panel_dock_widget.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"
#include "autoviz/ui/panel_title_tools.hpp"

namespace autoviz {
namespace grpc_panel {
namespace {

GrpcPanelPersistConfig MergeConfig(const GrpcPanelPersistConfig& base,
                                   const GrpcPanelPersistConfig& editor,
                                   const GrpcPanelPersistConfig& settings) {
  GrpcPanelPersistConfig out = editor;
  out.title = settings.title;
  out.timeout_ms = settings.timeout_ms > 0 ? settings.timeout_ms : base.timeout_ms;
  out.verify_cert = settings.verify_cert;
  out.ssl_override = settings.ssl_override;
  out.include_defaults = settings.include_defaults;
  out.max_response_mb =
      settings.max_response_mb > 0 ? settings.max_response_mb : base.max_response_mb;
  return out;
}

}  // namespace

GrpcPanel::GrpcPanel(QWidget* parent)
    : config_(DefaultGrpcPanelPersistConfig()), QWidget(parent) {
  setFocusPolicy(Qt::StrongFocus);
  ApplyPanelShell(this);

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);

  settings_container_ = new QWidget(this);
  settings_container_->hide();
  auto* settings_layout = new QVBoxLayout(settings_container_);
  settings_layout->setContentsMargins(0, 0, 0, 0);
  settings_scroll_ = new QScrollArea(settings_container_);
  settings_scroll_->setWidgetResizable(true);
  settings_scroll_->setFrameShape(QFrame::NoFrame);
  settings_scroll_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  settings_widget_ = new GrpcSettingsWidget(settings_scroll_);
  settings_scroll_->setWidget(settings_widget_);
  settings_layout->addWidget(settings_scroll_);

  editor_ = new GrpcEditorWidget(this);
  root->addWidget(settings_container_);
  root->addWidget(editor_, 1);

  connect(settings_widget_, &GrpcSettingsWidget::configChanged, this,
          &GrpcPanel::onSettingsConfigChanged);
  connect(editor_, &GrpcEditorWidget::configChanged, this,
          &GrpcPanel::onEditorConfigChanged);

  syncSettingsWidgetFromConfig();
  applyConfigToUi();
}

void GrpcPanel::installTitleBarTools(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  PanelContextMenuCallbacks callbacks;
  callbacks.current_object_name = QStringLiteral("GrpcDock");
  callbacks.change_panel = [this](const QString& object_name) {
    emit panelChangeRequested(object_name);
  };
  callbacks.split = [this](Qt::Orientation orientation) {
    emit panelSplitRequested(orientation);
  };
  callbacks.expand = [this]() { emit panelExpandRequested(); };
  callbacks.remove = [this]() { emit panelRemoveRequested(); };

  PanelTitleBarOptions options;
  options.show_settings = true;
  options.settings_checked = settingsVisible();
  options.on_settings_toggled = [this](bool visible) { onToggleSettings(visible); };
  options.on_expand = [this]() { emit panelExpandRequested(); };

  const PanelTitleBarTools tools =
      CreateRvizPanelTitleBarTools(dock, callbacks, options);
  settings_button_ = tools.settings_button;
  expand_button_ = tools.expand_button;
  dock->setTitleBarTools(tools.widget);
}

GrpcPanelPersistConfig GrpcPanel::config() const {
  return MergeConfig(config_, editor_->config(), settings_widget_->config());
}

void GrpcPanel::setConfig(const GrpcPanelPersistConfig& config) {
  config_ = config;
  applyConfigToUi();
  syncSettingsWidgetFromConfig();
}

void GrpcPanel::cloneConfigFrom(const GrpcPanelPersistConfig& config) {
  setConfig(config);
}

void GrpcPanel::setSettingsVisible(bool visible) {
  if (settings_container_ != nullptr) {
    settings_container_->setVisible(visible);
  }
  syncSettingsToolState();
}

bool GrpcPanel::settingsVisible() const {
  return settings_container_ != nullptr && settings_container_->isVisible();
}

void GrpcPanel::setSettingsButtonChecked(bool checked) {
  if (settings_button_ != nullptr) {
    settings_button_->blockSignals(true);
    settings_button_->setChecked(checked);
    settings_button_->blockSignals(false);
  }
}

void GrpcPanel::setExpandButtonChecked(bool checked) {
  if (expand_button_ != nullptr) {
    expand_button_->blockSignals(true);
    expand_button_->setChecked(checked);
    expand_button_->blockSignals(false);
  }
}

QWidget* GrpcPanel::settingsWidgetForInspector() {
  return SettingsScrollForInspector(settings_scroll_);
}

void GrpcPanel::recallSettingsWidget() {
  RecallSettingsScrollToContainer(settings_scroll_, settings_container_);
}

void GrpcPanel::focusInEvent(QFocusEvent* event) {
  QWidget::focusInEvent(event);
  emit activated();
}

void GrpcPanel::onToggleSettings(bool visible) {
  setSettingsVisible(visible);
  emit settingsToggled(visible);
}

void GrpcPanel::onEditorConfigChanged() {
  config_ = MergeConfig(config_, editor_->config(), settings_widget_->config());
  emit configChanged();
}

void GrpcPanel::onSettingsConfigChanged() {
  config_ = MergeConfig(config_, editor_->config(), settings_widget_->config());
  applyConfigToUi();
  emit configChanged();
}

void GrpcPanel::applyConfigToUi() {
  if (editor_ != nullptr) {
    editor_->setConfig(config_);
  }
}

void GrpcPanel::syncSettingsWidgetFromConfig() {
  if (settings_widget_ != nullptr) {
    settings_widget_->setConfig(config_);
  }
}

void GrpcPanel::syncSettingsToolState() {
  setSettingsButtonChecked(settingsVisible());
}

}  // namespace grpc_panel
}  // namespace autoviz
