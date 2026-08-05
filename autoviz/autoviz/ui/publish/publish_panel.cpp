/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/publish/publish_panel.hpp"

#include <QFocusEvent>
#include <QFrame>
#include <QHBoxLayout>
#include <QScrollArea>
#include <QToolButton>
#include <QVBoxLayout>

#include "autoviz/ui/icon_loader.hpp"
#include "autoviz/ui/panel_context_menu.hpp"
#include "autoviz/ui/panel_dock_widget.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"
#include "autoviz/ui/panel_title_tools.hpp"
#include "autoviz/ui/publish/publish_editor_widget.hpp"
#include "autoviz/ui/publish/publish_settings_widget.hpp"

namespace autoviz {
namespace publish_panel {
namespace {

QToolButton* AddTitleToolButton(QWidget* parent, const QIcon& icon,
                                const QString& tip, bool checkable = false) {
  return CreatePlotTitleToolButton(parent, icon, tip, checkable);
}

PublishPanelConfig MergeConfig(const PublishPanelConfig& base,
                               const PublishPanelConfig& editor,
                               const PublishPanelConfig& settings) {
  PublishPanelConfig out = editor;
  out.title = settings.title;
  out.button_label = settings.button_label.isEmpty() ? base.button_label
                                                     : settings.button_label;
  out.button_tooltip = settings.button_tooltip;
  out.button_color = settings.button_color.isValid() ? settings.button_color
                                                     : base.button_color;
  return out;
}

}  // namespace

PublishPanel::PublishPanel(common::VisualizationManager* manager, QWidget* parent)
    : manager_(manager), config_(DefaultPublishPanelConfig()), QWidget(parent) {
  setFocusPolicy(Qt::StrongFocus);
  setAttribute(Qt::WA_StyledBackground, true);
  setStyleSheet(QStringLiteral("PublishPanel { background: palette(window); }"));

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
  settings_widget_ = new PublishSettingsWidget(settings_scroll_);
  settings_scroll_->setWidget(settings_widget_);
  settings_layout->addWidget(settings_scroll_);

  editor_ = new PublishEditorWidget(manager_, this);
  root->addWidget(editor_, 1);

  connect(settings_widget_, &PublishSettingsWidget::configChanged, this, [this]() {
    config_ = MergeConfig(config_, editor_->config(), settings_widget_->config());
    applyConfigToUi();
    emit configChanged();
  });
  connect(editor_, &PublishEditorWidget::configChanged, this,
          &PublishPanel::onEditorConfigChanged);

  syncSettingsWidgetFromConfig();
  applyConfigToUi();
}

void PublishPanel::installTitleBarTools(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  auto* tools = new QWidget(dock);
  tools->setStyleSheet(PlotTitleToolsStyleSheet());
  auto* layout = new QHBoxLayout(tools);
  layout->setContentsMargins(0, 0, 4, 0);
  layout->setSpacing(0);

  layout->addWidget(CreateTitleSeparator(tools));

  auto* split_right = AddTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_split_right.svg")),
      tr("Split right"));
  layout->addWidget(split_right);
  connect(split_right, &QToolButton::clicked, this, [this]() {
    emit panelSplitRequested(Qt::Horizontal);
  });

  auto* split_down = AddTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_split_down.svg")),
      tr("Split down"));
  layout->addWidget(split_down);
  connect(split_down, &QToolButton::clicked, this, [this]() {
    emit panelSplitRequested(Qt::Vertical);
  });

  auto* expand = AddTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_fullscreen.svg")),
      tr("Expand"), true);
  expand_button_ = expand;
  layout->addWidget(expand);
  connect(expand, &QToolButton::clicked, this, [this]() { emit panelExpandRequested(); });

  settings_button_ = AddTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_settings.svg")),
      tr("Settings"), true);
  layout->addWidget(settings_button_);
  connect(settings_button_, &QToolButton::toggled, this, &PublishPanel::onToggleSettings);

  auto* more_button = AddTitleToolButton(
      tools,
      IconLoader::load(QStringLiteral(":/autoviz/icons/plot/plot_more.svg")),
      tr("More"));
  more_button->setPopupMode(QToolButton::InstantPopup);
  PanelContextMenuCallbacks callbacks;
  callbacks.current_object_name = QStringLiteral("PublishDock");
  callbacks.change_panel = [this](const QString& object_name) {
    emit panelChangeRequested(object_name);
  };
  callbacks.split = [this](Qt::Orientation orientation) {
    emit panelSplitRequested(orientation);
  };
  callbacks.expand = [this]() { emit panelExpandRequested(); };
  callbacks.remove = [this]() { emit panelRemoveRequested(); };
  more_button->setMenu(CreatePanelContextMenu(more_button, callbacks));
  layout->addWidget(more_button);

  dock->setTitleBarTools(tools);
}

PublishPanelConfig PublishPanel::config() const {
  return MergeConfig(config_, editor_->config(), settings_widget_->config());
}

void PublishPanel::setConfig(const PublishPanelConfig& config) {
  config_ = config;
  applyConfigToUi();
  syncSettingsWidgetFromConfig();
}

void PublishPanel::cloneConfigFrom(const PublishPanelConfig& config) {
  setConfig(config);
}

void PublishPanel::setSettingsVisible(bool visible) {
  if (settings_container_ != nullptr) {
    settings_container_->setVisible(visible);
  }
  syncSettingsToolState();
}

bool PublishPanel::settingsVisible() const {
  return settings_container_ != nullptr && settings_container_->isVisible();
}

void PublishPanel::setSettingsButtonChecked(bool checked) {
  if (settings_button_ != nullptr) {
    settings_button_->blockSignals(true);
    settings_button_->setChecked(checked);
    settings_button_->blockSignals(false);
  }
}

void PublishPanel::setExpandButtonChecked(bool checked) {
  if (expand_button_ != nullptr) {
    expand_button_->blockSignals(true);
    expand_button_->setChecked(checked);
    expand_button_->blockSignals(false);
  }
}

QWidget* PublishPanel::settingsWidgetForInspector() {
  return SettingsScrollForInspector(settings_scroll_);
}

void PublishPanel::recallSettingsWidget() {
  RecallSettingsScrollToContainer(settings_scroll_, settings_container_);
}

void PublishPanel::refreshSettingsChannels() {
  if (editor_ != nullptr) {
    editor_->refreshChannels();
  }
}

void PublishPanel::focusInEvent(QFocusEvent* event) {
  QWidget::focusInEvent(event);
  emit activated();
}

void PublishPanel::onToggleSettings(bool visible) {
  setSettingsVisible(visible);
  emit settingsToggled(visible);
}

void PublishPanel::onEditorConfigChanged() {
  config_ = MergeConfig(config_, editor_->config(), settings_widget_->config());
  emit configChanged();
}

void PublishPanel::applyConfigToUi() {
  if (editor_ != nullptr) {
    editor_->setConfig(config_);
  }
}

void PublishPanel::syncSettingsWidgetFromConfig() {
  if (settings_widget_ != nullptr) {
    settings_widget_->setConfig(config_);
  }
}

void PublishPanel::syncSettingsToolState() {
  setSettingsButtonChecked(settingsVisible());
}

}  // namespace publish_panel
}  // namespace autoviz
