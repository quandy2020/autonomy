/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file service_panel.hpp
 * @brief Dockable Service Call panel shell — editor, settings, title-bar tools.
 *
 * Hosts @ref ServiceEditorWidget and @ref ServiceSettingsWidget inside a
 * panel dock. Shares the Autoviz panel lifecycle signals (split / remove /
 * change / activated) used by VisualizationFrame.
 *
 * @see ServiceEditorWidget
 * @see ServiceSettingsWidget
 * @see ServiceCallPanelConfig
 * @see PanelDockWidget
 */

#pragma once

#include <QWidget>

#include <QPointer>

#include "autoviz/ui/service/service_types.hpp"

class QScrollArea;
class QToolButton;

namespace autoviz {

class PanelDockWidget;
namespace common {
class VisualizationManager;
}
namespace service_panel {

class ServiceEditorWidget;
class ServiceSettingsWidget;

/**
 * @class ServicePanel
 * @brief Top-level Service Call panel widget installed in a @ref PanelDockWidget.
 *
 * ## Layout
 *
 * @code
 * ┌─ dock title bar [⚙ settings] [⤢ expand] … ───────────────┐
 * │ ServiceEditorWidget (service / request / response)        │
 * │ optional settings scroll (timeout / layout / button)      │
 * └───────────────────────────────────────────────────────────┘
 * @endcode
 *
 * ## Data flow
 *
 * - **Out:** editor / settings edits emit @ref configChanged(); focus emits
 *   @ref activated(); title-bar actions emit split / expand / remove / change.
 * - **In:** @ref setConfig() / @ref cloneConfigFrom() restore state;
 *   @ref installTitleBarTools() wires dock chrome.
 *
 * @note Does not own @ref common::VisualizationManager.
 */
class ServicePanel : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs the panel with editor + settings widgets.
   *
   * @param manager Non-owning visualization manager passed to the editor.
   * @param parent Qt parent (dock content host).
   */
  explicit ServicePanel(common::VisualizationManager* manager,
                        QWidget* parent = nullptr);

  /**
   * @brief Installs settings / expand (and related) tools on the dock title bar.
   *
   * @param dock Hosting @ref PanelDockWidget; must outlive these tool buttons
   *        or they are cleared via @c QPointer.
   */
  void installTitleBarTools(PanelDockWidget* dock);

  /**
   * @brief Returns the combined editor + settings configuration.
   *
   * @return Current @ref ServiceCallPanelConfig.
   * @see setConfig()
   */
  ServiceCallPanelConfig config() const;

  /**
   * @brief Replaces configuration and pushes it into editor / settings UI.
   *
   * @param config Full panel config from session or clone.
   * @see config()
   * @see cloneConfigFrom()
   */
  void setConfig(const ServiceCallPanelConfig& config);

  /**
   * @brief Copies config from another panel instance (split / duplicate).
   *
   * @param config Source configuration (typically from the sibling panel).
   */
  void cloneConfigFrom(const ServiceCallPanelConfig& config);

  /**
   * @brief Shows or hides the inline settings area.
   *
   * @param visible When @c true, settings scroll is shown.
   * @see settingsVisible()
   * @see settingsToggled()
   */
  void setSettingsVisible(bool visible);

  /**
   * @brief Whether the settings area is currently visible.
   *
   * @return @c true if settings UI is shown.
   */
  bool settingsVisible() const;

  /**
   * @brief Syncs the title-bar settings tool button checked state.
   *
   * @param checked Desired checked state of @c settings_button_.
   */
  void setSettingsButtonChecked(bool checked);

  /**
   * @brief Syncs the title-bar expand tool button checked state.
   *
   * @param checked Desired checked state of @c expand_button_.
   */
  void setExpandButtonChecked(bool checked);

  /**
   * @brief Returns the settings widget for reparenting into the inspector.
   *
   * @return Non-owning pointer to @c settings_widget_ (may be reparented).
   * @see recallSettingsWidget()
   */
  QWidget* settingsWidgetForInspector();

  /**
   * @brief Reparents the settings widget back into this panel's scroll area.
   *
   * Call after the inspector releases the widget.
   */
  void recallSettingsWidget();

  /**
   * @brief Forwards service-list refresh into the editor.
   *
   * @see ServiceEditorWidget::refreshServices()
   */
  void refreshServices();

 signals:
  /**
   * @brief Emitted when durable panel configuration changes.
   *
   * Listeners should persist @ref config() into session config.
   */
  void configChanged();

  /**
   * @brief Emitted when the panel gains focus / becomes the active panel.
   */
  void activated();

  /**
   * @brief Emitted when settings visibility toggles.
   *
   * @param visible New visibility of the settings area.
   */
  void settingsToggled(bool visible);

  /**
   * @brief Requests splitting this panel in the dock layout.
   *
   * @param orientation Horizontal or vertical split.
   */
  void panelSplitRequested(Qt::Orientation orientation);

  /**
   * @brief Requests expanding this panel to fill the dock area.
   */
  void panelExpandRequested();

  /**
   * @brief Requests removing this panel from the layout.
   */
  void panelRemoveRequested();

  /**
   * @brief Requests replacing this panel with another panel type.
   *
   * @param object_name Target panel type / object name.
   */
  void panelChangeRequested(const QString& object_name);

 protected:
  /**
   * @brief Emits @ref activated() when the panel receives focus.
   *
   * @param event Focus event from Qt.
   */
  void focusInEvent(QFocusEvent* event) override;

 private slots:
  /**
   * @brief Title-bar settings tool toggled.
   *
   * @param visible New settings visibility.
   */
  void onToggleSettings(bool visible);

  /** @brief Editor config changed: merge into @c config_ and re-emit. */
  void onEditorConfigChanged();

  /**
   * @brief Settings form changed: merge chrome / timeout / layout into config
   *        and apply layout orientation to the editor.
   */
  void onSettingsConfigChanged();

 private:
  /** @brief Pushes @c config_ settings fields into @c settings_widget_. */
  void syncSettingsWidgetFromConfig();

  /** @brief Updates title-bar tool checked states from visibility flags. */
  void syncSettingsToolState();

  /** @brief Applies @c config_ to editor and settings widgets. */
  void applyConfigToUi();

  /** Non-owning visualization manager. */
  common::VisualizationManager* manager_ = nullptr;

  /** Canonical panel configuration. */
  ServiceCallPanelConfig config_;

  /** Main request / response editor. */
  ServiceEditorWidget* editor_ = nullptr;

  /** Timeout / layout / button chrome settings form. */
  ServiceSettingsWidget* settings_widget_ = nullptr;

  /** Scroll host for settings when shown inline. */
  QScrollArea* settings_scroll_ = nullptr;

  /** Container reparented with settings for inspector hand-off. */
  QWidget* settings_container_ = nullptr;

  /** Dock title-bar settings tool (weak). */
  QPointer<QToolButton> settings_button_;

  /** Dock title-bar expand tool (weak). */
  QPointer<QToolButton> expand_button_;
};

}  // namespace service_panel
}  // namespace autoviz
