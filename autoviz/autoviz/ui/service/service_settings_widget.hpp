/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file service_settings_widget.hpp
 * @brief Settings form for Service Call panel chrome and call options.
 *
 * Edits title, timeout, request/response layout orientation, and Call-button
 * appearance. Payload editing lives in @ref ServiceEditorWidget.
 *
 * @see ServicePanel
 * @see ServiceCallPanelConfig
 */

#pragma once

#include <QWidget>

#include "autoviz/ui/service/service_types.hpp"

class QCheckBox;
class QComboBox;
class QLineEdit;
class QPushButton;
class QSpinBox;

namespace autoviz {
namespace service_panel {

/**
 * @class ServiceSettingsWidget
 * @brief Edits Service Call panel title, timeout, layout, and button chrome.
 *
 * Emits @ref configChanged() on any field edit so @ref ServicePanel can sync
 * and persist configuration.
 *
 * @note Does not own the panel; @ref ServicePanel keeps the canonical config.
 */
class ServiceSettingsWidget : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Builds the settings form controls.
   *
   * @param parent Qt parent (settings scroll / inspector host).
   */
  explicit ServiceSettingsWidget(QWidget* parent = nullptr);

  /**
   * @brief Returns settings fields merged into the local @c config_.
   *
   * @return Copy of @c config_ reflecting current widgets.
   * @see setConfig()
   */
  ServiceCallPanelConfig config() const;

  /**
   * @brief Replaces local config and refreshes the form widgets.
   *
   * @param config Full panel config; chrome / timeout / layout are shown here.
   * @see config()
   */
  void setConfig(const ServiceCallPanelConfig& config);

 signals:
  /**
   * @brief Emitted after any user edit to settings fields.
   *
   * Listeners should read @ref config() and merge into the panel.
   */
  void configChanged();

 private slots:
  /** @brief Opens a color dialog and updates the Call-button color swatch. */
  void pickButtonColor();

 private:
  /** @brief Writes widgets into @c config_ and emits @ref configChanged(). */
  void emitConfigChanged();

  /** Working copy of panel config (settings subset edited here). */
  ServiceCallPanelConfig config_;

  QLineEdit* title_edit_ = nullptr;           /**< Panel / dock title. */
  QSpinBox* timeout_spin_ = nullptr;          /**< Call timeout (seconds). */
  QComboBox* layout_combo_ = nullptr;         /**< Vertical vs horizontal split. */
  QLineEdit* button_label_edit_ = nullptr;    /**< Call-button label. */
  QLineEdit* button_tooltip_edit_ = nullptr;  /**< Call-button tooltip. */
  QPushButton* button_color_button_ = nullptr;/**< Color swatch / picker. */
};

}  // namespace service_panel
}  // namespace autoviz
