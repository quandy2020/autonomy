/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file publish_settings_widget.hpp
 * @brief Compact settings form for Publish panel chrome (title / button look).
 *
 * Hosted in the panel settings scroll area or the inspector dock. Edits do
 * not change channel / JSON payload — those live in @ref PublishEditorWidget.
 *
 * @see PublishPanel
 * @see PublishPanelConfig
 */

#pragma once

#include <QWidget>

#include "autoviz/ui/publish/publish_types.hpp"

class QLineEdit;
class QPushButton;

namespace autoviz {
namespace publish_panel {

/**
 * @class PublishSettingsWidget
 * @brief Edits title, publish-button label / tooltip / color of a Publish panel.
 *
 * Emits @ref configChanged() on any field edit so @ref PublishPanel can sync
 * @c config_ and persist via session config.
 *
 * @note Does not own the panel; @ref PublishPanel keeps the canonical config
 *       and calls @ref setConfig() / @ref config().
 */
class PublishSettingsWidget : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Builds the settings form controls.
   *
   * @param parent Qt parent (settings scroll / inspector host).
   */
  explicit PublishSettingsWidget(QWidget* parent = nullptr);

  /**
   * @brief Returns the settings subset merged into the local @c config_.
   *
   * @return Copy of @c config_ reflecting current line edits / color.
   * @see setConfig()
   */
  PublishPanelConfig config() const;

  /**
   * @brief Replaces local config and refreshes the form widgets.
   *
   * @param config Full panel config; only chrome fields are shown here.
   * @see config()
   */
  void setConfig(const PublishPanelConfig& config);

 signals:
  /**
   * @brief Emitted after any user edit to title / button chrome.
   *
   * Listeners should read @ref config() and merge into the panel.
   */
  void configChanged();

 private:
  /** @brief Writes widgets into @c config_ and emits @ref configChanged(). */
  void emitConfigChanged();

  /** @brief Opens a color dialog and updates the button color swatch. */
  void pickButtonColor();

  /** Working copy of panel config (chrome fields edited here). */
  PublishPanelConfig config_;

  /** Panel / dock title editor. */
  QLineEdit* title_edit_ = nullptr;

  /** Primary publish-button label editor. */
  QLineEdit* button_label_edit_ = nullptr;

  /** Primary publish-button tooltip editor. */
  QLineEdit* button_tooltip_edit_ = nullptr;

  /** Color swatch button that opens the color picker. */
  QPushButton* button_color_button_ = nullptr;
};

}  // namespace publish_panel
}  // namespace autoviz
