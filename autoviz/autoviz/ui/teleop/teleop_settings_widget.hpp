/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file teleop_settings_widget.hpp
 * @brief Settings form for Teleop panel topic, rates, sticks, and button maps.
 *
 * Edits title, Twist topic, publish rate, stop-on-release, smart teleop, stick
 * mode / max speeds, and discrete Up/Down/Left/Right/Stop field mappings.
 *
 * @see TeleopPanel
 * @see TeleopPanelConfig
 * @see twistFieldLabel()
 */

#pragma once

#include <QWidget>

#include "autoviz/ui/teleop/teleop_types.hpp"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLineEdit;
class QVBoxLayout;

namespace autoviz {
namespace common {
class VisualizationManager;
}

namespace teleop {

/**
 * @class TeleopSettingsWidget
 * @brief Edits @ref TeleopPanelConfig fields not driven by the live sticks.
 *
 * Emits @ref configChanged() on any field edit so @ref TeleopPanel can sync
 * writers / timers and persist session config.
 *
 * @note @p manager is reserved for future channel pickers; may be @c nullptr.
 */
class TeleopSettingsWidget : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Builds the settings form controls.
   *
   * @param manager Non-owning visualization manager (optional / reserved).
   * @param parent Qt parent (settings scroll / inspector host).
   */
  explicit TeleopSettingsWidget(common::VisualizationManager* manager,
                                QWidget* parent = nullptr);

  /**
   * @brief Replaces local config and refreshes all form widgets.
   *
   * @param config Full panel config to display.
   * @see config()
   */
  void setConfig(const TeleopPanelConfig& config);

  /**
   * @brief Returns settings fields merged into the local @c config_.
   *
   * @return Copy of @c config_ reflecting current widgets.
   * @see setConfig()
   */
  TeleopPanelConfig config() const;

 signals:
  /**
   * @brief Emitted after any user edit to settings fields.
   *
   * Listeners should read @ref config() and apply / persist.
   */
  void configChanged();

 private slots:
  /** @brief Writes widgets into @c config_ and emits @ref configChanged(). */
  void emitConfigChanged();

 private:
  /**
   * @brief Adds one discrete-button row (field combo + value spin).
   *
   * @param layout Parent vertical layout to append the row to.
   * @param name Row label (e.g. @c "Up").
   * @param field_out Out-parameter for the created field combo.
   * @param value_out Out-parameter for the created value spin.
   * @param seed Initial field / value from config.
   */
  void addAxisRow(QVBoxLayout* layout, const QString& name, QComboBox** field_out,
                  QDoubleSpinBox** value_out, const TeleopButtonConfig& seed);

  /** @brief Pushes @c config_ button mappings into the axis editors. */
  void syncAxisEditors();

  /** Non-owning; reserved for channel discovery. */
  common::VisualizationManager* manager_ = nullptr;

  /** Working copy of panel config. */
  TeleopPanelConfig config_;

  QLineEdit* title_edit_ = nullptr;             /**< Panel / dock title. */
  QLineEdit* topic_edit_ = nullptr;             /**< Twist publish topic. */
  QDoubleSpinBox* publish_rate_spin_ = nullptr; /**< Loop publish rate (Hz). */
  QCheckBox* stop_on_release_check_ = nullptr;  /**< Zero on stick release. */
  QCheckBox* smart_teleop_check_ = nullptr;     /**< Route via teleop goals. */
  QComboBox* stick_mode_combo_ = nullptr;       /**< Dual vs Arcade. */
  QDoubleSpinBox* max_linear_spin_ = nullptr;   /**< Max linear (m/s). */
  QDoubleSpinBox* max_angular_spin_ = nullptr;  /**< Max angular (rad/s). */

  QComboBox* up_field_ = nullptr;       /**< Up button Twist field. */
  QDoubleSpinBox* up_value_ = nullptr;  /**< Up button value. */
  QComboBox* down_field_ = nullptr;     /**< Down button Twist field. */
  QDoubleSpinBox* down_value_ = nullptr;/**< Down button value. */
  QComboBox* left_field_ = nullptr;     /**< Left button Twist field. */
  QDoubleSpinBox* left_value_ = nullptr;/**< Left button value. */
  QComboBox* right_field_ = nullptr;    /**< Right button Twist field. */
  QDoubleSpinBox* right_value_ = nullptr;/**< Right button value. */
  QComboBox* stop_field_ = nullptr;     /**< Stop button Twist field. */
  QDoubleSpinBox* stop_value_ = nullptr;/**< Stop button value. */
};

}  // namespace teleop
}  // namespace autoviz
