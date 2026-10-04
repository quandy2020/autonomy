/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file teleop_types.hpp
 * @brief Shared enums and configuration for the Teleop panel.
 *
 * Defines stick modes, Twist field mappings for discrete buttons, and
 * @ref TeleopPanelConfig used by @ref TeleopPanel, @ref TeleopControlWidget,
 * and @ref TeleopSettingsWidget.
 *
 * @see TeleopPanel
 * @see teleop_twist_utils.hpp
 * @see DefaultTeleopPanelConfig()
 */

#pragma once

#include <QString>

namespace autoviz {
namespace teleop {

/**
 * @enum TeleopDirection
 * @brief Cardinal teleop button / keyboard direction (legacy discrete pad).
 *
 * Used when mapping discrete up/down/left/right/stop actions onto Twist
 * fields via @ref TeleopButtonConfig.
 */
enum class TeleopDirection {
  kUp = 0,   /**< Forward / positive primary axis. */
  kDown,     /**< Backward / negative primary axis. */
  kLeft,     /**< Strafe left or turn left (per button config). */
  kRight,    /**< Strafe right or turn right (per button config). */
  kStop,     /**< Explicit stop / zero command. */
};

/**
 * @enum TeleopTwistField
 * @brief Which @c geometry_msgs::Twist scalar a discrete button writes.
 *
 * @see TeleopButtonConfig
 * @see TwistFromButton()
 */
enum class TeleopTwistField {
  kLinearX = 0,  /**< Twist.linear.x (forward/back). */
  kLinearY,      /**< Twist.linear.y (strafe). */
  kLinearZ,      /**< Twist.linear.z. */
  kAngularX,     /**< Twist.angular.x. */
  kAngularY,     /**< Twist.angular.y. */
  kAngularZ,     /**< Twist.angular.z (yaw). */
};

/**
 * @struct TeleopButtonConfig
 * @brief Mapping of one discrete teleop button to a Twist field + value.
 *
 * @see TeleopPanelConfig::up
 * @see TwistFromButton()
 */
struct TeleopButtonConfig {
  TeleopTwistField field = TeleopTwistField::kLinearX; /**< Target Twist scalar. */
  double value = 0.0; /**< Value written when the button is active. */
};

/**
 * @enum TeleopStickMode
 * @brief Joystick layout: dual sticks vs single arcade stick.
 *
 * @see TeleopControlWidget::setStickMode()
 */
enum class TeleopStickMode {
  kDual = 0,    /**< Move + Turn sticks (default). */
  kArcade = 1,  /**< One stick: forward/back = speed, left/right = yaw. */
};

/**
 * @struct TeleopPanelConfig
 * @brief Full runtime configuration for one Teleop panel instance.
 *
 * Covers topic / publish rate, stick mode and speed limits, optional smart
 * teleop routing, discrete button mappings, and settings visibility.
 *
 * @see DefaultTeleopPanelConfig()
 * @see TeleopPanel::config()
 */
struct TeleopPanelConfig {
  QString title = QStringLiteral("Teleop"); /**< Panel / dock title. */
  QString topic = QStringLiteral("/cmd_vel"); /**< Twist publish topic. */
  double publish_rate_hz = 1.0; /**< Periodic publish rate while sticks are held. */
  bool stop_on_release = true;  /**< Publish zero Twist when sticks are released. */
  /** Route commands through task teleop goal channel instead of /cmd_vel. */
  bool smart_teleop_enabled = false;
  TeleopStickMode stick_mode = TeleopStickMode::kDual; /**< Dual vs Arcade. */
  /** Max |linear.x / linear.y| in m/s when stick is fully deflected. */
  double max_linear_speed = 0.5;
  /** Max |angular.z| in rad/s when stick is fully deflected. */
  double max_angular_speed = 0.5;
  TeleopButtonConfig up;    /**< Discrete Up button mapping. */
  TeleopButtonConfig down;  /**< Discrete Down button mapping. */
  TeleopButtonConfig left;  /**< Discrete Left button mapping. */
  TeleopButtonConfig right; /**< Discrete Right button mapping. */
  TeleopButtonConfig stop;  /**< Discrete Stop button mapping. */
  bool settings_visible = false; /**< Whether settings UI starts visible. */
};

/**
 * @brief Returns a default @ref TeleopPanelConfig for a new panel.
 *
 * @return Sensible starting config (cmd_vel, dual stick, stop-on-release).
 */
TeleopPanelConfig DefaultTeleopPanelConfig();

}  // namespace teleop
}  // namespace autoviz
