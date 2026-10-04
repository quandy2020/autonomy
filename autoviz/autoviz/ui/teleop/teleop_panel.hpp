/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file teleop_panel.hpp
 * @brief Dockable Teleop panel — sticks, settings, Twist / smart-teleop publish.
 *
 * Hosts @ref TeleopControlWidget and @ref TeleopSettingsWidget. Publishes
 * @c geometry_msgs::Twist on the configured topic and optionally routes
 * velocity through @c TeleopGoal / @c TeleopFeedback (smart teleop).
 *
 * ## Data flow
 *
 * - **Out:** stick motion → compose Twist → @c publish_timer_ / writers;
 *   config edits emit @ref configChanged().
 * - **In:** @ref setConfig() / @ref applySettings() restore topic, rates,
 *   stick mode, and smart-teleop routing.
 *
 * @see TeleopControlWidget
 * @see TeleopSettingsWidget
 * @see TeleopPanelConfig
 * @see PanelDockWidget
 */

#pragma once

#include <memory>
#include <optional>

#include <QPointer>
#include <QWidget>

#include <automsgs/msgs/geometry_msgs/twist.pb.h>

#include <autolink/node/reader.hpp>
#include <autolink/node/writer.hpp>
#include <automsgs/task/teleop.pb.h>
#include "autoviz/ui/teleop/teleop_types.hpp"

class QElapsedTimer;
class QFocusEvent;
class QScrollArea;
class QTimer;
class QToolButton;

namespace autoviz {

class PanelDockWidget;
namespace common {
class VisualizationManager;
}
namespace teleop {
class TeleopControlWidget;
class TeleopSettingsWidget;
}

namespace teleop {

/**
 * @class TeleopPanel
 * @brief Top-level Teleop panel widget installed in a @ref PanelDockWidget.
 *
 * ## Layout
 *
 * @code
 * ┌─ dock title bar [⚙ settings] … ──────────────────────────┐
 * │ TeleopControlWidget (sticks / speeds / smart teleop)      │
 * │ optional settings scroll (topic / rate / button maps)     │
 * └───────────────────────────────────────────────────────────┘
 * @endcode
 *
 * ## Publish paths
 *
 * - **Direct:** writes Twist to @ref TeleopPanelConfig::topic.
 * - **Smart teleop:** session start/stop + velocity goals via
 *   @c TeleopGoal writer; status from @c TeleopFeedback reader.
 *
 * @note Owns autolink reader/writer shared_ptrs for smart teleop; does not
 *       own @ref common::VisualizationManager.
 */
class TeleopPanel : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs the panel with control + settings widgets and timers.
   *
   * @param manager Non-owning visualization manager for channel / node access.
   * @param parent Qt parent (dock content host).
   */
  explicit TeleopPanel(common::VisualizationManager* manager,
                       QWidget* parent = nullptr);

  /**
   * @brief Stops timers and shuts down smart-teleop readers/writers.
   */
  ~TeleopPanel() override;

  /**
   * @brief Installs settings (and related) tools on the dock title bar.
   *
   * @param dock Hosting @ref PanelDockWidget; tools are held via @c QPointer.
   */
  void installTitleBarTools(PanelDockWidget* dock);

  /**
   * @brief Returns the combined control + settings configuration.
   *
   * @return Current @ref TeleopPanelConfig.
   * @see setConfig()
   */
  TeleopPanelConfig config() const;

  /**
   * @brief Replaces configuration and applies it to UI and publishers.
   *
   * @param config Full panel config from session or clone.
   * @see config()
   * @see applySettings()
   */
  void setConfig(const TeleopPanelConfig& config);

  /**
   * @brief Copies config from another panel instance (split / duplicate).
   *
   * @param config Source configuration.
   */
  void cloneConfigFrom(const TeleopPanelConfig& config);

  /**
   * @brief Applies settings fields (topic, rates, modes) without full rebuild.
   *
   * @param config Settings subset to merge / apply.
   * @see setConfig()
   */
  void applySettings(const TeleopPanelConfig& config);

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

  /**
   * @brief Publish timer tick: sends the composed / active Twist.
   */
  void onPublishTick();

  /**
   * @brief Control widget linear stick changed.
   *
   * @param x Normalized strafe (Dual) or unused (Arcade).
   * @param y Normalized forward / back.
   */
  void onLinearChanged(double x, double y);

  /**
   * @brief Control widget angular stick changed.
   *
   * @param turn Normalized yaw in [-1, 1].
   */
  void onAngularChanged(double turn);

  /** @brief Linear stick released; may publish stop if configured. */
  void onLinearReleased();

  /** @brief Angular stick released; may publish stop if configured. */
  void onAngularReleased();

  /** @brief Explicit stop: zero Twist and end smart-teleop motion. */
  void onStopClicked();

 private:
  /**
   * @brief Publishes @p twist on the configured cmd_vel-style topic.
   *
   * @param twist Velocity command to write.
   */
  void publishTwist(const automsgs::msgs::geometry_msgs::Twist& twist);

  /**
   * @brief Publishes velocity through the smart-teleop goal channel.
   *
   * @param twist Velocity command wrapped as a teleop goal.
   */
  void publishTeleopVelocity(const automsgs::msgs::geometry_msgs::Twist& twist);

  /** @brief Sends a smart-teleop session-start goal. */
  void publishTeleopSessionStart();

  /** @brief Sends a smart-teleop session-stop goal. */
  void publishTeleopSessionStop();

  /** @brief Publishes a zero Twist (and smart-teleop stop as needed). */
  void publishStop();

  /**
   * @brief Stores @p twist as the active command for the publish timer.
   *
   * @param twist Latest composed velocity.
   */
  void setActiveTwist(const automsgs::msgs::geometry_msgs::Twist& twist);

  /** @brief Starts/stops @c publish_timer_ based on activity and rate. */
  void updatePublishTimer();

  /**
   * @brief Builds Twist from current linear / angular stick state × max speeds.
   *
   * @return Composed velocity command.
   */
  automsgs::msgs::geometry_msgs::Twist composeTwist() const;

  /** @brief Composes and publishes the current stick state immediately. */
  void publishComposedTwist();

  /** @brief Pushes @c config_ into @c settings_widget_. */
  void syncSettingsWidgetFromConfig();

  /** @brief Updates title-bar tool checked state from settings visibility. */
  void syncSettingsToolState();

  /** @brief Lazily creates the smart-teleop @c TeleopGoal writer. */
  void ensureTeleopGoalWriter();

  /** @brief Drops and recreates the goal writer after failures / reconnect. */
  void resetTeleopGoalWriter();

  /** @brief Lazily creates the @c TeleopFeedback reader for status. */
  void ensureTeleopFeedbackReader();

  /** @brief Stops and clears smart-teleop readers/writers. */
  void shutdownTeleopReaders();

  /** @brief Pushes smart-teleop connection status into the control widget. */
  void updateSmartTeleopUiStatus();

  /** @brief Starts the session connect retry / wait timer. */
  void startSessionConnectTimer();

  /** @brief Stops the session connect timer. */
  void stopSessionConnectTimer();

  /** @brief Resets the goal writer when write failures exceed a streak. */
  void maybeResetTeleopGoalWriter();

  /** Non-owning visualization manager. */
  common::VisualizationManager* manager_ = nullptr;

  /** Canonical panel configuration. */
  TeleopPanelConfig config_;

  /** Live sticks / mode / speed UI. */
  TeleopControlWidget* control_ = nullptr;

  /** Topic / rate / button-map settings form. */
  TeleopSettingsWidget* settings_widget_ = nullptr;

  /** Scroll host for settings when shown inline. */
  QScrollArea* settings_scroll_ = nullptr;

  /** Container reparented with settings for inspector hand-off. */
  QWidget* settings_container_ = nullptr;

  /** Dock title-bar settings tool (weak). */
  QPointer<QToolButton> settings_button_;

  /** Periodic publish while sticks are held. */
  QTimer* publish_timer_ = nullptr;

  /** Last non-empty Twist held for timer ticks (nullopt when idle). */
  std::optional<automsgs::msgs::geometry_msgs::Twist> active_twist_;

  double linear_x_ = 0.0;   /**< Normalized strafe from Move stick. */
  double linear_y_ = 0.0;   /**< Normalized forward from Move / Arcade. */
  double angular_turn_ = 0.0; /**< Normalized yaw from Turn / Arcade. */
  bool linear_active_ = false;  /**< Move / Arcade stick currently held. */
  bool angular_active_ = false; /**< Turn stick currently held. */
  bool smart_teleop_session_active_ = false; /**< Session start sent, not stopped. */
  bool teleop_start_logged_this_connect_ = false; /**< Dedup start log per connect. */
  QString smart_teleop_status_text_; /**< Last status shown in the control UI. */

  /** Smart-teleop goal writer (created lazily). */
  std::shared_ptr<::autolink::Writer<::autonomy::task::proto::TeleopGoal>>
      teleop_goal_writer_;

  /** Smart-teleop feedback reader (created lazily). */
  std::shared_ptr<::autolink::Reader<::autonomy::task::proto::TeleopFeedback>>
      teleop_feedback_reader_;

  /** Waits / retries while establishing a smart-teleop session. */
  QTimer* session_connect_timer_ = nullptr;

  /** Elapsed time since last goal-writer reset attempt. */
  QElapsedTimer* goal_writer_reset_timer_ = nullptr;

  /** Consecutive goal write failures; triggers @ref maybeResetTeleopGoalWriter(). */
  int goal_write_fail_streak_ = 0;
};

}  // namespace teleop
}  // namespace autoviz
