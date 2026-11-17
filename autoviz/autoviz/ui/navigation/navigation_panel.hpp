/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from nav2_rviz_plugins/nav2_panel (Intel / Nav2).
 *****************************************************************************/

/**
 * @file navigation_panel.hpp
 * @brief Dockable Navigation panel — AB-point NavigateToPose and multi-point
 *        NavigateThroughPoses / waypoint following (Nav2 panel feature set).
 *
 * @see tools::GoalUpdater
 * @see PanelDockWidget
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include <QPointer>
#include <QString>
#include <QWidget>

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>

class QCheckBox;
class QDoubleSpinBox;
class QFrame;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;
class QStackedWidget;
class QTimer;
class QToolButton;
class QVBoxLayout;

namespace autoviz {

class PanelDockWidget;

namespace common {
class VisualizationManager;
}

namespace widget {
class CollapsiblePanel;
class InfoBar;
class SegmentedControl;
class StatusBadge;
}

namespace navigation {

struct PoseRow {
  QWidget* widget = nullptr;
  QLabel* index_label = nullptr;
  QLineEdit* frame_id_edit = nullptr;
  QDoubleSpinBox* pos_x_spin = nullptr;
  QDoubleSpinBox* pos_y_spin = nullptr;
  QDoubleSpinBox* yaw_spin = nullptr;
  QToolButton* remove_button = nullptr;
};

/**
 * @class NavigationPanel
 * @brief Nav2-compatible navigation control surface for Autoviz.
 *
 * AB navigation uses @c /goal_pose. Multi-point uses @c /goal_poses
 * (TaskServer through-poses) plus @c /navigate_through_poses when the
 * action server is ready, and sequential @c /goal_pose fallback for
 * Follow when @c /follow_waypoints is absent.
 */
class NavigationPanel : public QWidget {
  Q_OBJECT

 public:
  explicit NavigationPanel(common::VisualizationManager* manager,
                           QWidget* parent = nullptr);
  ~NavigationPanel() override;

  void installTitleBarTools(PanelDockWidget* dock);

 private slots:
  void onStartResetClicked();
  void onPauseResumeClicked();
  void onModeSegmentChanged(int index);
  void onPauseWaypointClicked();
  void onSendNavToPose();
  void onStartThroughPoses();
  void onStartWaypointFollowing();
  void onAddNavThroughPose();
  void onRemoveNavThroughPose();
  void onSaveWaypoints();
  void onLoadWaypoints();
  void onLoopEdited(int value);
  void onStoreInitialPoseToggled(int state);
  void onNewGoal(double x, double y, double theta, QString frame);
  void onProbeServers();
  void updateAccumulatedPosesFromTabs();
  void onBrowseBehaviorTree();

 private:
  enum class UiMode {
    kIdle,
    kRunning,
    kPaused,
    kAccumulating,
    kFollowingWp,
    kNavThroughPoses,
    kPausedWp,
  };

  struct Impl;
  using PoseStamped = automsgs::msgs::geometry_msgs::PoseStamped;

  void buildUi();
  void applyUi();
  void ensureClients();
  void cancelActionGoals();
  void cancelAllGoals();
  void publishCancelNavigation();
  void requestTaskCancel();
  void clearNavVisuals();
  void onNavigationFeedback(int status, float distance_remaining,
                            int poses_remaining);
  bool publishGoalPose(const PoseStamped& pose);
  bool publishGoalPoses(const std::vector<PoseStamped>& poses);
  void startNavigation(const PoseStamped& pose);
  void startNavThroughPoses();
  void startWaypointFollowing();
  void startSequentialWaypoint(size_t index);
  void createPoseRow(int index);
  void syncTabsWithAccumulatedPoses();
  void updateWpNavigationMarkers();
  void ensureWaypointMarkerDisplay();
  void refreshPoseEmptyState();
  bool lookupInitialPose(PoseStamped* out) const;
  PoseStamped makePose(double x, double y, double yaw_rad,
                       const QString& frame) const;
  void setGoalStatus(int8_t status);
  void setNavFeedback(double eta_s, double distance_m, double elapsed_s,
                      int recoveries, int poses_remaining = -1);
  void setWaypointNote(const QString& note);
  void runOnUi(std::function<void()> fn);
  void enterMode(UiMode mode);
  QString defaultFrame() const;
  bool isBusy() const;

  common::VisualizationManager* manager_ = nullptr;
  QPointer<PanelDockWidget> dock_;
  std::unique_ptr<Impl> impl_;

  UiMode mode_ = UiMode::kIdle;
  int loop_no_ = 0;

  widget::StatusBadge* nav_badge_ = nullptr;
  widget::StatusBadge* loc_badge_ = nullptr;
  widget::StatusBadge* goal_badge_ = nullptr;
  QLabel* eta_value_ = nullptr;
  QLabel* distance_value_ = nullptr;
  QLabel* elapsed_value_ = nullptr;
  QLabel* recoveries_value_ = nullptr;
  widget::InfoBar* note_bar_ = nullptr;
  widget::SegmentedControl* mode_segment_ = nullptr;
  QStackedWidget* mode_stack_ = nullptr;
  QWidget* ab_page_ = nullptr;
  QWidget* waypoint_page_ = nullptr;
  QWidget* pose_list_host_ = nullptr;
  QVBoxLayout* pose_list_layout_ = nullptr;
  QLabel* pose_empty_ = nullptr;
  QLabel* pose_count_label_ = nullptr;
  QPushButton* start_reset_button_ = nullptr;
  QPushButton* pause_resume_button_ = nullptr;
  QPushButton* add_pose_button_ = nullptr;
  QPushButton* remove_pose_button_ = nullptr;
  QPushButton* save_waypoints_button_ = nullptr;
  QPushButton* load_waypoints_button_ = nullptr;
  QPushButton* pause_waypoint_button_ = nullptr;
  QPushButton* start_nav_to_pose_button_ = nullptr;
  QPushButton* start_through_button_ = nullptr;
  QPushButton* start_follow_button_ = nullptr;
  QCheckBox* store_initial_pose_checkbox_ = nullptr;
  QSpinBox* nr_of_loops_ = nullptr;
  QLineEdit* behavior_tree_file_ = nullptr;
  QLineEdit* nav_to_pose_frame_id_ = nullptr;
  QDoubleSpinBox* nav_to_pose_x_ = nullptr;
  QDoubleSpinBox* nav_to_pose_y_ = nullptr;
  QDoubleSpinBox* nav_to_pose_yaw_ = nullptr;
  widget::CollapsiblePanel* advanced_ = nullptr;
  QTimer* probe_timer_ = nullptr;
};

}  // namespace navigation
}  // namespace autoviz
