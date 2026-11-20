// Copyright 2026 The Openbot Authors (duyongquan).
// SPDX-License-Identifier: Apache-2.0
/** @file coverage_panel.hpp
 * @brief Region coverage controls embedded in the navigation panel. */
#pragma once
#ifdef AUTONOMY_BUILD_COVERAGE
#include <QString>
#include <QWidget>

#include <cstddef>
#include <cstdint>
#include <future>
#include <memory>
#include <utility>
#include <vector>
#pragma push_macro("forever")
#undef forever
#include "autolink/action/client.hpp"
#pragma pop_macro("forever")
#include "autolink/node/reader.hpp"
#include "autolink/node/writer.hpp"
#include "automsgs/msgs/geometry_msgs/point_stamped.pb.h"
#include "automsgs/msgs/nav_msgs/odometry.pb.h"
#include "automsgs/msgs/visualization_msgs/marker_array.pb.h"
#include "autonomy/task/navigation/coverage/proto/coverage.pb.h"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QListWidget;
class QPlainTextEdit;
class QSpinBox;
class QProgressBar;
class QPushButton;
class QShortcut;
class QTimer;
class QToolButton;

namespace autoviz::common {
class VisualizationManager;
}

namespace autoviz::widget {
class InfoBar;
}

namespace autoviz::navigation {

/**
 * @class CoveragePanel
 * @brief Field editor and action client for complete region coverage.
 *
 * The panel keeps one editable ring list (work area first, then manual and
 * live obstacles), submits native compute/navigate action goals, shows the
 * full-path preview, and can capture obstacle blobs from the live map display
 * so a moving obstacle field can be re-planned without retyping coordinates.
 */
class CoveragePanel : public QWidget {
  Q_OBJECT

 public:
  /** @brief Bind controls to the visualization manager's native node. */
  explicit CoveragePanel(common::VisualizationManager* manager,
                         QWidget* parent = nullptr);
  ~CoveragePanel() override;

  /** @brief Cancel only goals owned by this coverage panel. */
  void CancelCoverage();

 private:
  /** @brief One editable field or obstacle ring. */
  struct Ring {
    /** @brief Display name, e.g. "Work area" or "Obstacle 1". */
    QString name;
    /** @brief Ring kind: field, obstacle or live. */
    QString kind;
    /** @brief Vertices in metres, counter-clockwise. */
    std::vector<std::pair<double, double>> points;
  };

  using Compute = autonomy::task::coverage::proto::ComputeCoveragePathAction;
  using Navigate =
      autonomy::task::coverage::proto::NavigateCompleteCoverageAction;
  using ComputeClient = autolink::action::Client<Compute>;
  using NavigateClient = autolink::action::Client<Navigate>;

  /** @brief Build the scrollable settings, field and action sections. */
  void BuildUi();

  /** @brief Validate the edited region rings and construct the request. */
  autonomy::task::coverage::proto::CoverageRequest BuildRequest() const;

  /** @brief Initialize native clients and the full-path preview display. */
  void EnsureClients();

  /** @brief Synchronize control availability with goal ownership. */
  void SetBusy(bool busy);

  /** @brief Replace the ring list, keeping a sensible selection. */
  void RebuildRingList(int selected);

  /** @brief Update Remove availability for the selected ring. */
  void LoadSelectedRing(int row);

  /** @brief Append an editable square obstacle at the field centre. */
  void AddObstacleRing();

  /** @brief Delete the selected obstacle ring. */
  void RemoveSelectedRing();

  /** @brief Delete every manual and live obstacle ring. */
  void ClearObstacleRings();

  /** @brief Capture obstacle blobs from the newest /map occupancy grid. */
  void RefreshLiveObstacles();

  /** @brief Toggle the advanced JSON editor and sync it with the ring list. */
  void OnAdvancedToggled(bool expanded);

  /** @brief Commit spin-box and points editors before submitting a goal. */
  void CommitEditors();

  /** @brief Show a status line with an optional semantic colour. */
  void SetStatus(const QString& text, const QString& state = QString());

  /** @brief Drive the progress bar for planning or execution. */
  void SetProgress(bool running, int value = -1);

  /** @brief Serialize all rings to JSON for the advanced editor. */
  QString SerializeRings() const;

  /** @brief Replace all rings from JSON; returns false on invalid input. */
  bool ApplyRingsJson(const QString& text);

  /** @brief Index of the work-area ring, or -1 when missing. */
  int FieldRingIndex() const;

  /** @brief Compute a complete path and publish its preview. */
  void PreviewCoverage();

  /** @brief Submit the edited field as a complete coverage navigation task. */
  void StartCoverage();

  /** @brief Observe native acceptance, progress and terminal results. */
  void PollActions();

  /** @brief Split the field into rooms, optionally from live /map. */
  void SplitRooms(bool from_map);

  /** @brief Start full coverage of one room. */
  void StartRegion(std::size_t index);

  /** @brief Clean every room in sequence. */
  void StartAllRooms();

  /** @brief Highlight the room containing the robot. */
  void UpdateCurrentRegion(double x, double y);

  /** @brief Start or finish interactive obstacle polygon drawing. */
  void ToggleObstacleDrawing();

  /** @brief Append one 3D click to the obstacle draft. */
  void HandleClickedPoint(double x, double y, const QString& frame);

  /** @brief Commit the draft polygon as an obstacle ring. */
  void FinishObstacleDraft();

  /** @brief Discard the draft polygon. */
  void CancelObstacleDraft();

  /** @brief Activate a viewport tool through the toolbar action. */
  void ActivateTool(const QString& tool_id);

  /** @brief Republish manual obstacles and the in-progress draft. */
  void PublishSketch();

  common::VisualizationManager* manager_{nullptr};
  std::vector<Ring> rings_;
  QListWidget* ring_list_{nullptr};
  QPlainTextEdit* polygons_{nullptr};
  QPushButton* advanced_{nullptr};
  QLabel* rings_summary_{nullptr};
  QPushButton* add_obstacle_{nullptr};
  QPushButton* draw_obstacle_{nullptr};
  QPushButton* remove_ring_{nullptr};
  QPushButton* clear_obstacles_{nullptr};
  QCheckBox* live_obstacles_{nullptr};
  QDoubleSpinBox* obstacle_margin_{nullptr};
  QPushButton* refresh_obstacles_{nullptr};
  QLabel* obstacle_status_{nullptr};
  QCheckBox* auto_replan_{nullptr};
  QLineEdit* frame_{nullptr};
  QDoubleSpinBox* radius_{nullptr};
  QDoubleSpinBox* swath_{nullptr};
  bool swath_manual_{false};
  double robot_x_{0.0};
  double robot_y_{0.0};
  bool have_robot_pose_{false};
  QDoubleSpinBox* headland_{nullptr};
  QComboBox* route_mode_{nullptr};
  QComboBox* path_mode_{nullptr};
  QCheckBox* approach_{nullptr};
  QPushButton* preview_{nullptr};
  QPushButton* start_{nullptr};
  QPushButton* cancel_{nullptr};
  QProgressBar* progress_{nullptr};
  widget::InfoBar* note_bar_{nullptr};
  QLabel* progress_value_{nullptr};
  QLabel* remaining_value_{nullptr};
  QListWidget* rooms_{nullptr};
  QSpinBox* room_columns_{nullptr};
  QSpinBox* room_rows_{nullptr};
  QPushButton* split_rooms_{nullptr};
  QPushButton* map_rooms_{nullptr};
  QPushButton* clean_rooms_{nullptr};
  QLabel* current_region_{nullptr};
  std::vector<autonomy::task::coverage::proto::CoverageRegion> regions_;
  bool cleaning_regions_{false};
  std::size_t region_index_{0};
  QTimer* timer_{nullptr};
  QTimer* live_timer_{nullptr};
  QShortcut* draw_shortcut_{nullptr};
  QShortcut* finish_shortcut_{nullptr};
  QShortcut* cancel_shortcut_{nullptr};
  bool drawing_{false};
  QString draft_frame_;
  std::vector<std::pair<double, double>> draft_;
  std::shared_ptr<autolink::Writer<automsgs::msgs::visualization_msgs::MarkerArray>>
      sketch_writer_;
  std::shared_ptr<autolink::Reader<automsgs::msgs::geometry_msgs::PointStamped>>
      click_reader_;
  std::shared_ptr<autolink::Reader<automsgs::msgs::nav_msgs::Odometry>>
      odom_reader_;
  bool busy_{false};
  bool cancel_requested_{false};
  bool pending_replan_{false};
  std::uint64_t obstacle_signature_{0};
  std::shared_ptr<ComputeClient> compute_client_;
  std::shared_ptr<NavigateClient> navigate_client_;
  std::shared_future<
      std::shared_ptr<autolink::action::ClientGoalHandle<Compute>>>
      compute_acceptance_;
  std::shared_future<
      std::shared_ptr<autolink::action::ClientGoalHandle<Navigate>>>
      navigate_acceptance_;
  std::shared_ptr<autolink::action::ClientGoalHandle<Compute>> compute_goal_;
  std::shared_ptr<autolink::action::ClientGoalHandle<Navigate>> navigate_goal_;
  std::shared_ptr<autolink::Writer<automsgs::msgs::nav_msgs::Path>>
      preview_writer_;
};

}  // namespace autoviz::navigation
#endif
