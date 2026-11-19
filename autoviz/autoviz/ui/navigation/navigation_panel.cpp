/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from nav2_rviz_plugins/nav2_panel (Intel / Nav2).
 *****************************************************************************/

#include "autoviz/ui/navigation/navigation_panel.hpp"

#ifdef forever
#undef forever
#endif

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <QAbstractSpinBox>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMetaObject>
#include <QPushButton>
#include <QPointer>
#include <QRegularExpression>
#include <QSignalBlocker>
#include <QScrollArea>
#include <QSizePolicy>
#include <QSpinBox>
#include <QStackedWidget>
#include <QThread>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>

#include <glog/logging.h>

#include <autolink/action/client.hpp>
#include <autolink/node/node.hpp>
#include <autolink/node/reader.hpp>
#include <autolink/node/writer.hpp>
#include <automsgs/actions/nav_actions.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/std_msgs/bool.pb.h>
#include <automsgs/msgs/time_utils.hpp>
#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>
#include <automsgs/task/navigation.pb.h>

#include "autoviz/common/session_config.hpp"
#ifdef AUTONOMY_BUILD_COVERAGE
#include "autoviz/ui/navigation/coverage_panel.hpp"
#endif
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/common/widget/display/status_badge.hpp"
#include "autoviz/plugins/displays/display.hpp"
#include "autoviz/plugins/displays/display_group.hpp"
#include "autoviz/common/widget/feedback/info_bar.hpp"
#include "autoviz/common/widget/input/segmented_control.hpp"
#include "autoviz/common/widget/layout/collapsible_panel.hpp"
#include "autoviz/plugins/tools/goal_pose/goal_common.hpp"
#include "autoviz/transform/buffer.hpp"
#include "autoviz/ui/app/icon_loader.hpp"
#include "autoviz/ui/panel/dock.hpp"
#include "autoviz/ui/theme/panel.hpp"
#include "autoviz/ui/theme/style.hpp"
#include "autoviz/ui/theme/tokens.hpp"

namespace autoviz {
namespace navigation {
namespace {

constexpr char kNavigateToPose[] = "/navigate_to_pose";
constexpr char kNavigateThroughPoses[] = "/navigate_through_poses";
constexpr char kFollowWaypoints[] = "/follow_waypoints";
constexpr char kWaypointsTopic[] = "/waypoints";
constexpr char kGoalPose[] = "/goal_pose";
constexpr char kGoalPoses[] = "/goal_poses";
constexpr char kCancelNavigation[] = "/cancel_navigation";
constexpr char kNavigationFeedback[] = "/autonomy/task/navigation/feedback";
constexpr char kPlanTopic[] = "/plan";

using PoseStamped = automsgs::msgs::geometry_msgs::PoseStamped;
using Marker = automsgs::msgs::visualization_msgs::Marker;
using MarkerArray = automsgs::msgs::visualization_msgs::MarkerArray;
using Duration = automsgs::msgs::builtin_interfaces::Duration;

double DurationSeconds(const Duration& duration) {
  return static_cast<double>(duration.sec()) +
         static_cast<double>(duration.nanosec()) * 1e-9;
}

QString FormatFixed(double value, int precision) {
  return QString::number(value, 'f', precision);
}

QString FormatSecondsChip(double seconds) {
  if (!std::isfinite(seconds) || seconds < 0.05) {
    return QStringLiteral("—");
  }
  if (seconds < 10.0) {
    return QStringLiteral("%1 s").arg(FormatFixed(seconds, 1));
  }
  return QStringLiteral("%1 s").arg(FormatFixed(seconds, 0));
}

QString FormatMetersChip(double meters) {
  if (!std::isfinite(meters) || meters < 0.005) {
    return QStringLiteral("—");
  }
  if (meters < 10.0) {
    return QStringLiteral("%1 m").arg(FormatFixed(meters, 2));
  }
  return QStringLiteral("%1 m").arg(FormatFixed(meters, 1));
}

widget::FeedbackLevel NoteLevel(const QString& note) {
  const QString lower = note.toLower();
  if (lower.contains(QLatin1String("fail")) ||
      lower.contains(QLatin1String("invalid")) ||
      lower.contains(QLatin1String("rejected")) ||
      lower.contains(QLatin1String("not available")) ||
      lower.contains(QLatin1String("cannot"))) {
    return widget::FeedbackLevel::kError;
  }
  if (lower.contains(QLatin1String("waiting")) ||
      lower.contains(QLatin1String("paused"))) {
    return widget::FeedbackLevel::kWarning;
  }
  return widget::FeedbackLevel::kInformation;
}

automsgs::msgs::geometry_msgs::Quaternion OrientationAroundZ(double yaw) {
  automsgs::msgs::geometry_msgs::Quaternion q;
  q.set_x(0.0);
  q.set_y(0.0);
  q.set_z(std::sin(yaw * 0.5));
  q.set_w(std::cos(yaw * 0.5));
  return q;
}

double YawFromOrientation(const automsgs::msgs::geometry_msgs::Quaternion& q) {
  const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  return std::atan2(siny_cosp, cosy_cosp);
}

double RadToDeg(double rad) { return rad * 180.0 / M_PI; }
double DegToRad(double deg) { return deg * M_PI / 180.0; }

widget::StatusBadge::Status GoalBadgeStatus(int8_t status) {
  switch (status) {
    case 2:
    case 4:
      return widget::StatusBadge::Status::kSuccess;
    case 5:
      return widget::StatusBadge::Status::kWarning;
    case 6:
      return widget::StatusBadge::Status::kError;
    default:
      return widget::StatusBadge::Status::kNeutral;
  }
}

QString GoalBadgeText(int8_t status) {
  switch (status) {
    case 2:
      return QObject::tr("Active");
    case 4:
      return QObject::tr("Reached");
    case 5:
      return QObject::tr("Canceled");
    case 6:
      return QObject::tr("Aborted");
    case 0:
      return QObject::tr("Unknown");
    default:
      return QObject::tr("Idle");
  }
}

QFrame* MakeMetricChip(const QString& caption, QLabel** value_out,
                       QWidget* parent) {
  auto* chip = new QFrame(parent);
  chip->setObjectName(QStringLiteral("NavigationMetricChip"));
  chip->setAttribute(Qt::WA_StyledBackground, true);
  chip->setStyleSheet(style::sheet(QStringLiteral("navigation/metric_chip")));
  auto* layout = new QVBoxLayout(chip);
  layout->setContentsMargins(8, 6, 8, 6);
  layout->setSpacing(0);
  auto* caption_label = new QLabel(caption, chip);
  caption_label->setObjectName(QStringLiteral("NavigationMetricCaption"));
  caption_label->setStyleSheet(
      style::sheet(QStringLiteral("navigation/metric_caption")));
  auto* value = new QLabel(QStringLiteral("—"), chip);
  value->setObjectName(QStringLiteral("NavigationMetricValue"));
  value->setStyleSheet(style::sheet(QStringLiteral("navigation/metric_value")));
  layout->addWidget(caption_label);
  layout->addWidget(value);
  chip->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  *value_out = value;
  return chip;
}

QDoubleSpinBox* MakeCompactSpin(QWidget* parent, double min, double max,
                                int decimals, double step,
                                const QString& suffix) {
  auto* spin = new QDoubleSpinBox(parent);
  spin->setRange(min, max);
  spin->setDecimals(decimals);
  spin->setSingleStep(step);
  spin->setSuffix(suffix);
  spin->setButtonSymbols(QAbstractSpinBox::NoButtons);
  spin->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  StyleCompactSettingsField(spin);
  return spin;
}

double ParseYamlNumber(const QString& token, bool* ok) {
  return token.trimmed().toDouble(ok);
}

bool ParseWaypointYaml(const QString& text, std::vector<PoseStamped>* out) {
  if (out == nullptr) {
    return false;
  }
  out->clear();
  const QStringList lines = text.split(QLatin1Char('\n'));
  PoseStamped current;
  std::vector<double> pose;
  std::vector<double> orientation;
  enum class Block { kNone, kPose, kOrientation };
  Block block = Block::kNone;
  bool in_waypoints = false;
  bool have_waypoint = false;

  const auto flush = [&]() {
    if (!have_waypoint) {
      return true;
    }
    if (pose.size() < 3 || orientation.size() < 4) {
      return false;
    }
    current.mutable_pose()->mutable_position()->set_x(pose[0]);
    current.mutable_pose()->mutable_position()->set_y(pose[1]);
    current.mutable_pose()->mutable_position()->set_z(pose[2]);
    current.mutable_pose()->mutable_orientation()->set_w(orientation[0]);
    current.mutable_pose()->mutable_orientation()->set_x(orientation[1]);
    current.mutable_pose()->mutable_orientation()->set_y(orientation[2]);
    current.mutable_pose()->mutable_orientation()->set_z(orientation[3]);
    if (current.header().frame_id().empty()) {
      current.mutable_header()->set_frame_id("map");
    }
    out->push_back(current);
    current = PoseStamped();
    pose.clear();
    orientation.clear();
    have_waypoint = false;
    block = Block::kNone;
    return true;
  };

  static const QRegularExpression kKey(QStringLiteral("^\\s*([A-Za-z0-9_]+)\\s*:\\s*$"));
  static const QRegularExpression kList(
      QStringLiteral("^\\s*-\\s*([+-]?(?:\\d+\\.?\\d*|\\.\\d+)(?:[eE][+-]?\\d+)?)\\s*$"));

  for (QString raw : lines) {
    const int comment = raw.indexOf(QLatin1Char('#'));
    if (comment >= 0) {
      raw = raw.left(comment);
    }
    if (raw.trimmed().isEmpty()) {
      continue;
    }
    const QRegularExpressionMatch key = kKey.match(raw);
    if (key.hasMatch()) {
      const QString name = key.captured(1);
      if (name == QLatin1String("waypoints")) {
        if (!flush()) {
          return false;
        }
        in_waypoints = true;
        continue;
      }
      if (!in_waypoints) {
        continue;
      }
      if (name == QLatin1String("pose")) {
        block = Block::kPose;
        continue;
      }
      if (name == QLatin1String("orientation")) {
        block = Block::kOrientation;
        continue;
      }
      if (name.startsWith(QLatin1String("waypoint"))) {
        if (!flush()) {
          return false;
        }
        have_waypoint = true;
        block = Block::kNone;
        continue;
      }
      block = Block::kNone;
      continue;
    }
    const QRegularExpressionMatch item = kList.match(raw);
    if (!item.hasMatch() || !in_waypoints) {
      continue;
    }
    bool ok = false;
    const double value = ParseYamlNumber(item.captured(1), &ok);
    if (!ok) {
      return false;
    }
    if (block == Block::kPose) {
      pose.push_back(value);
    } else if (block == Block::kOrientation) {
      orientation.push_back(value);
    }
  }
  return flush();
}

QString EmitWaypointYaml(const std::vector<PoseStamped>& poses) {
  QString out = QStringLiteral("waypoints:\n");
  for (size_t i = 0; i < poses.size(); ++i) {
    const auto& pose = poses[i];
    out += QStringLiteral("  waypoint%1:\n").arg(i);
    out += QStringLiteral("    pose:\n      - %1\n      - %2\n      - %3\n")
               .arg(pose.pose().position().x(), 0, 'g', 12)
               .arg(pose.pose().position().y(), 0, 'g', 12)
               .arg(pose.pose().position().z(), 0, 'g', 12);
    out += QStringLiteral(
               "    orientation:\n      - %1\n      - %2\n      - %3\n      - %4\n")
               .arg(pose.pose().orientation().w(), 0, 'g', 12)
               .arg(pose.pose().orientation().x(), 0, 'g', 12)
               .arg(pose.pose().orientation().y(), 0, 'g', 12)
               .arg(pose.pose().orientation().z(), 0, 'g', 12);
  }
  return out;
}

QPushButton* MakeCompactGhost(const QString& text, QWidget* parent) {
  QPushButton* button = MakeFlatActionButton(text, parent);
  button->setMinimumHeight(28);
  return button;
}

void SetMarkerColor(automsgs::msgs::std_msgs::ColorRGBA* color, float r, float g,
                    float b, float a) {
  if (color == nullptr) {
    return;
  }
  color->set_r(r);
  color->set_g(g);
  color->set_b(b);
  color->set_a(a);
}

bool DisplayCoversWaypoints(const display::Display* display) {
  if (display == nullptr) {
    return false;
  }
  if (display->typeId() == "MarkerArray" &&
      display->channel() == kWaypointsTopic) {
    return true;
  }
  const auto* group = dynamic_cast<const display::DisplayGroup*>(display);
  if (group == nullptr) {
    return false;
  }
  for (const auto& child : group->children()) {
    if (DisplayCoversWaypoints(child.get())) {
      return true;
    }
  }
  return false;
}

void FillMarkerHeader(Marker* marker, const PoseStamped& pose,
                      const std::string& fallback_frame) {
  if (marker == nullptr) {
    return;
  }
  *marker->mutable_header() = pose.header();
  if (marker->header().frame_id().empty()) {
    marker->mutable_header()->set_frame_id(fallback_frame);
  }
}

}  // namespace

struct NavigationPanel::Impl {
  using NavigateToPoseAction = automsgs::actions::NavigateToPoseAction;
  using NavigateThroughPosesAction = automsgs::actions::NavigateThroughPosesAction;
  using FollowWaypointsAction = automsgs::actions::FollowWaypointsAction;
  using PoseStamped = automsgs::msgs::geometry_msgs::PoseStamped;
  using NavToPoseClient = autolink::action::Client<NavigateToPoseAction>;
  using NavThroughPosesClient = autolink::action::Client<NavigateThroughPosesAction>;
  using FollowWaypointsClient = autolink::action::Client<FollowWaypointsAction>;

  int unique_id = 0;
  int goal_index = 0;
  int loop_count = 0;
  bool store_initial_pose = false;
  bool initial_pose_stored = false;
  bool sequential_wp = false;
  size_t sequential_index = 0;
  std::string base_frame = "base_footprint";

  std::vector<PoseStamped> accumulated_poses;
  std::vector<PoseStamped> stored_poses;
  std::vector<PoseRow> pose_rows;

  std::shared_ptr<NavToPoseClient> navigation_client;
  std::shared_ptr<NavThroughPosesClient> through_poses_client;
  std::shared_ptr<FollowWaypointsClient> waypoint_client;
  std::shared_ptr<NavToPoseClient::GoalHandle> navigation_goal_handle;
  std::shared_ptr<NavThroughPosesClient::GoalHandle> through_poses_goal_handle;
  std::shared_ptr<FollowWaypointsClient::GoalHandle> waypoint_goal_handle;
  std::shared_ptr<autolink::Writer<automsgs::msgs::visualization_msgs::MarkerArray>>
      waypoint_marker_writer;
  std::shared_ptr<autolink::Writer<automsgs::msgs::geometry_msgs::PoseStamped>>
      goal_pose_writer;
  std::shared_ptr<autolink::Writer<automsgs::msgs::geometry_msgs::PoseStampedArray>>
      goal_poses_writer;
  std::shared_ptr<autolink::Writer<automsgs::msgs::std_msgs::Bool>> cancel_writer;
  std::shared_ptr<autolink::Writer<automsgs::msgs::nav_msgs::Path>> plan_clear_writer;
  std::shared_ptr<autolink::Reader<autonomy::task::proto::NavigationFeedback>>
      feedback_reader;
  int cancel_epoch = 0;
  int markers_epoch = 0;
  bool suppress_nav_feedback = false;
};

using NavigateToPoseAction = automsgs::actions::NavigateToPoseAction;
using NavigateThroughPosesAction = automsgs::actions::NavigateThroughPosesAction;
using FollowWaypointsAction = automsgs::actions::FollowWaypointsAction;
using NavToPoseClient = autolink::action::Client<NavigateToPoseAction>;
using NavThroughPosesClient = autolink::action::Client<NavigateThroughPosesAction>;
using FollowWaypointsClient = autolink::action::Client<FollowWaypointsAction>;

NavigationPanel::NavigationPanel(common::VisualizationManager* manager,
                                 QWidget* parent)
    : QWidget(parent), manager_(manager), impl_(std::make_unique<Impl>()) {
  setFocusPolicy(Qt::StrongFocus);
  ApplyPanelShell(this);
  setObjectName(QStringLiteral("NavigationPanelContent"));
  setMinimumWidth(248);
  setStyleSheet(style::sheet(QStringLiteral("navigation/panel_root")));
  buildUi();
  applyUi();
  connect(&tools::GoalUpdater, &tools::GoalPoseUpdater::updateGoal, this,
          &NavigationPanel::onNewGoal);
  probe_timer_ = new QTimer(this);
  probe_timer_->setInterval(500);
  connect(probe_timer_, &QTimer::timeout, this, &NavigationPanel::onProbeServers);
  probe_timer_->start();
  onProbeServers();
  ensureWaypointMarkerDisplay();
}

NavigationPanel::~NavigationPanel() {
  cancelAllGoals();
}

void NavigationPanel::installTitleBarTools(PanelDockWidget* dock) {
  dock_ = dock;
}

void NavigationPanel::buildUi() {
  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);

  auto* scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  auto* content = new QWidget(scroll);
  scroll->setWidget(content);
  root->addWidget(scroll, 1);

  auto* layout = new QVBoxLayout(content);
  layout->setContentsMargins(10, 10, 10, 10);
  layout->setSpacing(10);

  auto* status = new QFrame(content);
  status->setObjectName(QStringLiteral("NavigationStatusCard"));
  status->setAttribute(Qt::WA_StyledBackground, true);
  status->setStyleSheet(style::sheet(QStringLiteral("navigation/status_card")));
  auto* status_layout = new QVBoxLayout(status);
  status_layout->setContentsMargins(10, 10, 10, 10);
  status_layout->setSpacing(8);

  auto* badge_row = new QHBoxLayout();
  badge_row->setSpacing(6);
  nav_badge_ = new widget::StatusBadge(tr("Nav"), widget::StatusBadge::Status::kNeutral, status);
  loc_badge_ = new widget::StatusBadge(tr("Loc"), widget::StatusBadge::Status::kNeutral, status);
  goal_badge_ = new widget::StatusBadge(tr("Idle"), widget::StatusBadge::Status::kNeutral, status);
  badge_row->addWidget(nav_badge_);
  badge_row->addWidget(loc_badge_);
  badge_row->addWidget(goal_badge_);
  badge_row->addStretch(1);
  status_layout->addLayout(badge_row);

  auto* metrics = new QGridLayout();
  metrics->setHorizontalSpacing(6);
  metrics->setVerticalSpacing(6);
  metrics->addWidget(MakeMetricChip(tr("ETA"), &eta_value_, status), 0, 0);
  metrics->addWidget(MakeMetricChip(tr("Distance"), &distance_value_, status), 0, 1);
  metrics->addWidget(MakeMetricChip(tr("Elapsed"), &elapsed_value_, status), 1, 0);
  metrics->addWidget(MakeMetricChip(tr("Recoveries"), &recoveries_value_, status), 1, 1);
  status_layout->addLayout(metrics);

  note_bar_ = new widget::InfoBar(status);
  note_bar_->SetClosable(false);
  note_bar_->SetTone(style::Tone::Shell);
  note_bar_->hide();
  status_layout->addWidget(note_bar_);
  layout->addWidget(status);

  mode_segment_ = new widget::SegmentedControl(content);
  mode_segment_->SetTone(style::Tone::Frost);
  mode_segment_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  mode_segment_->AddSegment(tr("AB point"));
  mode_segment_->AddSegment(tr("Waypoints"));
#ifdef AUTONOMY_BUILD_COVERAGE
  mode_segment_->AddSegment(tr("Coverage"));
#endif
  mode_segment_->SetCurrentIndex(0);
  mode_segment_->setToolTip(
      tr("AB point sends NavigateToPose. Waypoints accumulate poses for ThroughPoses or Follow."));
  layout->addWidget(mode_segment_);

  mode_stack_ = new QStackedWidget(content);

  ab_page_ = new QWidget(mode_stack_);
  auto* ab_layout = new QVBoxLayout(ab_page_);
  ab_layout->setContentsMargins(0, 2, 0, 0);
  ab_layout->setSpacing(8);
  auto* ab_hint = new QLabel(
      tr("Click 2D Goal Pose in the 3D view, or enter a pose below."), ab_page_);
  StyleHintLabel(ab_hint);
  ab_hint->setWordWrap(true);
  ab_layout->addWidget(ab_hint);

  auto* ab_form = new QFormLayout();
  ApplyCompactForm(ab_form);
  nav_to_pose_frame_id_ = new QLineEdit(QStringLiteral("map"), ab_page_);
  StyleCompactSettingsField(nav_to_pose_frame_id_);
  nav_to_pose_x_ = MakeCompactSpin(ab_page_, -1000.0, 1000.0, 2, 0.1, tr(" m"));
  nav_to_pose_y_ = MakeCompactSpin(ab_page_, -1000.0, 1000.0, 2, 0.1, tr(" m"));
  nav_to_pose_yaw_ = MakeCompactSpin(ab_page_, -180.0, 180.0, 1, 1.0, QStringLiteral("°"));
  nav_to_pose_yaw_->setWrapping(true);
  ab_form->addRow(MakeSettingsFormLabel(tr("Frame"), ab_page_), nav_to_pose_frame_id_);
  ab_form->addRow(MakeSettingsFormLabel(tr("X"), ab_page_), nav_to_pose_x_);
  ab_form->addRow(MakeSettingsFormLabel(tr("Y"), ab_page_), nav_to_pose_y_);
  ab_form->addRow(MakeSettingsFormLabel(tr("Yaw"), ab_page_), nav_to_pose_yaw_);
  ab_layout->addLayout(ab_form);
  start_nav_to_pose_button_ = MakePrimaryActionButton(tr("Go"), ab_page_);
  start_nav_to_pose_button_->setMinimumHeight(32);
  ab_layout->addWidget(start_nav_to_pose_button_);
  ab_layout->addStretch(1);
  mode_stack_->addWidget(ab_page_);

  waypoint_page_ = new QWidget(mode_stack_);
  auto* wp_layout = new QVBoxLayout(waypoint_page_);
  wp_layout->setContentsMargins(0, 2, 0, 0);
  wp_layout->setSpacing(8);

  auto* wp_header = new QHBoxLayout();
  pose_count_label_ = new QLabel(tr("0 poses"), waypoint_page_);
  StyleSectionTitle(pose_count_label_);
  wp_header->addWidget(pose_count_label_, 1);
  add_pose_button_ = MakeCompactGhost(tr("Add"), waypoint_page_);
  add_pose_button_->setToolTip(tr("Append a blank pose"));
  wp_header->addWidget(add_pose_button_);
  wp_layout->addLayout(wp_header);

  auto* wp_tools = new QHBoxLayout();
  wp_tools->setSpacing(6);
  load_waypoints_button_ = MakeCompactGhost(tr("Load"), waypoint_page_);
  save_waypoints_button_ = MakeCompactGhost(tr("Save"), waypoint_page_);
  remove_pose_button_ = MakeDestructiveFlatActionButton(tr("Remove last"), waypoint_page_);
  load_waypoints_button_->setToolTip(tr("Load Nav2 waypoint YAML"));
  save_waypoints_button_->setToolTip(tr("Save Nav2 waypoint YAML"));
  remove_pose_button_->setToolTip(tr("Remove the last pose"));
  wp_tools->addWidget(load_waypoints_button_);
  wp_tools->addWidget(save_waypoints_button_);
  wp_tools->addWidget(remove_pose_button_);
  wp_tools->addStretch(1);
  wp_layout->addLayout(wp_tools);

  pose_list_host_ = new QWidget(waypoint_page_);
  pose_list_layout_ = new QVBoxLayout(pose_list_host_);
  pose_list_layout_->setContentsMargins(0, 0, 0, 0);
  pose_list_layout_->setSpacing(6);
  pose_empty_ = new QLabel(
      tr("Click 2D Goal Pose to drop waypoints, or Add a blank pose."),
      pose_list_host_);
  StyleHintLabel(pose_empty_);
  pose_empty_->setWordWrap(true);
  pose_empty_->setAlignment(Qt::AlignCenter);
  pose_empty_->setMinimumHeight(48);
  pose_list_layout_->addWidget(pose_empty_);
  wp_layout->addWidget(pose_list_host_);

  auto* loop_row = new QHBoxLayout();
  loop_row->addWidget(MakeSettingsFormLabel(tr("Loops"), waypoint_page_));
  nr_of_loops_ = new QSpinBox(waypoint_page_);
  nr_of_loops_->setRange(0, 9999);
  nr_of_loops_->setValue(0);
  StyleCompactSettingsField(nr_of_loops_);
  nr_of_loops_->setMaximumWidth(88);
  loop_row->addWidget(nr_of_loops_);
  store_initial_pose_checkbox_ =
      new QCheckBox(tr("Start from robot"), waypoint_page_);
  loop_row->addWidget(store_initial_pose_checkbox_, 1);
  wp_layout->addLayout(loop_row);

  start_through_button_ = MakePrimaryActionButton(tr("Through poses"), waypoint_page_);
  start_follow_button_ = MakeCompactGhost(tr("Follow waypoints"), waypoint_page_);
  pause_waypoint_button_ = MakeCompactGhost(tr("Pause following"), waypoint_page_);
  start_through_button_->setMinimumHeight(32);
  start_follow_button_->setMinimumHeight(28);
  start_through_button_->setToolTip(
      tr("NavigateThroughPoses — visit every pose in order, then stop"));
  start_follow_button_->setToolTip(
      tr("FollowWaypoints — loop over poses (falls back to sequential NavigateToPose)"));
  wp_layout->addWidget(start_through_button_);
  wp_layout->addWidget(start_follow_button_);
  wp_layout->addWidget(pause_waypoint_button_);
  wp_layout->addStretch(1);
  mode_stack_->addWidget(waypoint_page_);
#ifdef AUTONOMY_BUILD_COVERAGE
  coverage_panel_ = new CoveragePanel(manager_, mode_stack_);
  mode_stack_->addWidget(coverage_panel_);
#endif
  layout->addWidget(mode_stack_, 1);

  advanced_ = new widget::CollapsiblePanel(content);
  advanced_->SetTitle(tr("Advanced"));
  advanced_->SetTone(style::Tone::Frost);
  advanced_->SetExpanded(false);
  auto* advanced_body = new QWidget();
  auto* advanced_layout = new QVBoxLayout(advanced_body);
  advanced_layout->setContentsMargins(0, 4, 0, 0);
  advanced_layout->setSpacing(6);
  auto* bt_row = new QHBoxLayout();
  behavior_tree_file_ = new QLineEdit(advanced_body);
  behavior_tree_file_->setPlaceholderText(tr("Default behavior tree"));
  StyleCompactSettingsField(behavior_tree_file_);
  auto* browse = new QToolButton(advanced_body);
  browse->setText(QStringLiteral("…"));
  browse->setToolTip(tr("Browse for a behavior tree XML file"));
  browse->setCursor(Qt::PointingHandCursor);
  browse->setFixedSize(28, 28);
  bt_row->addWidget(behavior_tree_file_, 1);
  bt_row->addWidget(browse);
  advanced_layout->addLayout(bt_row);
  advanced_->SetContentWidget(advanced_body);
  layout->addWidget(advanced_);

  auto* footer = new QFrame(this);
  footer->setObjectName(QStringLiteral("NavigationFooter"));
  footer->setAttribute(Qt::WA_StyledBackground, true);
  footer->setStyleSheet(style::sheet(QStringLiteral("navigation/footer")));
  auto* footer_layout = new QHBoxLayout(footer);
  footer_layout->setContentsMargins(10, 8, 10, 10);
  footer_layout->setSpacing(8);
  pause_resume_button_ = MakeCompactGhost(tr("Pause"), footer);
  start_reset_button_ = MakePrimaryActionButton(tr("Reset"), footer);
  pause_resume_button_->setMinimumHeight(32);
  start_reset_button_->setMinimumHeight(32);
  footer_layout->addWidget(pause_resume_button_);
  footer_layout->addWidget(start_reset_button_, 1);
  root->addWidget(footer);

  setNavFeedback(0.0, 0.0, 0.0, 0);
  setGoalStatus(0);

  connect(start_reset_button_, &QPushButton::clicked, this,
          &NavigationPanel::onStartResetClicked);
  connect(pause_resume_button_, &QPushButton::clicked, this,
          &NavigationPanel::onPauseResumeClicked);
  connect(mode_segment_, &widget::SegmentedControl::CurrentIndexChanged, this,
          &NavigationPanel::onModeSegmentChanged);
  connect(pause_waypoint_button_, &QPushButton::clicked, this,
          &NavigationPanel::onPauseWaypointClicked);
  connect(start_nav_to_pose_button_, &QPushButton::clicked, this,
          &NavigationPanel::onSendNavToPose);
  connect(start_through_button_, &QPushButton::clicked, this,
          &NavigationPanel::onStartThroughPoses);
  connect(start_follow_button_, &QPushButton::clicked, this,
          &NavigationPanel::onStartWaypointFollowing);
  connect(add_pose_button_, &QPushButton::clicked, this,
          &NavigationPanel::onAddNavThroughPose);
  connect(remove_pose_button_, &QPushButton::clicked, this,
          &NavigationPanel::onRemoveNavThroughPose);
  connect(save_waypoints_button_, &QPushButton::clicked, this,
          &NavigationPanel::onSaveWaypoints);
  connect(load_waypoints_button_, &QPushButton::clicked, this,
          &NavigationPanel::onLoadWaypoints);
  connect(nr_of_loops_, qOverload<int>(&QSpinBox::valueChanged), this,
          &NavigationPanel::onLoopEdited);
  connect(store_initial_pose_checkbox_, &QCheckBox::stateChanged, this,
          &NavigationPanel::onStoreInitialPoseToggled);
  connect(browse, &QToolButton::clicked, this,
          &NavigationPanel::onBrowseBehaviorTree);
}

bool NavigationPanel::isBusy() const {
  return mode_ == UiMode::kRunning || mode_ == UiMode::kFollowingWp ||
         mode_ == UiMode::kNavThroughPoses;
}

void NavigationPanel::applyUi() {
#ifdef AUTONOMY_BUILD_COVERAGE
  start_reset_button_->setVisible(!coverage_selected_);
  pause_resume_button_->setVisible(!coverage_selected_);
  if (coverage_selected_) {
    setMinimumSize(400, 700);
    const QSignalBlocker blocker(mode_segment_);
    mode_segment_->SetCurrentIndex(2);
    mode_stack_->setCurrentIndex(2);
    return;
  }
  setMinimumSize(0, 0);
#endif
  const bool idle = mode_ == UiMode::kIdle;
  const bool paused = mode_ == UiMode::kPaused;
  const bool accumulating = mode_ == UiMode::kAccumulating;
  const bool following = mode_ == UiMode::kFollowingWp;
  const bool paused_wp = mode_ == UiMode::kPausedWp;
  const bool waypoint_mode =
      accumulating || following || mode_ == UiMode::kNavThroughPoses || paused_wp;
  const bool can_edit_poses = accumulating && !isBusy();

  const bool lock_mode = isBusy() || paused || paused_wp;
  const QSignalBlocker block_mode(mode_segment_);
  mode_segment_->SetCurrentIndex(waypoint_mode ? 1 : 0);
  mode_segment_->SetSegmentEnabled(0, !lock_mode);
  mode_segment_->SetSegmentEnabled(1, !lock_mode);
  mode_stack_->setCurrentIndex(waypoint_mode ? 1 : 0);

  if (isBusy() || paused_wp) {
    start_reset_button_->setText(tr("Cancel"));
    start_reset_button_->setToolTip(tr("Cancel the active navigation goal"));
    pause_resume_button_->setEnabled(false);
    pause_resume_button_->setText(tr("Pause"));
  } else if (paused) {
    start_reset_button_->setText(tr("Reset"));
    start_reset_button_->setToolTip(tr("Cancel leftover goals and clear poses"));
    pause_resume_button_->setText(tr("Resume"));
    pause_resume_button_->setEnabled(true);
    pause_resume_button_->setToolTip(tr("Resume navigation"));
  } else {
    start_reset_button_->setText(tr("Reset"));
    start_reset_button_->setToolTip(tr("Cancel leftover goals and clear poses"));
    pause_resume_button_->setText(tr("Pause"));
    pause_resume_button_->setEnabled(idle);
    pause_resume_button_->setToolTip(tr("Pause navigation"));
  }

  start_nav_to_pose_button_->setEnabled(idle);
  start_nav_to_pose_button_->setText(mode_ == UiMode::kRunning ? tr("Going…")
                                                               : tr("Go"));
  nav_to_pose_frame_id_->setEnabled(!isBusy());
  nav_to_pose_x_->setEnabled(!isBusy());
  nav_to_pose_y_->setEnabled(!isBusy());
  nav_to_pose_yaw_->setEnabled(!isBusy());
  start_through_button_->setEnabled(can_edit_poses);
  start_follow_button_->setEnabled(can_edit_poses);
  add_pose_button_->setEnabled(can_edit_poses);
  remove_pose_button_->setEnabled(can_edit_poses && !impl_->pose_rows.empty());
  save_waypoints_button_->setEnabled(!impl_->accumulated_poses.empty());
  load_waypoints_button_->setEnabled(can_edit_poses);
  nr_of_loops_->setEnabled(can_edit_poses);
  store_initial_pose_checkbox_->setEnabled(can_edit_poses);
  pause_waypoint_button_->setVisible(following || paused_wp);
  pause_waypoint_button_->setEnabled(following || paused_wp);
  pause_waypoint_button_->setText(paused_wp ? tr("Resume following")
                                            : tr("Pause following"));
  for (auto& row : impl_->pose_rows) {
    if (row.frame_id_edit != nullptr) {
      row.frame_id_edit->setEnabled(can_edit_poses);
    }
    if (row.pos_x_spin != nullptr) {
      row.pos_x_spin->setEnabled(can_edit_poses);
    }
    if (row.pos_y_spin != nullptr) {
      row.pos_y_spin->setEnabled(can_edit_poses);
    }
    if (row.yaw_spin != nullptr) {
      row.yaw_spin->setEnabled(can_edit_poses);
    }
    if (row.remove_button != nullptr) {
      row.remove_button->setEnabled(can_edit_poses);
    }
  }
  pose_count_label_->setText(
      tr("%n poses", "", static_cast<int>(impl_->accumulated_poses.size())));
  refreshPoseEmptyState();
}

void NavigationPanel::enterMode(UiMode mode) {
  mode_ = mode;
  tools::GoalUpdater.setAccumulateOnly(mode_ == UiMode::kAccumulating ||
                                       mode_ == UiMode::kNavThroughPoses ||
                                       mode_ == UiMode::kFollowingWp ||
                                       mode_ == UiMode::kPausedWp);
  applyUi();
}

void NavigationPanel::ensureClients() {
  if (manager_ == nullptr) {
    return;
  }
  auto node = manager_->autolinkNode();
  if (!node) {
    return;
  }
  if (!impl_->navigation_client) {
    impl_->navigation_client = std::make_shared<NavToPoseClient>(node, kNavigateToPose);
  }
  if (!impl_->through_poses_client) {
    impl_->through_poses_client =
        std::make_shared<NavThroughPosesClient>(node, kNavigateThroughPoses);
  }
  if (!impl_->waypoint_client) {
    impl_->waypoint_client =
        std::make_shared<FollowWaypointsClient>(node, kFollowWaypoints);
  }
  if (!impl_->waypoint_marker_writer) {
    impl_->waypoint_marker_writer = node->CreateWriter<MarkerArray>(kWaypointsTopic);
  }
  if (!impl_->goal_pose_writer) {
    impl_->goal_pose_writer =
        node->CreateWriter<automsgs::msgs::geometry_msgs::PoseStamped>(kGoalPose);
  }
  if (!impl_->goal_poses_writer) {
    impl_->goal_poses_writer =
        node->CreateWriter<automsgs::msgs::geometry_msgs::PoseStampedArray>(
            kGoalPoses);
  }
  if (!impl_->cancel_writer) {
    impl_->cancel_writer =
        node->CreateWriter<automsgs::msgs::std_msgs::Bool>(kCancelNavigation);
  }
  if (!impl_->plan_clear_writer) {
    impl_->plan_clear_writer =
        node->CreateWriter<automsgs::msgs::nav_msgs::Path>(kPlanTopic);
  }
  if (!impl_->feedback_reader) {
    impl_->feedback_reader =
        node->CreateReader<autonomy::task::proto::NavigationFeedback>(
            kNavigationFeedback,
            [this](const std::shared_ptr<autonomy::task::proto::NavigationFeedback>&
                       feedback) {
              if (!feedback) {
                return;
              }
              const int status = static_cast<int>(feedback->status());
              const float distance = feedback->distance_remaining();
              const int total = feedback->total_waypoints();
              const int index = feedback->current_waypoint_index();
              const int remaining = total > index ? total - index : 0;
              runOnUi([this, status, distance, remaining]() {
                onNavigationFeedback(status, distance, remaining);
              });
            });
  }
}

void NavigationPanel::onProbeServers() {
  ensureClients();
  const bool nav_ready =
      impl_->navigation_client && impl_->navigation_client->ActionServerIsReady();
  nav_badge_->SetText(tr("Nav"));
  nav_badge_->SetStatus(nav_ready ? widget::StatusBadge::Status::kSuccess
                                  : widget::StatusBadge::Status::kNeutral);
  const bool through_ready =
      impl_->through_poses_client &&
      impl_->through_poses_client->ActionServerIsReady();
  nav_badge_->setToolTip(
      nav_ready
          ? (through_ready ? tr("NavigateToPose and ThroughPoses are ready")
                           : tr("NavigateToPose ready; ThroughPoses uses /goal_poses"))
          : tr("Waiting for /navigate_to_pose; ThroughPoses still uses /goal_poses"));

  bool loc_ready = false;
  if (manager_ != nullptr && manager_->tfBuffer() != nullptr) {
    const std::string fixed = defaultFrame().toStdString();
    loc_ready = manager_->tfBuffer()->canTransform(
        fixed, impl_->base_frame, automsgs::msgs::builtin_interfaces::ZeroTime());
  }
  loc_badge_->SetText(tr("Loc"));
  loc_badge_->SetStatus(loc_ready ? widget::StatusBadge::Status::kSuccess
                                  : widget::StatusBadge::Status::kNeutral);
  loc_badge_->setToolTip(loc_ready ? tr("TF map ← base is available")
                                   : tr("Waiting for localization TF"));
}

void NavigationPanel::onStartResetClicked() {
  if (isBusy() || mode_ == UiMode::kPausedWp) {
    impl_->suppress_nav_feedback = true;
    cancelAllGoals();
    clearNavVisuals();
    setWaypointNote(QString());
    setGoalStatus(5);
    enterMode(UiMode::kIdle);
    return;
  }
  cancelAllGoals();
  clearNavVisuals();
  impl_->accumulated_poses.clear();
  impl_->stored_poses.clear();
  updateWpNavigationMarkers();
  syncTabsWithAccumulatedPoses();
  setGoalStatus(0);
  setNavFeedback(0.0, 0.0, 0.0, 0);
  setWaypointNote(QString());
  enterMode(UiMode::kIdle);
}

void NavigationPanel::onPauseResumeClicked() {
  if (mode_ == UiMode::kIdle) {
    cancelAllGoals();
    enterMode(UiMode::kPaused);
    return;
  }
  if (mode_ == UiMode::kPaused) {
    enterMode(UiMode::kIdle);
  }
}

void NavigationPanel::onModeSegmentChanged(int index) {
#ifdef AUTONOMY_BUILD_COVERAGE
  if (!isBusy()) {
    coverage_selected_ = index == 2;
    if (coverage_selected_) {
      mode_ = UiMode::kIdle;
      applyUi();
      return;
    }
  }
#endif
  if (isBusy() || mode_ == UiMode::kPaused || mode_ == UiMode::kPausedWp) {
    applyUi();
    return;
  }
  if (index == 1) {
    impl_->loop_count = 0;
    impl_->initial_pose_stored = false;
    impl_->goal_index = 0;
    impl_->sequential_wp = false;
    enterMode(UiMode::kAccumulating);
    return;
  }
  if (mode_ == UiMode::kAccumulating) {
    enterMode(UiMode::kIdle);
  }
}

void NavigationPanel::onStartThroughPoses() {
  updateAccumulatedPosesFromTabs();
  if (impl_->accumulated_poses.empty()) {
    setWaypointNote(tr("Add at least one pose first"));
    return;
  }
  startNavThroughPoses();
}

void NavigationPanel::onStartWaypointFollowing() {
  updateAccumulatedPosesFromTabs();
  if (impl_->accumulated_poses.empty()) {
    setWaypointNote(tr("Add at least one pose first"));
    return;
  }
  startWaypointFollowing();
}

void NavigationPanel::onPauseWaypointClicked() {
  if (mode_ == UiMode::kFollowingWp) {
    impl_->stored_poses = impl_->accumulated_poses;
    cancelAllGoals();
    setWaypointNote(tr("Navigation is paused."));
    enterMode(UiMode::kPausedWp);
    return;
  }
  if (mode_ == UiMode::kPausedWp) {
    impl_->accumulated_poses = impl_->stored_poses;
    startWaypointFollowing();
  }
}

void NavigationPanel::onSendNavToPose() {
  if (mode_ != UiMode::kIdle) {
    return;
  }
  const PoseStamped pose =
      makePose(nav_to_pose_x_->value(), nav_to_pose_y_->value(),
               DegToRad(nav_to_pose_yaw_->value()), nav_to_pose_frame_id_->text());
  startNavigation(pose);
}

void NavigationPanel::onAddNavThroughPose() {
  createPoseRow(static_cast<int>(impl_->pose_rows.size()));
  updateAccumulatedPosesFromTabs();
  applyUi();
}

void NavigationPanel::onRemoveNavThroughPose() {
  if (impl_->pose_rows.empty()) {
    return;
  }
  const int index = static_cast<int>(impl_->pose_rows.size()) - 1;
  impl_->pose_rows[index].widget->hide();
  pose_list_layout_->removeWidget(impl_->pose_rows[index].widget);
  impl_->pose_rows[index].widget->deleteLater();
  impl_->pose_rows.erase(impl_->pose_rows.begin() + index);
  for (size_t i = 0; i < impl_->pose_rows.size(); ++i) {
    impl_->pose_rows[i].index_label->setText(QString::number(static_cast<int>(i) + 1));
  }
  updateAccumulatedPosesFromTabs();
  applyUi();
}

void NavigationPanel::onSaveWaypoints() {
  updateAccumulatedPosesFromTabs();
  if (impl_->accumulated_poses.empty()) {
    setWaypointNote(tr("No accumulated points to save"));
    return;
  }
  QString file = QFileDialog::getSaveFileName(this, tr("Save Waypoints"), QString(),
                                              tr("yaml (*.yaml);;All Files (*)"));
  if (file.isEmpty()) {
    return;
  }
  if (!file.endsWith(QLatin1String(".yaml"), Qt::CaseInsensitive) &&
      !file.endsWith(QLatin1String(".yml"), Qt::CaseInsensitive)) {
    file += QLatin1String(".yaml");
  }
  std::ofstream out(file.toStdString());
  if (!out) {
    setWaypointNote(tr("Failed to write %1").arg(file));
    return;
  }
  out << EmitWaypointYaml(impl_->accumulated_poses).toStdString();
  setWaypointNote(tr("Saved %1 waypoints").arg(impl_->accumulated_poses.size()));
}

void NavigationPanel::onLoadWaypoints() {
  const QString file = QFileDialog::getOpenFileName(
      this, tr("Load Waypoints"), QString(), tr("yaml (*.yaml);;All Files (*)"));
  if (file.isEmpty()) {
    return;
  }
  std::ifstream in(file.toStdString());
  if (!in) {
    setWaypointNote(tr("Failed to open %1").arg(file));
    return;
  }
  const std::string raw((std::istreambuf_iterator<char>(in)),
                        std::istreambuf_iterator<char>());
  std::vector<PoseStamped> loaded;
  if (!ParseWaypointYaml(QString::fromStdString(raw), &loaded)) {
    setWaypointNote(tr("Invalid waypoint YAML"));
    return;
  }
  impl_->accumulated_poses = std::move(loaded);
  syncTabsWithAccumulatedPoses();
  updateWpNavigationMarkers();
  setWaypointNote(tr("Loaded %1 waypoints").arg(impl_->accumulated_poses.size()));
}

void NavigationPanel::onLoopEdited(int value) {
  loop_no_ = std::max(0, value);
  applyUi();
}

void NavigationPanel::onBrowseBehaviorTree() {
  const QString file = QFileDialog::getOpenFileName(
      this, tr("Behavior tree XML"), QString(),
      tr("XML (*.xml);;All Files (*)"));
  if (!file.isEmpty()) {
    behavior_tree_file_->setText(file);
  }
}

void NavigationPanel::onStoreInitialPoseToggled(int state) {
  impl_->store_initial_pose = state == Qt::Checked;
}

void NavigationPanel::onNewGoal(double x, double y, double theta, QString frame) {
  const PoseStamped pose = makePose(x, y, theta, frame);
  nav_to_pose_frame_id_->setText(frame);
  nav_to_pose_x_->setValue(x);
  nav_to_pose_y_->setValue(y);
  nav_to_pose_yaw_->setValue(RadToDeg(theta));

  if (!impl_->stored_poses.empty() &&
      (mode_ == UiMode::kPausedWp || mode_ == UiMode::kFollowingWp)) {
    setWaypointNote(tr("Cannot set goal in pause state"));
    return;
  }
  if (mode_ == UiMode::kAccumulating) {
    setWaypointNote(QString());
    impl_->accumulated_poses.push_back(pose);
    syncTabsWithAccumulatedPoses();
    updateWpNavigationMarkers();
    return;
  }
  if (isBusy()) {
    return;
  }
  // 2D Goal Pose already publishes /goal_pose (TaskServer). Track it here so
  // Cancel is available; do not send a second NavigateToPose.
  impl_->suppress_nav_feedback = false;
  ++impl_->cancel_epoch;
  impl_->accumulated_poses.clear();
  updateWpNavigationMarkers();
  ensureClients();
  setGoalStatus(2);
  setWaypointNote(QString());
  enterMode(UiMode::kRunning);
}

void NavigationPanel::createPoseRow(int index) {
  PoseRow row;
  auto* card = new QFrame(pose_list_host_);
  card->setObjectName(QStringLiteral("NavigationPoseCard"));
  card->setAttribute(Qt::WA_StyledBackground, true);
  card->setStyleSheet(style::sheet(QStringLiteral("navigation/pose_card")));
  row.widget = card;

  auto* card_layout = new QVBoxLayout(card);
  card_layout->setContentsMargins(8, 6, 8, 8);
  card_layout->setSpacing(6);

  auto* header = new QHBoxLayout();
  header->setSpacing(6);
  row.index_label = new QLabel(QString::number(index + 1), card);
  row.index_label->setObjectName(QStringLiteral("NavigationPoseIndex"));
  row.index_label->setStyleSheet(style::sheet(QStringLiteral("navigation/pose_index")));
  row.index_label->setFixedWidth(18);
  row.index_label->setAlignment(Qt::AlignCenter);
  row.frame_id_edit = new QLineEdit(defaultFrame(), card);
  StyleCompactSettingsField(row.frame_id_edit);
  row.frame_id_edit->setPlaceholderText(tr("frame"));
  row.remove_button = new QToolButton(card);
  row.remove_button->setText(QStringLiteral("×"));
  row.remove_button->setAutoRaise(true);
  row.remove_button->setCursor(Qt::PointingHandCursor);
  row.remove_button->setToolTip(tr("Remove this pose"));
  row.remove_button->setFixedSize(22, 22);
  header->addWidget(row.index_label);
  header->addWidget(row.frame_id_edit, 1);
  header->addWidget(row.remove_button);
  card_layout->addLayout(header);

  auto* values = new QHBoxLayout();
  values->setSpacing(6);
  row.pos_x_spin = MakeCompactSpin(card, -1000.0, 1000.0, 2, 0.1, tr(" m"));
  row.pos_y_spin = MakeCompactSpin(card, -1000.0, 1000.0, 2, 0.1, tr(" m"));
  row.yaw_spin = MakeCompactSpin(card, -180.0, 180.0, 1, 1.0, QStringLiteral("°"));
  row.yaw_spin->setWrapping(true);
  row.pos_x_spin->setToolTip(tr("X"));
  row.pos_y_spin->setToolTip(tr("Y"));
  row.yaw_spin->setToolTip(tr("Yaw"));
  values->addWidget(row.pos_x_spin, 1);
  values->addWidget(row.pos_y_spin, 1);
  values->addWidget(row.yaw_spin, 1);
  card_layout->addLayout(values);

  QWidget* card_ptr = card;
  connect(row.remove_button, &QToolButton::clicked, this, [this, card_ptr]() {
    for (size_t i = 0; i < impl_->pose_rows.size(); ++i) {
      if (impl_->pose_rows[i].widget != card_ptr) {
        continue;
      }
      impl_->pose_rows[i].widget->hide();
      pose_list_layout_->removeWidget(impl_->pose_rows[i].widget);
      impl_->pose_rows[i].widget->deleteLater();
      impl_->pose_rows.erase(impl_->pose_rows.begin() + static_cast<int>(i));
      for (size_t n = 0; n < impl_->pose_rows.size(); ++n) {
        impl_->pose_rows[n].index_label->setText(
            QString::number(static_cast<int>(n) + 1));
      }
      updateAccumulatedPosesFromTabs();
      applyUi();
      return;
    }
  });
  connect(row.frame_id_edit, &QLineEdit::textChanged, this,
          &NavigationPanel::updateAccumulatedPosesFromTabs);
  connect(row.pos_x_spin, &QDoubleSpinBox::valueChanged, this,
          &NavigationPanel::updateAccumulatedPosesFromTabs);
  connect(row.pos_y_spin, &QDoubleSpinBox::valueChanged, this,
          &NavigationPanel::updateAccumulatedPosesFromTabs);
  connect(row.yaw_spin, &QDoubleSpinBox::valueChanged, this,
          &NavigationPanel::updateAccumulatedPosesFromTabs);

  pose_list_layout_->insertWidget(static_cast<int>(impl_->pose_rows.size()), card);
  impl_->pose_rows.push_back(row);
}

void NavigationPanel::syncTabsWithAccumulatedPoses() {
  for (auto& row : impl_->pose_rows) {
    if (row.widget == nullptr) {
      continue;
    }
    row.widget->hide();
    pose_list_layout_->removeWidget(row.widget);
    row.widget->deleteLater();
  }
  impl_->pose_rows.clear();
  for (size_t i = 0; i < impl_->accumulated_poses.size(); ++i) {
    createPoseRow(static_cast<int>(i));
    const auto& pose = impl_->accumulated_poses[i];
    auto& row = impl_->pose_rows[i];
    const QSignalBlocker b1(row.frame_id_edit);
    const QSignalBlocker b2(row.pos_x_spin);
    const QSignalBlocker b3(row.pos_y_spin);
    const QSignalBlocker b4(row.yaw_spin);
    row.frame_id_edit->setText(QString::fromStdString(pose.header().frame_id()));
    row.pos_x_spin->setValue(pose.pose().position().x());
    row.pos_y_spin->setValue(pose.pose().position().y());
    row.yaw_spin->setValue(RadToDeg(YawFromOrientation(pose.pose().orientation())));
  }
  refreshPoseEmptyState();
}

void NavigationPanel::updateAccumulatedPosesFromTabs() {
  impl_->accumulated_poses.clear();
  impl_->accumulated_poses.reserve(impl_->pose_rows.size());
  for (const auto& row : impl_->pose_rows) {
    impl_->accumulated_poses.push_back(
        makePose(row.pos_x_spin->value(), row.pos_y_spin->value(),
                 DegToRad(row.yaw_spin->value()), row.frame_id_edit->text()));
  }
  updateWpNavigationMarkers();
  if (pose_count_label_ != nullptr) {
    pose_count_label_->setText(
        tr("%n poses", "", static_cast<int>(impl_->accumulated_poses.size())));
  }
  refreshPoseEmptyState();
}

void NavigationPanel::refreshPoseEmptyState() {
  if (pose_empty_ != nullptr) {
    pose_empty_->setVisible(impl_->pose_rows.empty());
  }
}

PoseStamped NavigationPanel::makePose(double x, double y, double yaw,
                                      const QString& frame) const {
  PoseStamped pose;
  QString frame_id = frame.trimmed();
  if (frame_id.isEmpty()) {
    frame_id = defaultFrame();
  }
  pose.mutable_header()->set_frame_id(frame_id.toStdString());
  *pose.mutable_header()->mutable_stamp() = automsgs::msgs::builtin_interfaces::TimeNow();
  pose.mutable_pose()->mutable_position()->set_x(x);
  pose.mutable_pose()->mutable_position()->set_y(y);
  pose.mutable_pose()->mutable_position()->set_z(0.0);
  *pose.mutable_pose()->mutable_orientation() = OrientationAroundZ(yaw);
  return pose;
}

QString NavigationPanel::defaultFrame() const {
  if (manager_ != nullptr && !manager_->fixedFrame().empty()) {
    return QString::fromStdString(manager_->fixedFrame());
  }
  return QStringLiteral("map");
}

bool NavigationPanel::lookupInitialPose(PoseStamped* out) const {
  if (out == nullptr || manager_ == nullptr || manager_->tfBuffer() == nullptr) {
    return false;
  }
  try {
    const auto transform = manager_->tfBuffer()->lookupTransform(
        defaultFrame().toStdString(), impl_->base_frame,
        automsgs::msgs::builtin_interfaces::ZeroTime());
    out->mutable_header()->set_frame_id(transform.header().frame_id());
    *out->mutable_header()->mutable_stamp() = transform.header().stamp();
    out->mutable_pose()->mutable_position()->set_x(
        transform.transform().translation().x());
    out->mutable_pose()->mutable_position()->set_y(
        transform.transform().translation().y());
    out->mutable_pose()->mutable_position()->set_z(
        transform.transform().translation().z());
    *out->mutable_pose()->mutable_orientation() = transform.transform().rotation();
    return true;
  } catch (const std::exception& ex) {
    LOG(WARNING) << "NavigationPanel: initial pose TF failed: " << ex.what();
    return false;
  }
}

void NavigationPanel::ensureWaypointMarkerDisplay() {
  if (manager_ == nullptr) {
    return;
  }
  for (const display::Display* display : manager_->displays()) {
    if (DisplayCoversWaypoints(display)) {
      return;
    }
  }
  common::DisplayConfig config;
  config.type = "MarkerArray";
  config.name = "Waypoints";
  config.channel = kWaypointsTopic;
  config.enabled = true;
  config.properties["use_custom_color"] = "false";
  config.properties["color"] = "0;255;0";
  config.properties["point_size"] = "4";
  config.properties["mesh_style"] = "solid";
  manager_->addDisplay(config);
}

void NavigationPanel::updateWpNavigationMarkers() {
  ensureWaypointMarkerDisplay();
  if (!impl_->waypoint_marker_writer) {
    ensureClients();
  }
  if (!impl_->waypoint_marker_writer) {
    return;
  }

  // Match nav2_rviz_plugins::Nav2Panel::updateWpNavigationMarkers:
  // green ARROW, red SPHERE, green "wp_N" TEXT_VIEW_FACING. No route line.
  const std::string fallback_frame = defaultFrame().toStdString();
  const auto& poses = impl_->accumulated_poses;

  MarkerArray array;
  Marker* clear_all = array.add_markers();
  clear_all->set_action(Marker::DELETEALL);

  impl_->unique_id = 0;
  for (size_t i = 0; i < poses.size(); ++i) {
    const auto& pose = poses[i];

    Marker* arrow = array.add_markers();
    FillMarkerHeader(arrow, pose, fallback_frame);
    arrow->set_id(impl_->unique_id++);
    arrow->set_type(Marker::ARROW);
    arrow->set_action(Marker::ADD);
    *arrow->mutable_pose() = pose.pose();
    arrow->mutable_scale()->set_x(0.7);
    arrow->mutable_scale()->set_y(0.12);
    arrow->mutable_scale()->set_z(0.06);
    SetMarkerColor(arrow->mutable_color(), 0.0f, 1.0f, 0.0f, 1.0f);

    Marker* sphere = array.add_markers();
    FillMarkerHeader(sphere, pose, fallback_frame);
    sphere->set_id(impl_->unique_id++);
    sphere->set_type(Marker::SPHERE);
    sphere->set_action(Marker::ADD);
    *sphere->mutable_pose() = pose.pose();
    sphere->mutable_scale()->set_x(0.12);
    sphere->mutable_scale()->set_y(0.12);
    sphere->mutable_scale()->set_z(0.12);
    SetMarkerColor(sphere->mutable_color(), 1.0f, 0.0f, 0.0f, 1.0f);

    Marker* text = array.add_markers();
    FillMarkerHeader(text, pose, fallback_frame);
    text->set_id(impl_->unique_id++);
    text->set_type(Marker::TEXT_VIEW_FACING);
    text->set_action(Marker::ADD);
    *text->mutable_pose() = pose.pose();
    text->mutable_pose()->mutable_orientation()->set_x(0.0);
    text->mutable_pose()->mutable_orientation()->set_y(0.0);
    text->mutable_pose()->mutable_orientation()->set_z(0.0);
    text->mutable_pose()->mutable_orientation()->set_w(1.0);
    text->mutable_pose()->mutable_position()->set_z(
        pose.pose().position().z() + 0.45);
    text->mutable_scale()->set_z(0.32);
    SetMarkerColor(text->mutable_color(), 0.0f, 1.0f, 0.0f, 1.0f);
    text->set_text("wp_" + std::to_string(i + 1));
  }

  ++impl_->markers_epoch;
  const int epoch = impl_->markers_epoch;
  impl_->waypoint_marker_writer->Write(std::make_shared<MarkerArray>(array));
  QTimer::singleShot(80, this, [this, array, epoch]() {
    if (impl_->markers_epoch != epoch || !impl_->waypoint_marker_writer) {
      return;
    }
    impl_->waypoint_marker_writer->Write(std::make_shared<MarkerArray>(array));
  });
}

void NavigationPanel::cancelActionGoals() {
  if (impl_->navigation_goal_handle && impl_->navigation_client) {
    impl_->navigation_client->AsyncCancelGoal(impl_->navigation_goal_handle);
  } else if (impl_->navigation_client) {
    impl_->navigation_client->AsyncCancelAllGoals();
  }
  if (impl_->through_poses_goal_handle && impl_->through_poses_client) {
    impl_->through_poses_client->AsyncCancelGoal(impl_->through_poses_goal_handle);
  } else if (impl_->through_poses_client) {
    impl_->through_poses_client->AsyncCancelAllGoals();
  }
  if (impl_->waypoint_goal_handle && impl_->waypoint_client) {
    impl_->waypoint_client->AsyncCancelGoal(impl_->waypoint_goal_handle);
  } else if (impl_->waypoint_client) {
    impl_->waypoint_client->AsyncCancelAllGoals();
  }
  impl_->navigation_goal_handle.reset();
  impl_->through_poses_goal_handle.reset();
  impl_->waypoint_goal_handle.reset();
  impl_->sequential_wp = false;
}

void NavigationPanel::publishCancelNavigation() {
  ensureClients();
  if (impl_->cancel_writer) {
    auto msg = std::make_shared<automsgs::msgs::std_msgs::Bool>();
    msg->set_data(true);
    impl_->cancel_writer->Write(msg);
  }
  if (impl_->goal_poses_writer) {
    automsgs::msgs::geometry_msgs::PoseStampedArray empty;
    empty.mutable_header()->set_frame_id(defaultFrame().toStdString());
    *empty.mutable_header()->mutable_stamp() =
        automsgs::msgs::builtin_interfaces::TimeNow();
    impl_->goal_poses_writer->Write(
        std::make_shared<automsgs::msgs::geometry_msgs::PoseStampedArray>(
            empty));
  }
}

void NavigationPanel::requestTaskCancel() {
  ++impl_->cancel_epoch;
  impl_->suppress_nav_feedback = true;
  const int epoch = impl_->cancel_epoch;
  publishCancelNavigation();
  // /goal_poses and 2D Goal Pose retry at 50/200 ms. Re-send cancel after
  // that window so a late START cannot outrun Cancel.
  for (int delay_ms : {80, 200, 350, 550}) {
    QTimer::singleShot(delay_ms, this, [this, epoch]() {
      if (impl_->cancel_epoch == epoch) {
        publishCancelNavigation();
      }
    });
  }
}

void NavigationPanel::clearNavVisuals() {
  ensureClients();
  if (impl_->plan_clear_writer) {
    automsgs::msgs::nav_msgs::Path empty;
    empty.mutable_header()->set_frame_id(defaultFrame().toStdString());
    *empty.mutable_header()->mutable_stamp() =
        automsgs::msgs::builtin_interfaces::TimeNow();
    impl_->plan_clear_writer->Write(empty);
  }
  tools::GoalUpdater.clearGoal();
}

void NavigationPanel::cancelAllGoals() {
  cancelActionGoals();
  requestTaskCancel();
}

void NavigationPanel::onNavigationFeedback(int status, float distance_remaining,
                                          int poses_remaining) {
  namespace nav_proto = autonomy::task::proto;
  const bool through = mode_ == UiMode::kNavThroughPoses;
  const bool following = mode_ == UiMode::kFollowingWp;
  switch (status) {
    case nav_proto::NAV_STATUS_PLANNING:
    case nav_proto::NAV_STATUS_NAVIGATING:
      if (impl_->suppress_nav_feedback) {
        break;
      }
      setGoalStatus(2);
      setNavFeedback(0.0, distance_remaining, 0.0, 0,
                     through ? poses_remaining : -1);
      if (mode_ == UiMode::kIdle || mode_ == UiMode::kPaused) {
        enterMode(UiMode::kRunning);
      }
      if ((through || following) && poses_remaining >= 0 &&
          !impl_->accumulated_poses.empty()) {
        const int n = static_cast<int>(impl_->accumulated_poses.size());
        int current = n - poses_remaining;
        if (current < 0) {
          current = 0;
        }
        if (current >= n) {
          current = n - 1;
        }
        if (current != impl_->goal_index) {
          impl_->goal_index = current;
          updateWpNavigationMarkers();
        }
      }
      break;
    case nav_proto::NAV_STATUS_SUCCEEDED:
      setGoalStatus(4);
      if (following && impl_->sequential_wp &&
          !impl_->navigation_goal_handle) {
        startSequentialWaypoint(impl_->sequential_index + 1);
        break;
      }
      if (mode_ == UiMode::kRunning || through || following) {
        enterMode(UiMode::kIdle);
      }
      break;
    case nav_proto::NAV_STATUS_CANCELED:
      setGoalStatus(5);
      clearNavVisuals();
      if (mode_ == UiMode::kRunning || through || following) {
        enterMode(UiMode::kIdle);
      }
      break;
    case nav_proto::NAV_STATUS_FAILED:
      setGoalStatus(6);
      if (mode_ == UiMode::kRunning || through || following) {
        enterMode(UiMode::kIdle);
      }
      break;
    case nav_proto::NAV_STATUS_PAUSED:
      if (mode_ == UiMode::kRunning) {
        enterMode(UiMode::kPaused);
      }
      break;
    default:
      break;
  }
}

bool NavigationPanel::publishGoalPose(const PoseStamped& pose) {
  ensureClients();
  if (!impl_->goal_pose_writer) {
    return false;
  }
  const int epoch = impl_->cancel_epoch;
  const bool ok =
      impl_->goal_pose_writer->Write(std::make_shared<PoseStamped>(pose));
  QTimer::singleShot(50, this, [this, pose, epoch]() {
    if (impl_->cancel_epoch != epoch || !impl_->goal_pose_writer) {
      return;
    }
    impl_->goal_pose_writer->Write(std::make_shared<PoseStamped>(pose));
  });
  QTimer::singleShot(200, this, [this, pose, epoch]() {
    if (impl_->cancel_epoch != epoch || !impl_->goal_pose_writer) {
      return;
    }
    impl_->goal_pose_writer->Write(std::make_shared<PoseStamped>(pose));
  });
  return ok;
}

bool NavigationPanel::publishGoalPoses(const std::vector<PoseStamped>& poses) {
  ensureClients();
  if (!impl_->goal_poses_writer || poses.empty()) {
    return false;
  }
  automsgs::msgs::geometry_msgs::PoseStampedArray array;
  *array.mutable_header() = poses.front().header();
  for (const auto& pose : poses) {
    *array.add_poses() = pose.pose();
  }
  const int epoch = impl_->cancel_epoch;
  const bool ok = impl_->goal_poses_writer->Write(
      std::make_shared<automsgs::msgs::geometry_msgs::PoseStampedArray>(array));
  QTimer::singleShot(50, this, [this, array, epoch]() {
    if (impl_->cancel_epoch != epoch || !impl_->goal_poses_writer) {
      return;
    }
    impl_->goal_poses_writer->Write(
        std::make_shared<automsgs::msgs::geometry_msgs::PoseStampedArray>(
            array));
  });
  QTimer::singleShot(200, this, [this, array, epoch]() {
    if (impl_->cancel_epoch != epoch || !impl_->goal_poses_writer) {
      return;
    }
    impl_->goal_poses_writer->Write(
        std::make_shared<automsgs::msgs::geometry_msgs::PoseStampedArray>(
            array));
  });
  return ok;
}

void NavigationPanel::startNavigation(const PoseStamped& pose) {
  ensureClients();
  impl_->suppress_nav_feedback = false;
  const bool keep_sequential = impl_->sequential_wp;
  cancelActionGoals();
  ++impl_->cancel_epoch;
  impl_->sequential_wp = keep_sequential;

  if (!impl_->navigation_client ||
      !impl_->navigation_client->ActionServerIsReady()) {
    if (!publishGoalPose(pose)) {
      setWaypointNote(tr("Failed to publish /goal_pose"));
      return;
    }
    setGoalStatus(2);
    if (!keep_sequential) {
      enterMode(UiMode::kRunning);
    }
    return;
  }
  NavigateToPoseAction::Goal goal;
  *goal.mutable_pose() = pose;
  goal.set_behavior_tree(behavior_tree_file_->text().toStdString());

  NavToPoseClient::SendGoalOptions options;
  options.feedback_callback =
      [this](std::shared_ptr<NavToPoseClient::GoalHandle>,
             std::shared_ptr<const NavigateToPoseAction::Feedback> feedback) {
        if (!feedback) {
          return;
        }
        const double eta = DurationSeconds(feedback->estimated_time_remaining());
        const double elapsed = DurationSeconds(feedback->navigation_time());
        const float distance = feedback->distance_remaining();
        const int recoveries = feedback->number_of_recoveries();
        runOnUi([this, eta, elapsed, distance, recoveries]() {
          setGoalStatus(2);
          setNavFeedback(eta, distance, elapsed, recoveries);
        });
      };
  options.result_callback = [this](const NavToPoseClient::WrappedResult& result) {
    const int8_t code = static_cast<int8_t>(result.code);
    runOnUi([this, code]() {
      impl_->navigation_goal_handle.reset();
      setGoalStatus(code);
      if (impl_->sequential_wp && mode_ == UiMode::kFollowingWp &&
          code == static_cast<int8_t>(autolink::action::ResultCode::SUCCEEDED)) {
        startSequentialWaypoint(impl_->sequential_index + 1);
        return;
      }
      impl_->sequential_wp = false;
      if (mode_ == UiMode::kRunning || mode_ == UiMode::kFollowingWp) {
        enterMode(UiMode::kIdle);
      }
    });
  };
  options.goal_response_callback =
      [this](std::shared_ptr<NavToPoseClient::GoalHandle> handle) {
        runOnUi([this, handle]() {
          impl_->navigation_goal_handle = handle;
          if (!handle) {
            impl_->sequential_wp = false;
            setWaypointNote(tr("NavigateToPose goal was rejected"));
            enterMode(UiMode::kIdle);
            return;
          }
          setGoalStatus(2);
          if (!impl_->sequential_wp) {
            enterMode(UiMode::kRunning);
          }
        });
      };
  impl_->navigation_client->AsyncSendGoal(goal, options);
  if (!impl_->sequential_wp) {
    enterMode(UiMode::kRunning);
  }
}

void NavigationPanel::startNavThroughPoses() {
  ensureClients();
  impl_->suppress_nav_feedback = false;
  cancelActionGoals();
  ++impl_->cancel_epoch;
  if (!publishGoalPoses(impl_->accumulated_poses)) {
    setWaypointNote(tr("Failed to publish /goal_poses"));
    return;
  }
  impl_->goal_index = 0;
  setGoalStatus(2);
  setWaypointNote(tr("Through poses via /goal_poses (%n)", "",
                     static_cast<int>(impl_->accumulated_poses.size())));
  enterMode(UiMode::kNavThroughPoses);
  updateWpNavigationMarkers();
}

void NavigationPanel::startWaypointFollowing() {
  ensureClients();
  impl_->suppress_nav_feedback = false;
  if (impl_->store_initial_pose && !impl_->initial_pose_stored) {
    PoseStamped initial;
    if (!lookupInitialPose(&initial)) {
      setWaypointNote(tr("Could not transform %1 to %2")
                          .arg(QString::fromStdString(impl_->base_frame), defaultFrame()));
      return;
    }
    impl_->accumulated_poses.insert(impl_->accumulated_poses.begin(), initial);
    syncTabsWithAccumulatedPoses();
    updateWpNavigationMarkers();
    impl_->initial_pose_stored = true;
    if (impl_->loop_count == 0) {
      impl_->goal_index = 1;
    }
  }
  impl_->stored_poses = impl_->accumulated_poses;

  if (impl_->waypoint_client && impl_->waypoint_client->ActionServerIsReady()) {
    cancelActionGoals();
    ++impl_->cancel_epoch;
    FollowWaypointsAction::Goal goal;
    goal.set_number_of_loops(static_cast<uint32_t>(std::max(0, loop_no_)));
    goal.set_goal_index(static_cast<uint32_t>(std::max(0, impl_->goal_index)));
    for (const auto& pose : impl_->accumulated_poses) {
      *goal.add_poses() = pose;
    }
    FollowWaypointsClient::SendGoalOptions options;
    options.feedback_callback =
        [this](std::shared_ptr<FollowWaypointsClient::GoalHandle>,
               std::shared_ptr<const FollowWaypointsAction::Feedback> feedback) {
          if (!feedback) {
            return;
          }
          const uint32_t current = feedback->current_waypoint();
          runOnUi([this, current]() {
            const int next_index = static_cast<int>(current);
            const bool index_changed = next_index != impl_->goal_index;
            impl_->goal_index = next_index;
            setGoalStatus(2);
            setWaypointNote(tr("Waypoint %1 / %2")
                                .arg(current + 1)
                                .arg(impl_->accumulated_poses.size()));
            if (index_changed) {
              updateWpNavigationMarkers();
            }
          });
        };
    options.result_callback =
        [this](const FollowWaypointsClient::WrappedResult& result) {
          const int8_t code = static_cast<int8_t>(result.code);
          runOnUi([this, code]() {
            impl_->waypoint_goal_handle.reset();
            setGoalStatus(code);
            if (mode_ == UiMode::kFollowingWp) {
              enterMode(UiMode::kIdle);
            }
          });
        };
    options.goal_response_callback =
        [this](std::shared_ptr<FollowWaypointsClient::GoalHandle> handle) {
          runOnUi([this, handle]() {
            impl_->waypoint_goal_handle = handle;
            if (!handle) {
              setWaypointNote(tr("FollowWaypoints goal was rejected"));
              enterMode(UiMode::kIdle);
            }
          });
        };
    impl_->waypoint_client->AsyncSendGoal(goal, options);
    enterMode(UiMode::kFollowingWp);
    updateWpNavigationMarkers();
    return;
  }

  impl_->sequential_wp = true;
  impl_->sequential_index = static_cast<size_t>(std::max(0, impl_->goal_index));
  impl_->loop_count = 0;
  enterMode(UiMode::kFollowingWp);
  startSequentialWaypoint(impl_->sequential_index);
}

void NavigationPanel::startSequentialWaypoint(size_t index) {
  if (!impl_->sequential_wp || mode_ != UiMode::kFollowingWp) {
    return;
  }
  if (impl_->accumulated_poses.empty()) {
    impl_->sequential_wp = false;
    enterMode(UiMode::kIdle);
    return;
  }
  if (index >= impl_->accumulated_poses.size()) {
    ++impl_->loop_count;
    if (impl_->loop_count > loop_no_) {
      impl_->sequential_wp = false;
      setGoalStatus(4);
      enterMode(UiMode::kIdle);
      return;
    }
    index = impl_->store_initial_pose && !impl_->accumulated_poses.empty() ? 1 : 0;
    if (index >= impl_->accumulated_poses.size()) {
      index = 0;
    }
  }
  impl_->sequential_index = index;
  impl_->goal_index = static_cast<int>(index);
  updateWpNavigationMarkers();
  setWaypointNote(tr("Waypoint %1 / %2 (loop %3/%4)")
                      .arg(index + 1)
                      .arg(impl_->accumulated_poses.size())
                      .arg(impl_->loop_count)
                      .arg(loop_no_));
  startNavigation(impl_->accumulated_poses[index]);
}

void NavigationPanel::setGoalStatus(int8_t status) {
  goal_badge_->SetText(GoalBadgeText(status));
  goal_badge_->SetStatus(GoalBadgeStatus(status));
}

void NavigationPanel::setNavFeedback(double eta_s, double distance_m,
                                    double elapsed_s, int recoveries,
                                    int poses_remaining) {
  eta_value_->setText(FormatSecondsChip(eta_s));
  distance_value_->setText(FormatMetersChip(distance_m));
  elapsed_value_->setText(FormatSecondsChip(elapsed_s));
  recoveries_value_->setText(QString::number(std::max(0, recoveries)));
  if (poses_remaining >= 0) {
    setWaypointNote(tr("%n poses remaining", "", poses_remaining));
  }
}

void NavigationPanel::setWaypointNote(const QString& note) {
  if (note.isEmpty()) {
    note_bar_->hide();
    note_bar_->SetMessage(QString());
    return;
  }
  note_bar_->SetLevel(NoteLevel(note));
  note_bar_->SetMessage(note);
  note_bar_->show();
}

void NavigationPanel::runOnUi(std::function<void()> fn) {
  if (!fn) {
    return;
  }
  if (QThread::currentThread() == thread()) {
    fn();
    return;
  }
  QPointer<NavigationPanel> guard(this);
  QMetaObject::invokeMethod(
      this,
      [guard, fn = std::move(fn)]() {
        if (!guard) {
          return;
        }
        fn();
      },
      Qt::QueuedConnection);
}

}  // namespace navigation
}  // namespace autoviz
