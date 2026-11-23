// Copyright 2026 The Openbot Authors (duyongquan).
// SPDX-License-Identifier: Apache-2.0
/** @file coverage_panel.cpp
 * @brief Region coverage panel implementation. */
#include "autoviz/ui/navigation/coverage_panel.hpp"

#ifdef AUTONOMY_BUILD_COVERAGE

#include <QAbstractItemView>
#include <QAction>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <array>
#include <QColor>
#include <QAbstractSpinBox>
#include <QFontDatabase>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPlainTextEdit>
#include <QPointer>
#include <QProgressBar>
#include <QPushButton>
#include <QRegularExpression>
#include <QScrollArea>
#include <QKeySequence>
#include <QShortcut>
#include <QSignalBlocker>
#include <QTime>
#include <QTimer>
#include <QToolButton>
#include <QUuid>
#include <QVBoxLayout>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

#include "autoviz/common/session_config.hpp"
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/plugins/displays/display.hpp"
#include "autoviz/plugins/displays/map/map_display.hpp"
#include "autoviz/ui/navigation/coverage_obstacles.hpp"
#include "autoviz/common/widget/feedback/info_bar.hpp"
#include "autoviz/ui/theme/panel.hpp"
#include "autoviz/ui/theme/style.hpp"
#include "automsgs/msgs/geometry_msgs/point_stamped.pb.h"
#include "automsgs/msgs/visualization_msgs/marker_array.pb.h"

namespace autoviz::navigation {
namespace {

constexpr const char* kFieldKind = "field";
constexpr const char* kObstacleKind = "obstacle";
constexpr const char* kLiveKind = "live";

/** @brief Short one-line description of a ring for the list widget. */
/**
 * @brief Metric chip matching the Navigation tab status card.
 * @param caption Chip caption.
 * @param[out] value_out Receives the value label.
 * @param parent Parent widget.
 * @return New metric chip frame.
 */
QFrame* MakeCoverageMetricChip(const QString& caption, QLabel** value_out,
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
  auto* value = new QLabel(QStringLiteral("\u2014"), chip);
  value->setObjectName(QStringLiteral("NavigationMetricValue"));
  value->setStyleSheet(style::sheet(QStringLiteral("navigation/metric_value")));
  layout->addWidget(caption_label);
  layout->addWidget(value);
  chip->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  *value_out = value;
  return chip;
}

/**
 * @brief Compact ghost button matching the Navigation tab actions.
 * @param text Button label.
 * @param parent Parent widget.
 * @return New flat button sized like NavigationPanel compact actions.
 */
QPushButton* MakeCoverageCompactGhost(const QString& text, QWidget* parent) {
  QPushButton* button = MakeFlatActionButton(text, parent);
  button->setMinimumHeight(28);
  return button;
}

QString RingLabel(const QString& name, std::size_t points) {
  return QObject::tr("%1 · %2 pts").arg(name).arg(points);
}

}  // namespace

CoveragePanel::CoveragePanel(common::VisualizationManager* manager,
                             QWidget* parent)
    : QWidget(parent), manager_(manager) {
  setObjectName(QStringLiteral("CoveragePanel"));
  rings_.push_back({tr("Work area"), QString::fromLatin1(kFieldKind),
                    {{-3.0, -2.0}, {3.0, -2.0}, {3.0, 2.0}, {-3.0, 2.0}}});
  BuildUi();
}

CoveragePanel::~CoveragePanel() { CancelCoverage(); }

void CoveragePanel::BuildUi() {
  setObjectName(QStringLiteral("CoveragePanel"));
  auto* outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 2, 0, 0);
  outer->setSpacing(8);

  // Status card mirrors the AB/Waypoints tab: metric chips above an info bar.
  auto* status_card = new QFrame(this);
  status_card->setObjectName(QStringLiteral("NavigationStatusCard"));
  status_card->setAttribute(Qt::WA_StyledBackground, true);
  status_card->setStyleSheet(
      style::sheet(QStringLiteral("navigation.status_card")));
  auto* status_layout = new QVBoxLayout(status_card);
  status_layout->setContentsMargins(8, 8, 8, 8);
  status_layout->setSpacing(6);
  auto* metrics = new QGridLayout();
  metrics->setHorizontalSpacing(6);
  metrics->setVerticalSpacing(6);
  metrics->addWidget(
      MakeCoverageMetricChip(tr("Progress"), &progress_value_, status_card), 0,
      0);
  metrics->addWidget(
      MakeCoverageMetricChip(tr("Remaining"), &remaining_value_, status_card), 0,
      1);
  status_layout->addLayout(metrics);
  note_bar_ = new widget::InfoBar(status_card);
  note_bar_->setObjectName(QStringLiteral("coverageStatus"));
  note_bar_->SetClosable(false);
  note_bar_->SetTone(style::Tone::Shell);
  note_bar_->SetTitle(tr("Coverage"));
  note_bar_->SetMessage(tr("Waiting for the coverage server"));
  status_layout->addWidget(note_bar_);
  outer->addWidget(status_card);

  auto* hint = new QLabel(
      tr("Click Draw obstacle, then click 3+ points in the 3D view. "
         "Click the first point again or press Enter to finish."),
      this);
  StyleHintLabel(hint);
  hint->setWordWrap(true);
  outer->addWidget(hint);

  auto* scroll = new QScrollArea(this);
  scroll->setObjectName(QStringLiteral("AutovizSettingsScroll"));
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  outer->addWidget(scroll, 1);
  auto* content = new QWidget(scroll);
  scroll->setWidget(content);
  auto* layout = new QVBoxLayout(content);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(8);

  auto* settings_title = new QLabel(tr("Coverage settings"), content);
  StyleSectionTitle(settings_title);
  layout->addWidget(settings_title);

  auto* form = new QFormLayout();
  ApplyCompactForm(form);
  frame_ = new QLineEdit(QStringLiteral("map"), content);
  frame_->setObjectName(QStringLiteral("coverageFrame"));
  StyleCompactSettingsField(frame_);
  form->addRow(MakeSettingsFormLabel(tr("Frame"), content), frame_);

  radius_ = new QDoubleSpinBox(content);
  radius_->setObjectName(QStringLiteral("coverageRobotRadius"));
  radius_->setRange(0.05, 5.0);
  radius_->setDecimals(2);
  radius_->setSingleStep(0.05);
  radius_->setValue(0.4);
  radius_->setSuffix(tr(" m"));
  radius_->setButtonSymbols(QAbstractSpinBox::NoButtons);
  radius_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  StyleCompactSettingsField(radius_);
  radius_->setToolTip(
      tr("Cleaning radius; the swath width is twice this value."));
  form->addRow(MakeSettingsFormLabel(tr("Robot radius"), content), radius_);

  swath_ = new QDoubleSpinBox(content);
  swath_->setObjectName(QStringLiteral("coverageSwathSpacing"));
  swath_->setRange(0.05, 5.0);
  swath_->setDecimals(2);
  swath_->setSingleStep(0.01);
  swath_->setValue(0.15);
  swath_->setSuffix(tr(" m"));
  swath_->setButtonSymbols(QAbstractSpinBox::NoButtons);
  swath_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  StyleCompactSettingsField(swath_);
  swath_->setToolTip(
      tr("Centre-to-centre distance between neighbouring coverage passes. "
         "0.15 m keeps the lines dense; raise it towards twice the robot radius for a wider sweep."));
  form->addRow(MakeSettingsFormLabel(tr("Swath spacing"), content),
               swath_);


  headland_ = new QDoubleSpinBox(content);
  headland_->setObjectName(QStringLiteral("coverageHeadland"));
  headland_->setRange(0.1, 20.0);
  headland_->setDecimals(2);
  headland_->setSingleStep(0.05);
  headland_->setValue(1.2);
  headland_->setSuffix(tr(" m"));
  headland_->setButtonSymbols(QAbstractSpinBox::NoButtons);
  headland_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  StyleCompactSettingsField(headland_);
  form->addRow(MakeSettingsFormLabel(tr("Headland"), content), headland_);

  route_mode_ = new QComboBox(content);
  route_mode_->setObjectName(QStringLiteral("coverageRoute"));
  route_mode_->addItem(tr("Boustrophedon"), QStringLiteral("BOUSTROPHEDON"));
  route_mode_->addItem(tr("Snake"), QStringLiteral("SNAKE"));
  route_mode_->addItem(tr("Spiral"), QStringLiteral("SPIRAL"));
  route_mode_->addItem(tr("TSP"), QStringLiteral("TSP"));
  StyleCompactSettingsField(route_mode_);
  form->addRow(MakeSettingsFormLabel(tr("Route"), content), route_mode_);

  path_mode_ = new QComboBox(content);
  path_mode_->setObjectName(QStringLiteral("coverageTurns"));
  path_mode_->addItem(tr("Dubins"), QStringLiteral("DUBIN"));
  path_mode_->addItem(tr("Reeds-Shepp"), QStringLiteral("REEDS_SHEPP"));
  StyleCompactSettingsField(path_mode_);
  form->addRow(MakeSettingsFormLabel(tr("Turns"), content), path_mode_);
  layout->addLayout(form);

  auto* field_title = new QLabel(tr("Field and obstacles"), content);
  StyleSectionTitle(field_title);
  layout->addWidget(field_title);
  ring_list_ = new QListWidget(content);
  ring_list_->setObjectName(QStringLiteral("coverageRings"));
  ring_list_->setSelectionMode(QAbstractItemView::SingleSelection);
  ring_list_->setAlternatingRowColors(true);
  ring_list_->setMinimumHeight(88);
  ring_list_->setMaximumHeight(132);
  StyleFrostedPanelList(ring_list_);
  layout->addWidget(ring_list_);
  rings_summary_ = new QLabel(content);
  StyleHintLabel(rings_summary_);
  layout->addWidget(rings_summary_);

  auto* draw_row = new QHBoxLayout();
  draw_row->setSpacing(6);
  draw_obstacle_ = MakeCoverageCompactGhost(tr("Draw obstacle"), content);
  draw_obstacle_->setObjectName(QStringLiteral("coverageDrawObstacle"));
  draw_obstacle_->setToolTip(
      tr("Press Ctrl+Alt+O, then click at least three points in the 3D view. "
         "Enter finishes the obstacle, Esc cancels."));
  add_obstacle_ = MakeCoverageCompactGhost(tr("Add square"), content);
  add_obstacle_->setObjectName(QStringLiteral("coverageAddObstacle"));
  add_obstacle_->setToolTip(
      tr("Append a square obstacle at the field centre."));
  draw_row->addWidget(draw_obstacle_, 2);
  draw_row->addWidget(add_obstacle_, 1);
  layout->addLayout(draw_row);

  auto* ring_buttons = new QHBoxLayout();
  ring_buttons->setSpacing(6);
  remove_ring_ = MakeDestructiveFlatActionButton(tr("Remove"), content);
  remove_ring_->setObjectName(QStringLiteral("coverageRemoveObstacle"));
  clear_obstacles_ = MakeCoverageCompactGhost(tr("Clear"), content);
  clear_obstacles_->setObjectName(QStringLiteral("coverageClearObstacles"));
  clear_obstacles_->setToolTip(tr("Remove every manual and live obstacle."));
  ring_buttons->addWidget(remove_ring_);
  ring_buttons->addWidget(clear_obstacles_);
  ring_buttons->addStretch(1);
  layout->addLayout(ring_buttons);

  advanced_ = new QPushButton(tr("Advanced JSON"), content);
  advanced_->setObjectName(QStringLiteral("coverageAdvanced"));
  advanced_->setCheckable(true);
  advanced_->setCursor(Qt::PointingHandCursor);
  StylePanelButton(advanced_, QStringLiteral("ghost"));
  layout->addWidget(advanced_, 0, Qt::AlignLeft);
  polygons_ = new QPlainTextEdit(content);
  polygons_->setObjectName(QStringLiteral("coveragePolygons"));
  polygons_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  polygons_->setMinimumHeight(80);
  polygons_->setMaximumHeight(120);
  polygons_->setPlaceholderText(
      QStringLiteral("[[[x,y], ...], [obstacle x,y, ...]]"));
  polygons_->setVisible(false);
  layout->addWidget(polygons_);

  auto* rooms_title = new QLabel(tr("Rooms"), content);
  StyleSectionTitle(rooms_title);
  layout->addWidget(rooms_title);
  auto* rooms_row = new QHBoxLayout();
  rooms_row->setSpacing(6);
  room_columns_ = new QSpinBox(content);
  room_columns_->setRange(1, 4);
  room_columns_->setValue(2);
  StyleCompactSettingsField(room_columns_);
  room_columns_->setMaximumWidth(56);
  room_rows_ = new QSpinBox(content);
  room_rows_->setRange(1, 4);
  room_rows_->setValue(2);
  StyleCompactSettingsField(room_rows_);
  room_rows_->setMaximumWidth(56);
  auto* grid_label = MakeSettingsFormLabel(tr("Grid"), content);
  rooms_row->addWidget(grid_label);
  rooms_row->addWidget(room_columns_);
  rooms_row->addWidget(new QLabel(QStringLiteral("×"), content));
  rooms_row->addWidget(room_rows_);
  split_rooms_ = MakeCoverageCompactGhost(tr("Split map"), content);
  split_rooms_->setObjectName(QStringLiteral("coverageSplitRooms"));
  split_rooms_->setToolTip(
      tr("Split the work area into colored rooms and plan each one."));
  rooms_row->addWidget(split_rooms_, 1);
  map_rooms_ = MakeCoverageCompactGhost(tr("From map"), content);
  map_rooms_->setObjectName(QStringLiteral("coverageMapRooms"));
  map_rooms_->setToolTip(tr("Split live /map free space into rooms."));
  rooms_row->addWidget(map_rooms_);
  layout->addLayout(rooms_row);
  rooms_ = new QListWidget(content);
  rooms_->setObjectName(QStringLiteral("coverageRooms"));
  rooms_->setMinimumHeight(64);
  rooms_->setMaximumHeight(96);
  StyleFrostedPanelList(rooms_);
  layout->addWidget(rooms_);
  current_region_ = new QLabel(tr("Current room: —"), content);
  current_region_->setObjectName(QStringLiteral("coverageCurrentRoom"));
  StyleHintLabel(current_region_);
  layout->addWidget(current_region_);
  clean_rooms_ = MakePrimaryActionButton(tr("Clean all rooms"), content);
  clean_rooms_->setObjectName(QStringLiteral("coverageCleanRooms"));
  clean_rooms_->setEnabled(false);
  layout->addWidget(clean_rooms_);

  auto* live_title = new QLabel(tr("Live obstacles"), content);
  StyleSectionTitle(live_title);
  layout->addWidget(live_title);
  live_obstacles_ = new QCheckBox(tr("Capture from /map"), content);
  live_obstacles_->setObjectName(QStringLiteral("coverageLiveObstacles"));
  live_obstacles_->setToolTip(
      tr("Read occupied cells from the newest /map message. Static walls "
         "larger than the size limit are skipped; small blobs become "
         "exclusion rings."));
  layout->addWidget(live_obstacles_);
  auto* live_row = new QHBoxLayout();
  live_row->setSpacing(6);
  live_row->addWidget(MakeSettingsFormLabel(tr("Margin"), content));
  obstacle_margin_ = new QDoubleSpinBox(content);
  obstacle_margin_->setObjectName(QStringLiteral("coverageObstacleMargin"));
  obstacle_margin_->setRange(0.0, 2.0);
  obstacle_margin_->setDecimals(2);
  obstacle_margin_->setSingleStep(0.05);
  obstacle_margin_->setValue(0.2);
  obstacle_margin_->setSuffix(tr(" m"));
  obstacle_margin_->setButtonSymbols(QAbstractSpinBox::NoButtons);
  obstacle_margin_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  StyleCompactSettingsField(obstacle_margin_);
  obstacle_margin_->setToolTip(tr("Clearance added around every live blob."));
  refresh_obstacles_ = MakeCoverageCompactGhost(tr("Refresh"), content);
  refresh_obstacles_->setObjectName(
      QStringLiteral("coverageRefreshObstacles"));
  live_row->addWidget(obstacle_margin_, 1);
  live_row->addWidget(refresh_obstacles_);
  layout->addLayout(live_row);
  obstacle_status_ = new QLabel(tr("Live capture is off"), content);
  obstacle_status_->setObjectName(QStringLiteral("coverageObstacleStatus"));
  obstacle_status_->setWordWrap(true);
  StyleHintLabel(obstacle_status_);
  layout->addWidget(obstacle_status_);
  auto_replan_ = new QCheckBox(tr("Re-plan when obstacles change"), content);
  auto_replan_->setObjectName(QStringLiteral("coverageAutoReplan"));
  auto_replan_->setEnabled(false);
  layout->addWidget(auto_replan_);
  layout->addStretch(1);

  // Fixed footer mirrors the AB/Waypoints action strip.
  auto* footer = new QFrame(this);
  footer->setObjectName(QStringLiteral("NavigationFooter"));
  footer->setAttribute(Qt::WA_StyledBackground, true);
  footer->setStyleSheet(style::sheet(QStringLiteral("navigation/footer")));
  auto* footer_layout = new QVBoxLayout(footer);
  footer_layout->setContentsMargins(10, 8, 10, 10);
  footer_layout->setSpacing(6);
  progress_ = new QProgressBar(footer);
  progress_->setObjectName(QStringLiteral("coverageProgress"));
  progress_->setRange(0, 100);
  progress_->setValue(0);
  progress_->setTextVisible(false);
  progress_->setMaximumHeight(6);
  progress_->setVisible(false);
  footer_layout->addWidget(progress_);
  approach_ = new QCheckBox(tr("Navigate to the coverage start"), footer);
  approach_->setObjectName(QStringLiteral("coverageApproach"));
  approach_->setChecked(true);
  footer_layout->addWidget(approach_);
  auto* footer_buttons = new QHBoxLayout();
  footer_buttons->setSpacing(8);
  preview_ = MakeCoverageCompactGhost(tr("Preview"), footer);
  preview_->setObjectName(QStringLiteral("coveragePreview"));
  preview_->setMinimumHeight(32);
  preview_->setToolTip(tr("Compute and publish the full coverage path."));
  cancel_ = MakeDestructiveFlatActionButton(tr("Cancel"), footer);
  cancel_->setObjectName(QStringLiteral("coverageCancel"));
  cancel_->setMinimumHeight(32);
  cancel_->setEnabled(false);
  start_ = MakePrimaryActionButton(tr("Start coverage"), footer);
  start_->setObjectName(QStringLiteral("coverageStart"));
  start_->setMinimumHeight(32);
  start_->setToolTip(tr("Compute and execute the complete coverage task."));
  footer_buttons->addWidget(preview_);
  footer_buttons->addWidget(cancel_);
  footer_buttons->addWidget(start_, 1);
  footer_layout->addLayout(footer_buttons);
  outer->addWidget(footer);

  connect(ring_list_, &QListWidget::currentRowChanged, this,
          &CoveragePanel::LoadSelectedRing);
  connect(add_obstacle_, &QPushButton::clicked, this,
          &CoveragePanel::AddObstacleRing);
  connect(draw_obstacle_, &QPushButton::clicked, this,
          &CoveragePanel::ToggleObstacleDrawing);
  connect(remove_ring_, &QPushButton::clicked, this,
          &CoveragePanel::RemoveSelectedRing);
  connect(clear_obstacles_, &QPushButton::clicked, this,
          &CoveragePanel::ClearObstacleRings);
  connect(advanced_, &QPushButton::toggled, this,
          &CoveragePanel::OnAdvancedToggled);
  connect(live_obstacles_, &QCheckBox::toggled, this, [this](bool enabled) {
    auto_replan_->setEnabled(enabled);
    if (!enabled) {
      auto_replan_->setChecked(false);
      live_timer_->stop();
      obstacle_status_->setText(tr("Live capture is off"));
    } else {
      live_timer_->start();
      RefreshLiveObstacles();
    }
  });
  connect(refresh_obstacles_, &QPushButton::clicked, this,
          &CoveragePanel::RefreshLiveObstacles);
  connect(obstacle_margin_, &QDoubleSpinBox::valueChanged, this, [this] {
    if (live_obstacles_->isChecked()) {
      RefreshLiveObstacles();
    }
  });
  live_timer_ = new QTimer(this);
  live_timer_->setInterval(1000);
  connect(live_timer_, &QTimer::timeout, this,
          &CoveragePanel::RefreshLiveObstacles);

  connect(split_rooms_, &QPushButton::clicked, this,
          [this] { SplitRooms(false); });
  connect(map_rooms_, &QPushButton::clicked, this,
          [this] { SplitRooms(true); });
  connect(clean_rooms_, &QPushButton::clicked, this,
          &CoveragePanel::StartAllRooms);
  connect(preview_, &QPushButton::clicked, this,
          &CoveragePanel::PreviewCoverage);
  connect(start_, &QPushButton::clicked, this, &CoveragePanel::StartCoverage);
  connect(cancel_, &QPushButton::clicked, this, &CoveragePanel::CancelCoverage);
  timer_ = new QTimer(this);
  timer_->setInterval(100);
  connect(timer_, &QTimer::timeout, this, &CoveragePanel::PollActions);
  timer_->start();

  // Deterministic demo/CI hook: once the live map is available, split it into
  // rooms automatically. The smoke test captures the coverage screenshot
  // without driving the pointer.
  if (qEnvironmentVariableIsSet("AUTOVIZ_COVERAGE_AUTOPLAN")) {
    auto* autoplan = new QTimer(this);
    autoplan->setInterval(1000);
    QPointer<CoveragePanel> self(this);
    connect(autoplan, &QTimer::timeout, this, [self, autoplan] {
      if (!self) {
        return;
      }
      display::MapDisplay* map_display = nullptr;
      for (auto* candidate : self->manager_->displays()) {
        auto* map = dynamic_cast<display::MapDisplay*>(candidate);
        if (map != nullptr && map->channel() == "/map") {
          map_display = map;
          break;
        }
      }
      if (map_display == nullptr || map_display->mapMessage() == nullptr) {
        return;
      }
      autoplan->stop();
      QTimer::singleShot(300, self.data(), [self] {
        if (self) {
          self->SplitRooms(true);
        }
      });
    });
    autoplan->start();
  }

  draw_shortcut_ =
      new QShortcut(QKeySequence(QStringLiteral("Ctrl+Alt+O")), this);
  draw_shortcut_->setContext(Qt::WindowShortcut);
  connect(draw_shortcut_, &QShortcut::activated, this,
          &CoveragePanel::ToggleObstacleDrawing);
  finish_shortcut_ = new QShortcut(this);
  finish_shortcut_->setKeys({QKeySequence(Qt::Key_Return),
                             QKeySequence(Qt::Key_Enter)});
  finish_shortcut_->setContext(Qt::WindowShortcut);
  finish_shortcut_->setEnabled(false);
  connect(finish_shortcut_, &QShortcut::activated, this,
          &CoveragePanel::FinishObstacleDraft);
  cancel_shortcut_ = new QShortcut(QKeySequence(Qt::Key_Escape), this);
  cancel_shortcut_->setContext(Qt::WindowShortcut);
  cancel_shortcut_->setEnabled(false);
  connect(cancel_shortcut_, &QShortcut::activated, this,
          &CoveragePanel::CancelObstacleDraft);

  RebuildRingList(0);
}

void CoveragePanel::EnsureClients() {
  if (manager_ == nullptr || manager_->autolinkNode() == nullptr) {
    return;
  }
  auto node = manager_->autolinkNode();
  if (!compute_client_) {
    compute_client_ =
        std::make_shared<ComputeClient>(node, "/compute_coverage_path");
  }
  if (!navigate_client_) {
    navigate_client_ =
        std::make_shared<NavigateClient>(node, "/navigate_complete_coverage");
  }
  if (!preview_writer_) {
    preview_writer_ =
        node->CreateWriter<automsgs::msgs::nav_msgs::Path>("/coverage/plan");
  }
  bool exists = false;
  for (const auto* display : manager_->displays()) {
    if (display != nullptr && display->channel() == "/coverage/plan") {
      exists = true;
    }
  }
  if (!exists) {
    common::DisplayConfig config;
    config.type = "Path";
    config.name = "Full coverage path";
    config.channel = "/coverage/plan";
    config.enabled = true;
    config.properties["color"] = "255;215;70";
    config.properties["line_width"] = "0.10";
    config.properties["alpha"] = "0.95";
    config.properties["offset"] = "0;0;0.10";
    manager_->addDisplay(config);
  }
  bool markers_exist = false;
  for (const auto* display : manager_->displays()) {
    if (display != nullptr && display->channel() == "/coverage/markers") {
      markers_exist = true;
    }
  }
  if (!markers_exist) {
    common::DisplayConfig config;
    config.type = "MarkerArray";
    config.name = "Coverage field and swaths";
    config.channel = "/coverage/markers";
    config.enabled = true;
    config.properties["use_custom_color"] = "false";
    manager_->addDisplay(config);
  }
  if (!odom_reader_) {
    QPointer<CoveragePanel> self(this);
    odom_reader_ = node->CreateReader<automsgs::msgs::nav_msgs::Odometry>(
        "/odom", [self](const std::shared_ptr<automsgs::msgs::nav_msgs::Odometry>& msg) {
          if (!self || !msg) return;
          const double x = msg->pose().pose().pose().position().x();
          const double y = msg->pose().pose().pose().position().y();
          QMetaObject::invokeMethod(self, [self, x, y] {
            if (!self) {
              return;
            }
            self->robot_x_ = x;
            self->robot_y_ = y;
            self->have_robot_pose_ = true;
            self->UpdateCurrentRegion(x, y);
          }, Qt::QueuedConnection);
        });
  }
  if (!sketch_writer_) {
    sketch_writer_ = node->CreateWriter<
        automsgs::msgs::visualization_msgs::MarkerArray>("/coverage/sketch");
  }
  bool rooms_exist = false;
  for (const auto* display : manager_->displays()) {
    if (display != nullptr && display->channel() == "/coverage/regions") {
      rooms_exist = true;
    }
  }
  if (!rooms_exist) {
    common::DisplayConfig config;
    config.type = "MarkerArray";
    config.name = "Coverage rooms";
    config.channel = "/coverage/regions";
    config.enabled = true;
    config.properties["use_custom_color"] = "false";
    manager_->addDisplay(config);
  }
  bool sketch_exists = false;
  for (const auto* display : manager_->displays()) {
    if (display != nullptr && display->channel() == "/coverage/sketch") {
      sketch_exists = true;
    }
  }
  if (!sketch_exists) {
    common::DisplayConfig config;
    config.type = "MarkerArray";
    config.name = "Coverage obstacles";
    config.channel = "/coverage/sketch";
    config.enabled = true;
    config.properties["use_custom_color"] = "false";
    manager_->addDisplay(config);
  }
}

int CoveragePanel::FieldRingIndex() const {
  for (int index = 0; index < static_cast<int>(rings_.size()); ++index) {
    if (rings_[index].kind == QLatin1String(kFieldKind)) {
      return index;
    }
  }
  return -1;
}

void CoveragePanel::RebuildRingList(int selected) {
  {
    const QSignalBlocker blocker(ring_list_);
    ring_list_->clear();
    for (const auto& ring : rings_) {
      auto* item =
          new QListWidgetItem(RingLabel(ring.name, ring.points.size()),
                              ring_list_);
      item->setToolTip(tr("%1 · %2 vertices")
                           .arg(ring.name)
                           .arg(ring.points.size()));
      if (ring.kind == QLatin1String(kFieldKind)) {
        item->setForeground(QColor(QStringLiteral("#1f4fd8")));
      } else if (ring.kind == QLatin1String(kLiveKind)) {
        item->setForeground(QColor(QStringLiteral("#0f8b8d")));
      } else {
        item->setForeground(QColor(QStringLiteral("#b9770e")));
      }
    }
    int obstacles = 0;
    for (const auto& ring : rings_) {
      if (ring.kind != QLatin1String(kFieldKind)) {
        ++obstacles;
      }
    }
    rings_summary_->setText(
        obstacles == 0 ? tr("No obstacles · click Draw obstacle to add")
                       : tr("%1 obstacle ring(s)").arg(obstacles));
    if (ring_list_->count() > 0) {
      ring_list_->setCurrentRow(
          std::clamp(selected, 0, ring_list_->count() - 1));
    }
  }
  LoadSelectedRing(ring_list_->currentRow());
}

void CoveragePanel::LoadSelectedRing(int row) {
  const bool valid = row >= 0 && row < static_cast<int>(rings_.size());
  remove_ring_->setEnabled(valid && !busy_ &&
                           rings_[row].kind != QLatin1String(kFieldKind));
}

void CoveragePanel::AddObstacleRing() {
  const int field = FieldRingIndex();
  double centre_x = 0.0;
  double centre_y = 0.0;
  if (field >= 0 && !rings_[field].points.empty()) {
    centre_x = 0.0;
    centre_y = 0.0;
    for (const auto& point : rings_[field].points) {
      centre_x += point.first;
      centre_y += point.second;
    }
    centre_x /= static_cast<double>(rings_[field].points.size());
    centre_y /= static_cast<double>(rings_[field].points.size());
  }
  int ordinal = 1;
  for (const auto& ring : rings_) {
    if (ring.kind == QLatin1String(kObstacleKind)) {
      ++ordinal;
    }
  }
  constexpr double kHalfSize = 0.5;
  Ring ring;
  ring.name = tr("Obstacle %1").arg(ordinal);
  ring.kind = QString::fromLatin1(kObstacleKind);
  ring.points = {{centre_x - kHalfSize, centre_y - kHalfSize},
                 {centre_x + kHalfSize, centre_y - kHalfSize},
                 {centre_x + kHalfSize, centre_y + kHalfSize},
                 {centre_x - kHalfSize, centre_y + kHalfSize}};
  rings_.push_back(std::move(ring));
  RebuildRingList(static_cast<int>(rings_.size()) - 1);
  PublishSketch();
  SetStatus(tr("Obstacle added at the field centre; edit its points"),
            QStringLiteral("ok"));
}

void CoveragePanel::RemoveSelectedRing() {
  const int row = ring_list_->currentRow();
  if (row < 0 || row >= static_cast<int>(rings_.size())) {
    return;
  }
  if (rings_[row].kind == QLatin1String(kFieldKind)) {
    SetStatus(tr("The work area ring cannot be removed"),
              QStringLiteral("error"));
    return;
  }
  rings_.erase(rings_.begin() + row);
  RebuildRingList(std::max(0, row - 1));
  PublishSketch();
  SetStatus(tr("Obstacle removed"), QStringLiteral("ok"));
}

void CoveragePanel::ClearObstacleRings() {
  const std::size_t before = rings_.size();
  rings_.erase(std::remove_if(rings_.begin(), rings_.end(),
                              [](const Ring& ring) {
                                return ring.kind !=
                                       QLatin1String(kFieldKind);
                              }),
               rings_.end());
  RebuildRingList(0);
  PublishSketch();
  if (before == rings_.size()) {
    SetStatus(tr("No obstacle rings to clear"));
  } else {
    SetStatus(tr("Removed %1 obstacle ring(s)")
                  .arg(static_cast<int>(before - rings_.size())),
              QStringLiteral("ok"));
  }
}

void CoveragePanel::RefreshLiveObstacles() {
  if (manager_ == nullptr) {
    return;
  }
  display::MapDisplay* map_display = nullptr;
  for (auto* candidate : manager_->displays()) {
    auto* map = dynamic_cast<display::MapDisplay*>(candidate);
    if (map == nullptr) {
      continue;
    }
    if (map->channel() == "/map") {
      map_display = map;
      break;
    }
    if (map_display == nullptr) {
      map_display = map;
    }
  }
  if (map_display == nullptr) {
    obstacle_status_->setText(
        tr("No Map display; add a Map display on /map first"));
    return;
  }
  const auto* grid = map_display->mapMessage();
  if (grid == nullptr) {
    obstacle_status_->setText(
        tr("Waiting for %1").arg(QString::fromStdString(map_display->channel())));
    return;
  }
  ObstacleExtractionOptions options;
  options.margin = obstacle_margin_->value();
  const auto extracted = CoverageObstacleExtractor::Extract(*grid, options);
  const std::uint64_t signature = CoverageObstacleExtractor::Signature(
      *grid, options.lethal_threshold);
  const bool changed =
      obstacle_signature_ != 0 && signature != obstacle_signature_;
  obstacle_signature_ = signature;

  const int selected = ring_list_->currentRow();
  rings_.erase(std::remove_if(rings_.begin(), rings_.end(),
                              [](const Ring& ring) {
                                return ring.kind ==
                                       QLatin1String(kLiveKind);
                              }),
               rings_.end());
  int ordinal = 0;
  for (const auto& obstacle : extracted) {
    Ring ring;
    ring.name = tr("Live %1").arg(++ordinal);
    ring.kind = QString::fromLatin1(kLiveKind);
    ring.points = obstacle.points;
    rings_.push_back(std::move(ring));
  }
  RebuildRingList(selected);
  obstacle_status_->setText(
      tr("%1 live obstacle(s) · %2 · %3")
          .arg(ordinal)
          .arg(QString::fromStdString(map_display->channel()))
          .arg(QTime::currentTime().toString(QStringLiteral("HH:mm:ss"))));
  if (!changed || !auto_replan_->isChecked()) {
    return;
  }
  if (busy_) {
    pending_replan_ = true;
    obstacle_status_->setText(
        obstacle_status_->text() + tr(" · re-plan queued"));
    return;
  }
  PreviewCoverage();
}

void CoveragePanel::OnAdvancedToggled(bool expanded) {
  advanced_->setText(expanded ? tr("Hide JSON") : tr("Advanced JSON"));
  if (expanded) {
    polygons_->setPlainText(SerializeRings());
    polygons_->setVisible(true);
    return;
  }
  if (!ApplyRingsJson(polygons_->toPlainText())) {
    const QSignalBlocker blocker(advanced_);
    advanced_->setChecked(true);
    return;
  }
  polygons_->setVisible(false);
}

QString CoveragePanel::SerializeRings() const {
  QJsonArray array;
  for (const auto& ring : rings_) {
    QJsonArray points;
    for (const auto& point : ring.points) {
      QJsonArray coordinate;
      coordinate.append(point.first);
      coordinate.append(point.second);
      points.append(coordinate);
    }
    array.append(points);
  }
  return QString::fromUtf8(
      QJsonDocument(array).toJson(QJsonDocument::Compact));
}

bool CoveragePanel::ApplyRingsJson(const QString& text) {
  QJsonParseError parse_error{};
  const auto document = QJsonDocument::fromJson(text.toUtf8(), &parse_error);
  if (parse_error.error != QJsonParseError::NoError) {
    SetStatus(parse_error.errorString(), QStringLiteral("error"));
    return false;
  }
  if (!document.isArray() || document.array().isEmpty()) {
    SetStatus(tr("Provide at least one polygon ring"),
              QStringLiteral("error"));
    return false;
  }
  std::vector<Ring> rings;
  int ordinal = 0;
  const auto array = document.array();
  for (int index = 0; index < array.size(); ++index) {
    const auto coordinates = array[index].toArray();
    if (!array[index].isArray() || coordinates.size() < 3) {
      SetStatus(tr("Ring %1 needs at least three points").arg(index + 1),
                QStringLiteral("error"));
      return false;
    }
    Ring ring;
    ring.kind = index == 0 ? QString::fromLatin1(kFieldKind)
                           : QString::fromLatin1(kObstacleKind);
    ring.name = index == 0 ? tr("Work area")
                           : tr("Obstacle %1").arg(++ordinal);
    for (const auto& coordinate : coordinates) {
      const auto point = coordinate.toArray();
      if (point.size() != 2 || !point[0].isDouble() || !point[1].isDouble()) {
        SetStatus(tr("Ring %1 coordinates must be [x, y] numbers")
                      .arg(index + 1),
                  QStringLiteral("error"));
        return false;
      }
      ring.points.emplace_back(point[0].toDouble(), point[1].toDouble());
    }
    rings.push_back(std::move(ring));
  }
  rings_ = std::move(rings);
  obstacle_signature_ = 0;
  pending_replan_ = false;
  RebuildRingList(0);
  PublishSketch();
  SetStatus(tr("Imported %1 ring(s)").arg(rings_.size()),
            QStringLiteral("ok"));
  return true;
}

void CoveragePanel::CommitEditors() {
  radius_->interpretText();
  headland_->interpretText();
  obstacle_margin_->interpretText();
}

void CoveragePanel::SetStatus(const QString& text, const QString& state) {
  if (note_bar_ == nullptr) {
    return;
  }
  note_bar_->SetMessage(text);
  if (state == QLatin1String("error")) {
    note_bar_->SetLevel(widget::FeedbackLevel::kError);
  } else if (state == QLatin1String("ok")) {
    note_bar_->SetLevel(widget::FeedbackLevel::kSuccess);
  } else {
    note_bar_->SetLevel(widget::FeedbackLevel::kInformation);
  }
}

void CoveragePanel::SetProgress(bool running, int value) {
  if (progress_ == nullptr) {
    return;
  }
  progress_->setVisible(running);
  if (!running) {
    progress_->setRange(0, 100);
    progress_->setValue(0);
    if (progress_value_ != nullptr) {
      progress_value_->setText(QStringLiteral("\u2014"));
    }
    if (remaining_value_ != nullptr) {
      remaining_value_->setText(QStringLiteral("\u2014"));
    }
    return;
  }
  if (value < 0) {
    progress_->setRange(0, 0);
    if (progress_value_ != nullptr) {
      progress_value_->setText(tr("Planning"));
    }
    return;
  }
  const int percent = std::clamp(value, 0, 100);
  progress_->setRange(0, 100);
  progress_->setValue(percent);
  if (progress_value_ != nullptr) {
    progress_value_->setText(QStringLiteral("%1%").arg(percent));
  }
}

void CoveragePanel::SetBusy(bool busy) {
  busy_ = busy;
  const bool enabled = !busy;
  const auto enable = [enabled](QWidget* widget) {
    if (widget != nullptr) {
      widget->setEnabled(enabled);
    }
  };
  enable(preview_);
  enable(start_);
  enable(frame_);
  enable(radius_);
  enable(swath_);
  enable(headland_);
  enable(route_mode_);
  enable(path_mode_);
  enable(approach_);
  enable(ring_list_);
  enable(add_obstacle_);
  enable(draw_obstacle_);
  enable(clear_obstacles_);
  enable(live_obstacles_);
  enable(obstacle_margin_);
  enable(refresh_obstacles_);
  enable(advanced_);
  enable(rooms_);
  enable(room_columns_);
  enable(room_rows_);
  enable(split_rooms_);
  enable(map_rooms_);
  if (clean_rooms_ != nullptr) {
    clean_rooms_->setEnabled(enabled && !regions_.empty());
  }
  enable(polygons_);
  if (cancel_ != nullptr) {
    cancel_->setEnabled(busy);
  }
  if (auto_replan_ != nullptr) {
    auto_replan_->setEnabled(enabled && live_obstacles_ != nullptr &&
                             live_obstacles_->isChecked());
  }
  if (!busy) {
    LoadSelectedRing(ring_list_ != nullptr ? ring_list_->currentRow() : -1);
  } else if (remove_ring_ != nullptr) {
    remove_ring_->setEnabled(false);
  }
}

autonomy::task::coverage::proto::CoverageRequest CoveragePanel::BuildRequest()
    const {
  const int field = FieldRingIndex();
  if (field < 0) {
    throw std::invalid_argument("A work area ring is required");
  }
  for (const auto& ring : rings_) {
    if (ring.points.size() < 3) {
      throw std::invalid_argument(
          "Every ring needs at least three points");
    }
  }
  // A hole that crosses the work-area boundary reaches the planner as an
  // invalid self-intersection. Reject it here with the offending ring name.
  {
    const auto& area = rings_[field].points;
    for (int index = 0; index < static_cast<int>(rings_.size()); ++index) {
      if (index == field) {
        continue;
      }
      for (const auto& point : rings_[index].points) {
        bool inside = false;
        for (std::size_t i = 0, j = area.size() - 1; i < area.size(); j = i++) {
          const bool crosses =
              ((area[i].second > point.second) !=
               (area[j].second > point.second)) &&
              (point.first <
               (area[j].first - area[i].first) * (point.second - area[i].second) /
                       (area[j].second - area[i].second) +
                   area[i].first);
          if (crosses) {
            inside = !inside;
          }
        }
        if (!inside) {
          throw std::invalid_argument(
              rings_[index].name.toStdString() +
              " has a point outside the work area");
        }
      }
    }
  }
  autonomy::task::coverage::proto::CoverageRequest request;
  request.set_frame_id(frame_->text().trimmed().toStdString());
  if (request.frame_id().empty()) {
    throw std::invalid_argument("Frame is empty");
  }
  std::vector<int> order;
  order.push_back(field);
  for (int index = 0; index < static_cast<int>(rings_.size()); ++index) {
    if (index != field) {
      order.push_back(index);
    }
  }
  for (const int index : order) {
    auto* ring = request.add_polygons();
    for (const auto& point : rings_[index].points) {
      auto* coordinate = ring->add_coordinates();
      coordinate->set_axis1(point.first);
      coordinate->set_axis2(point.second);
    }
  }
  request.set_robot_radius(radius_->value());
  request.set_swath_spacing(swath_->value());
  if (have_robot_pose_) {
    auto* pose = request.mutable_robot_pose();
    pose->set_axis1(robot_x_);
    pose->set_axis2(robot_y_);
  }
  request.mutable_headland_mode()->set_mode("CONSTANT");
  request.mutable_headland_mode()->set_width(headland_->value());
  request.mutable_route_mode()->set_mode(
      route_mode_->currentData().toString().toStdString());
  request.mutable_route_mode()->set_spiral_n(4);
  request.mutable_path_mode()->set_mode(
      path_mode_->currentData().toString().toStdString());
  request.mutable_path_mode()->set_continuity_mode("DISCONTINUOUS");
  request.mutable_path_mode()->set_turn_point_distance(0.05);
  request.set_coordinates_in_cartesian_frame(true);
  return request;
}

void CoveragePanel::PreviewCoverage() {
  if (drawing_) {
    CancelObstacleDraft();
  }
  try {
    CommitEditors();
    EnsureClients();
    if (!compute_client_ || !compute_client_->ActionServerIsReady()) {
      throw std::runtime_error("Coverage planner is not ready");
    }
    Compute::Goal goal;
    *goal.mutable_request() = BuildRequest();
    cancel_requested_ = false;
    pending_replan_ = false;
    compute_acceptance_ = compute_client_->AsyncSendGoal(goal);
    SetBusy(true);
    SetProgress(true);
    SetStatus(tr("Planning the full coverage path…"));
  } catch (const std::exception& error) {
    SetStatus(QString::fromUtf8(error.what()), QStringLiteral("error"));
  }
}

void CoveragePanel::StartCoverage() {
  if (drawing_) {
    CancelObstacleDraft();
  }
  try {
    CommitEditors();
    EnsureClients();
    if (!navigate_client_ || !navigate_client_->ActionServerIsReady()) {
      throw std::runtime_error("Coverage navigator is not ready");
    }
    Navigate::Goal goal;
    *goal.mutable_request() = BuildRequest();
    goal.set_task_id(
        QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString());
    goal.set_client_id("autoviz-navigation-panel");
    goal.set_navigate_to_start(approach_->isChecked());
    QPointer<CoveragePanel> self(this);
    NavigateClient::SendGoalOptions options;
    options.feedback_callback = [self](auto, const auto& feedback) {
      if (!self || !feedback) {
        return;
      }
      const double progress = feedback->progress();
      const double distance = feedback->distance_remaining();
      QMetaObject::invokeMethod(
          self,
          [self, progress, distance] {
            if (!self) {
              return;
            }
            self->SetProgress(true,
                              static_cast<int>(std::lround(progress * 100.0)));
            if (self->remaining_value_ != nullptr) {
              self->remaining_value_->setText(
                  QStringLiteral("%1 m").arg(distance, 0, 'f', 1));
            }
            self->SetStatus(tr("Covering %1% · %2 m remaining")
                                .arg(progress * 100, 0, 'f', 1)
                                .arg(distance, 0, 'f', 1));
          },
          Qt::QueuedConnection);
    };
    cancel_requested_ = false;
    pending_replan_ = false;
    navigate_acceptance_ = navigate_client_->AsyncSendGoal(goal, options);
    SetBusy(true);
    SetProgress(true, 0);
    SetStatus(tr("Starting complete coverage…"));
  } catch (const std::exception& error) {
    SetStatus(QString::fromUtf8(error.what()), QStringLiteral("error"));
  }
}

void CoveragePanel::CancelCoverage() {
  cancel_requested_ = true;
  if (compute_goal_ && compute_client_) {
    compute_client_->AsyncCancelGoal(compute_goal_);
  }
  if (navigate_goal_ && navigate_client_) {
    navigate_client_->AsyncCancelGoal(navigate_goal_);
  }
  if (busy_) {
    SetStatus(tr("Canceling coverage…"));
  }
}

void CoveragePanel::PollActions() {
  // The plan is a dense boustrophedon polyline: one household map already
  // holds ~26k poses. A uniform stride used to cut that down to 1500 points,
  // which collapsed whole rooms into a couple of diagonals and made them look
  // uncovered. The Path display draws a plain line strip, so keep the geometry
  // intact and only thin pathological inputs.
  const auto PreviewPath = [this](const auto& path) {
    constexpr int kPreviewPoseLimit = 200000;
    const int count = path.poses_size();
    if (count <= kPreviewPoseLimit) {
      preview_writer_->Write(path);
      return;
    }
    automsgs::msgs::nav_msgs::Path thin;
    thin.mutable_header()->set_frame_id(path.header().frame_id());
    const int stride = (count + kPreviewPoseLimit - 1) / kPreviewPoseLimit;
    for (int index = 0; index < count; index += stride) {
      *thin.add_poses() = path.poses().Get(index);
    }
    *thin.add_poses() = path.poses().Get(count - 1);
    preview_writer_->Write(thin);
  };
  try {
    EnsureClients();
    if (compute_acceptance_.valid() &&
        compute_acceptance_.wait_for(std::chrono::seconds(0)) ==
            std::future_status::ready) {
      compute_goal_ = compute_acceptance_.get();
      compute_acceptance_ = {};
      if (!compute_goal_) {
        SetStatus(tr("Coverage plan rejected"), QStringLiteral("error"));
        SetBusy(false);
        SetProgress(false);
      } else if (cancel_requested_) {
        compute_client_->AsyncCancelGoal(compute_goal_);
      }
    }
    if (navigate_acceptance_.valid() &&
        navigate_acceptance_.wait_for(std::chrono::seconds(0)) ==
            std::future_status::ready) {
      navigate_goal_ = navigate_acceptance_.get();
      navigate_acceptance_ = {};
      if (!navigate_goal_) {
        SetStatus(tr("Coverage navigation rejected"),
                  QStringLiteral("error"));
        SetBusy(false);
        SetProgress(false);
      } else if (cancel_requested_) {
        navigate_client_->AsyncCancelGoal(navigate_goal_);
      }
    }
    if (compute_goal_) {
      auto future = compute_goal_->AsyncGetResult();
      if (future.wait_for(std::chrono::seconds(0)) ==
          std::future_status::ready) {
        const auto result = future.get();
        if (result.result && result.result->error_code() == 0) {
          if (result.result->regions_size() > 0) {
            regions_.clear();
            if (rooms_ != nullptr) {
              rooms_->clear();
            }
            for (const auto& region : result.result->regions()) {
              regions_.push_back(region);
              if (rooms_ != nullptr) {
                auto* item = new QListWidgetItem(
                    tr("%1. %2 · %3 m² · %4%")
                        .arg(region.order())
                        .arg(QString::fromStdString(region.name()))
                        .arg(region.area(), 0, 'f', 1)
                        .arg(region.coverage() * 100, 0, 'f', 0),
                    rooms_);
                static const std::array<const char*, 8> kColors{
                    "#3388f2", "#40bf72", "#f2a633", "#bf73f2",
                    "#26cccc", "#f273a6", "#99805a", "#b3d940"};
                item->setForeground(QColor(
                    kColors[static_cast<std::size_t>(region.color_index()) %
                            kColors.size()]));
              }
            }
            if (clean_rooms_ != nullptr) {
              clean_rooms_->setEnabled(!regions_.empty());
            }
            PreviewPath(result.result->nav_path());
            SetStatus(tr("Rooms ready · %1 room(s) · %2% · %3 m travel")
                          .arg(result.result->regions_size())
                          .arg(result.result->core_coverage() * 100, 0, 'f', 1)
                          .arg(result.result->travel_distance(), 0, 'f', 1),
                      QStringLiteral("ok"));
            compute_goal_.reset();
            SetBusy(false);
            SetProgress(false);
            return;
          }
          PreviewPath(result.result->nav_path());
          if (progress_value_ != nullptr) {
            progress_value_->setText(
                QStringLiteral("%1%").arg(result.result->core_coverage() * 100,
                                          0, 'f', 1));
          }
          SetStatus(tr("Plan ready · %1 swaths · %2% core coverage")
                        .arg(result.result->coverage_path().swaths_size())
                        .arg(result.result->core_coverage() * 100, 0, 'f', 2),
                    QStringLiteral("ok"));
        } else {
          SetStatus(result.result
                        ? QString::fromStdString(result.result->error_msg())
                        : tr("Coverage planning failed"),
                    QStringLiteral("error"));
        }
        compute_goal_.reset();
        SetBusy(false);
        SetProgress(false);
      }
    }
    if (navigate_goal_) {
      auto future = navigate_goal_->AsyncGetResult();
      if (future.wait_for(std::chrono::seconds(0)) ==
          std::future_status::ready) {
        const auto result = future.get();
        if (result.result && result.result->error_code() == 0) {
          if (cleaning_regions_ && region_index_ + 1 < regions_.size()) {
            navigate_goal_.reset();
            SetProgress(false);
            StartRegion(region_index_ + 1);
            return;
          }
          const bool finished_all =
              cleaning_regions_ && !regions_.empty();
          cleaning_regions_ = false;
          SetProgress(false);
          if (progress_value_ != nullptr) {
            progress_value_->setText(QStringLiteral("100%"));
          }
          if (remaining_value_ != nullptr) {
            remaining_value_->setText(QStringLiteral("0 m"));
          }
          SetStatus(finished_all
                        ? tr("All %1 room(s) cleaned").arg(regions_.size())
                        : tr("Complete coverage succeeded · %1 segment(s)")
                              .arg(result.result->completed_segments()),
                    QStringLiteral("ok"));
        } else {
          SetStatus(result.result
                        ? QString::fromStdString(result.result->error_msg())
                        : tr("Coverage navigation failed"),
                    QStringLiteral("error"));
        }
        navigate_goal_.reset();
        SetBusy(false);
        if (!(result.result && result.result->error_code() == 0)) {
          SetProgress(false);
        }
        if (pending_replan_ && auto_replan_->isChecked()) {
          pending_replan_ = false;
          PreviewCoverage();
        }
      }
    }
  } catch (const std::exception& error) {
    SetStatus(QString::fromUtf8(error.what()), QStringLiteral("error"));
    SetBusy(false);
    SetProgress(false);
  }
}

void CoveragePanel::ToggleObstacleDrawing() {
  if (busy_ || !isVisible()) {
    return;
  }
  if (drawing_) {
    FinishObstacleDraft();
    return;
  }
  EnsureClients();
  if (manager_ == nullptr || manager_->autolinkNode() == nullptr) {
    SetStatus(tr("Autolink is not ready"), QStringLiteral("error"));
    return;
  }
  if (!click_reader_) {
    QPointer<CoveragePanel> self(this);
    click_reader_ = manager_->autolinkNode()->CreateReader<
        automsgs::msgs::geometry_msgs::PointStamped>(
        "/clicked_point",
        [self](const std::shared_ptr<
               automsgs::msgs::geometry_msgs::PointStamped>& message) {
          if (!self || !message) {
            return;
          }
          const double x = message->point().x();
          const double y = message->point().y();
          const QString frame =
              QString::fromStdString(message->header().frame_id());
          QMetaObject::invokeMethod(
              self,
              [self, x, y, frame] {
                if (self) {
                  self->HandleClickedPoint(x, y, frame);
                }
              },
              Qt::QueuedConnection);
        });
  }
  // Keep Publish Point armed until the polygon is finished; its default
  // single-click mode would revert to Interact after the first vertex.
  if (auto* tool = manager_->tools().toolById("PublishPoint")) {
    tool->setPropertyValue("single_click", "false");
  }
  draft_.clear();
  draft_frame_ = frame_->text().trimmed();
  drawing_ = true;
  finish_shortcut_->setEnabled(true);
  cancel_shortcut_->setEnabled(true);
  draw_obstacle_->setText(tr("Finish (Enter)"));
  ActivateTool(QStringLiteral("PublishPoint"));
  PublishSketch();
  SetStatus(tr("Click at least three points in the 3D view · "
               "Enter finishes · Esc cancels"));
}

void CoveragePanel::HandleClickedPoint(double x, double y,
                                       const QString& frame) {
  if (!drawing_) {
    return;
  }
  if (!frame.isEmpty() && !draft_frame_.isEmpty() && frame != draft_frame_) {
    SetStatus(tr("Clicked point is in %1; draw in %2")
                  .arg(frame, draft_frame_),
              QStringLiteral("error"));
    return;
  }
  if (!std::isfinite(x) || !std::isfinite(y)) {
    return;
  }
  if (draft_.size() >= 3) {
    const double dx = x - draft_.front().first;
    const double dy = y - draft_.front().second;
    if (std::hypot(dx, dy) < 0.25) {
      FinishObstacleDraft();
      return;
    }
  }
  draft_.emplace_back(x, y);
  PublishSketch();
  draw_obstacle_->setText(tr("Finish (%1)").arg(draft_.size()));
  SetStatus(tr("%1 point(s) · Enter finishes · Esc cancels")
                .arg(draft_.size()));
}

void CoveragePanel::FinishObstacleDraft() {
  if (!drawing_) {
    return;
  }
  if (draft_.size() < 3) {
    SetStatus(tr("Need at least three points (currently %1)")
                  .arg(draft_.size()),
              QStringLiteral("error"));
    return;
  }
  int ordinal = 1;
  for (const auto& ring : rings_) {
    if (ring.kind == QLatin1String(kObstacleKind)) {
      ++ordinal;
    }
  }
  const std::size_t count = draft_.size();
  Ring ring;
  ring.name = tr("Obstacle %1").arg(ordinal);
  ring.kind = QString::fromLatin1(kObstacleKind);
  ring.points = draft_;
  rings_.push_back(std::move(ring));
  draft_.clear();
  drawing_ = false;
  finish_shortcut_->setEnabled(false);
  cancel_shortcut_->setEnabled(false);
  draw_obstacle_->setText(tr("Draw obstacle (Ctrl+Alt+O)"));
  if (manager_ != nullptr) {
    if (auto* tool = manager_->tools().toolById("PublishPoint")) {
      tool->setPropertyValue("single_click", "true");
    }
    ActivateTool(QStringLiteral("Interact"));
  }
  const int row = static_cast<int>(rings_.size()) - 1;
  RebuildRingList(row);
  PublishSketch();
  SetStatus(tr("Obstacle %1 added with %2 points")
                .arg(ordinal)
                .arg(static_cast<int>(count)),
            QStringLiteral("ok"));
  if (!busy_) {
    PreviewCoverage();
  }
}

void CoveragePanel::CancelObstacleDraft() {
  if (!drawing_) {
    return;
  }
  drawing_ = false;
  draft_.clear();
  finish_shortcut_->setEnabled(false);
  cancel_shortcut_->setEnabled(false);
  draw_obstacle_->setText(tr("Draw obstacle (Ctrl+Alt+O)"));
  if (manager_ != nullptr) {
    if (auto* tool = manager_->tools().toolById("PublishPoint")) {
      tool->setPropertyValue("single_click", "true");
    }
    ActivateTool(QStringLiteral("Interact"));
  }
  PublishSketch();
  SetStatus(tr("Obstacle drawing canceled"));
}

void CoveragePanel::PublishSketch() {
  if (!sketch_writer_) {
    return;
  }
  using Marker = automsgs::msgs::visualization_msgs::Marker;
  using Markers = automsgs::msgs::visualization_msgs::MarkerArray;
  const std::string frame = frame_->text().trimmed().toStdString();
  Markers markers;
  auto* clear = markers.add_markers();
  clear->mutable_header()->set_frame_id(frame);
  clear->set_action(Marker::DELETEALL);
  int marker_id = 1;
  auto add_ring = [&](const std::vector<std::pair<double, double>>& points,
                      float red, float green, float blue, float width) {
    if (points.size() < 2) {
      return;
    }
    auto* marker = markers.add_markers();
    marker->mutable_header()->set_frame_id(frame);
    marker->set_ns("coverage_obstacle_sketch");
    marker->set_id(marker_id++);
    marker->set_type(Marker::LINE_STRIP);
    marker->set_action(Marker::ADD);
    marker->mutable_pose()->mutable_orientation()->set_w(1.0);
    marker->mutable_scale()->set_x(width);
    marker->mutable_color()->set_r(red);
    marker->mutable_color()->set_g(green);
    marker->mutable_color()->set_b(blue);
    marker->mutable_color()->set_a(0.95f);
    for (const auto& point : points) {
      auto* out = marker->add_points();
      out->set_x(point.first);
      out->set_y(point.second);
      out->set_z(0.05);
    }
    if (points.size() >= 3) {
      auto* out = marker->add_points();
      out->set_x(points.front().first);
      out->set_y(points.front().second);
      out->set_z(0.05);
    }
  };
  for (const auto& ring : rings_) {
    if (ring.kind == QLatin1String(kFieldKind)) {
      add_ring(ring.points, 0.15f, 0.45f, 0.95f, 0.04f);
    } else if (ring.kind == QLatin1String(kObstacleKind)) {
      add_ring(ring.points, 0.90f, 0.45f, 0.10f, 0.05f);
    }
  }
  if (!draft_.empty()) {
    add_ring(draft_, 0.15f, 0.80f, 0.35f, 0.06f);
    auto* dots = markers.add_markers();
    dots->mutable_header()->set_frame_id(frame);
    dots->set_ns("coverage_obstacle_vertices");
    dots->set_id(marker_id++);
    dots->set_type(Marker::SPHERE_LIST);
    dots->set_action(Marker::ADD);
    dots->mutable_pose()->mutable_orientation()->set_w(1.0);
    dots->mutable_scale()->set_x(0.12);
    dots->mutable_scale()->set_y(0.12);
    dots->mutable_scale()->set_z(0.12);
    dots->mutable_color()->set_r(0.15f);
    dots->mutable_color()->set_g(0.80f);
    dots->mutable_color()->set_b(0.35f);
    dots->mutable_color()->set_a(0.95f);
    for (const auto& point : draft_) {
      auto* out = dots->add_points();
      out->set_x(point.first);
      out->set_y(point.second);
      out->set_z(0.06);
    }
  }
  sketch_writer_->Write(markers);
}

void CoveragePanel::ActivateTool(const QString& tool_id) {
  // The viewport dispatches mouse events to its dock-local tool, which
  // only the toolbar action updates. Trigger that action so the panel and
  // the toolbar stay in sync.
  QWidget* top = window();
  if (top == nullptr) {
    return;
  }
  for (QAction* action : top->findChildren<QAction*>()) {
    if (action != nullptr && action->objectName() == tool_id &&
        action->isCheckable()) {
      if (!action->isChecked()) {
        action->trigger();
      }
      return;
    }
  }
}


void CoveragePanel::SplitRooms(bool from_map) {
  try {
    CommitEditors();
    EnsureClients();
    if (compute_client_ == nullptr || !compute_client_->ActionServerIsReady()) {
      throw std::runtime_error("Coverage planner is not ready");
    }
    Compute::Goal goal;
    *goal.mutable_request() = BuildRequest();
    goal.mutable_request()->set_generate_regions(true);
    goal.mutable_request()->set_region_columns(room_columns_->value());
    goal.mutable_request()->set_region_rows(room_rows_->value());
    if (from_map) {
      display::MapDisplay* map_display = nullptr;
      for (auto* candidate : manager_->displays()) {
        auto* map = dynamic_cast<display::MapDisplay*>(candidate);
        if (map != nullptr && map->channel() == "/map") {
          map_display = map;
          break;
        }
      }
      const auto* grid =
          map_display != nullptr ? map_display->mapMessage() : nullptr;
      if (grid == nullptr) {
        throw std::runtime_error("No /map occupancy grid available");
      }
      auto* map = goal.mutable_request()->mutable_map();
      map->set_frame_id(grid->header().frame_id());
      map->set_resolution(grid->info().resolution());
      map->set_width(grid->info().width());
      map->set_height(grid->info().height());
      map->set_origin_x(grid->info().origin().position().x());
      map->set_origin_y(grid->info().origin().position().y());
      const auto& q = grid->info().origin().orientation();
      map->set_origin_yaw(std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                                    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z())));
      for (const int value : grid->data()) {
        map->add_data(value);
      }
    }
    cancel_requested_ = false;
    compute_acceptance_ = compute_client_->AsyncSendGoal(goal);
    SetBusy(true);
    SetProgress(true);
    SetStatus(tr("Splitting the map into rooms…"));
  } catch (const std::exception& error) {
    SetStatus(QString::fromUtf8(error.what()), QStringLiteral("error"));
  }
}

void CoveragePanel::StartAllRooms() {
  if (regions_.empty()) {
    SetStatus(tr("Split the map into rooms first"));
    return;
  }
  cleaning_regions_ = true;
  StartRegion(0);
}

void CoveragePanel::StartRegion(std::size_t index) {
  if (index >= regions_.size()) {
    cleaning_regions_ = false;
    SetBusy(false);
    SetProgress(false);
    SetStatus(tr("All %1 room(s) cleaned").arg(regions_.size()),
              QStringLiteral("ok"));
    return;
  }
  try {
    EnsureClients();
    if (navigate_client_ == nullptr || !navigate_client_->ActionServerIsReady()) {
      throw std::runtime_error("Coverage navigator is not ready");
    }
    const auto& region = regions_[index];
    Navigate::Goal goal;
    auto request = BuildRequest();
    request.clear_polygons();
    request.set_generate_regions(false);
    request.set_generate_headland(false);
    for (const auto& polygon : region.polygons()) {
      auto* ring = request.add_polygons();
      for (const auto& point : polygon.coordinates()) {
        auto* coordinate = ring->add_coordinates();
        coordinate->set_axis1(point.axis1());
        coordinate->set_axis2(point.axis2());
      }
    }
    *goal.mutable_request() = request;
    goal.set_task_id(
        QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString());
    goal.set_client_id("autoviz-coverage-rooms");
    goal.set_navigate_to_start(true);
    region_index_ = index;
    navigate_acceptance_ = navigate_client_->AsyncSendGoal(goal);
    SetBusy(true);
    SetProgress(true, static_cast<int>(100.0 * index /
                                      static_cast<double>(regions_.size())));
    SetStatus(tr("Cleaning room %1 of %2 · %3")
                  .arg(index + 1)
                  .arg(regions_.size())
                  .arg(QString::fromStdString(region.name())));
  } catch (const std::exception& error) {
    cleaning_regions_ = false;
    SetStatus(QString::fromUtf8(error.what()), QStringLiteral("error"));
  }
}

void CoveragePanel::UpdateCurrentRegion(double x, double y) {
  if (current_region_ == nullptr || regions_.empty()) {
    return;
  }
  for (std::size_t index = 0; index < regions_.size(); ++index) {
    const auto& region = regions_[index];
    if (region.polygons_size() == 0) continue;
    const auto& ring = region.polygons(0).coordinates();
    bool inside = false;
    for (int i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
      const double xi = ring.Get(i).axis1();
      const double yi = ring.Get(i).axis2();
      const double xj = ring.Get(j).axis1();
      const double yj = ring.Get(j).axis2();
      if (((yi > y) != (yj > y)) &&
          (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
        inside = !inside;
      }
    }
    if (inside) {
      current_region_->setText(tr("Current room: %1").arg(index + 1));
      if (rooms_ != nullptr) rooms_->setCurrentRow(static_cast<int>(index));
      return;
    }
  }
  current_region_->setText(tr("Current room: —"));
}

}  // namespace autoviz::navigation
#endif
