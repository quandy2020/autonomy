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

/** @brief Create a compact metre spin box that commits on editing finish. */
QDoubleSpinBox* MakeLengthSpin(double minimum, double maximum, double value,
                               QWidget* parent) {
  auto* spin = new QDoubleSpinBox(parent);
  spin->setRange(minimum, maximum);
  spin->setDecimals(2);
  spin->setSingleStep(0.05);
  spin->setValue(value);
  spin->setSuffix(QStringLiteral(" m"));
  spin->setKeyboardTracking(false);
  spin->setMinimumWidth(96);
  return spin;
}

/** @brief Render ring points one "x, y" pair per line. */
QString FormatPoints(const std::vector<std::pair<double, double>>& points) {
  QStringList lines;
  lines.reserve(static_cast<int>(points.size()));
  for (const auto& point : points) {
    lines << QStringLiteral("%1, %2")
                 .arg(point.first, 0, 'f', 3)
                 .arg(point.second, 0, 'f', 3);
  }
  return lines.join(QChar('\n'));
}

/** @brief Parse "x, y" or "x y" lines; reports the failing coordinate. */
std::vector<std::pair<double, double>> ParsePoints(const QString& text,
                                                   QString* error) {
  std::vector<std::pair<double, double>> points;
  const auto lines = text.split(QChar('\n'));
  for (int index = 0; index < lines.size(); ++index) {
    QString line = lines[index].trimmed();
    if (line.isEmpty() || line.startsWith(QChar('#'))) {
      continue;
    }
    line.replace(QChar(','), QChar(' '));
    line.replace(QChar(';'), QChar(' '));
    const auto parts = line.split(QRegularExpression(QStringLiteral("\\s+")),
                                  Qt::SkipEmptyParts);
    bool ok_x = false;
    bool ok_y = false;
    const double x = parts.size() == 2 ? parts[0].toDouble(&ok_x) : 0.0;
    const double y = parts.size() == 2 ? parts[1].toDouble(&ok_y) : 0.0;
    if (parts.size() != 2 || !ok_x || !ok_y || !std::isfinite(x) ||
        !std::isfinite(y)) {
      if (error != nullptr) {
        *error = QObject::tr("Line %1 must be 'x, y' numbers").arg(index + 1);
      }
      return {};
    }
    points.emplace_back(x, y);
  }
  return points;
}

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
      tr("Cover the work area and route around obstacles. Each pass cleans "
         "twice the robot radius."),
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

  swath_hint_ = new QLabel(content);
  StyleHintLabel(swath_hint_);
  swath_hint_->setText(
      tr("Swath width %1 m").arg(2.0 * radius_->value(), 0, 'f', 2));
  form->addRow(QString(), swath_hint_);

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
  draw_obstacle_ = MakeCoverageCompactGhost(tr("Draw polygon"), content);
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

  auto* points_label = new QLabel(tr("Ring points (x, y per line)"), content);
  StyleHintLabel(points_label);
  layout->addWidget(points_label);
  points_ = new QPlainTextEdit(content);
  points_->setObjectName(QStringLiteral("coverageRingPoints"));
  points_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  points_->setMinimumHeight(64);
  points_->setMaximumHeight(96);
  points_->setPlaceholderText(QStringLiteral("-3, -2\n3, -2\n3, 2\n-3, 2"));
  layout->addWidget(points_);

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

  connect(radius_, &QDoubleSpinBox::valueChanged, this, [this](double value) {
    swath_hint_->setText(
        tr("Swath width %1 m").arg(2.0 * value, 0, 'f', 2));
  });
  connect(ring_list_, &QListWidget::currentRowChanged, this,
          &CoveragePanel::LoadSelectedRing);
  points_timer_ = new QTimer(this);
  points_timer_->setSingleShot(true);
  points_timer_->setInterval(350);
  connect(points_timer_, &QTimer::timeout, this,
          &CoveragePanel::ApplyPointsEditor);
  connect(points_, &QPlainTextEdit::textChanged, this, [this] {
    if (!loading_ring_) {
      points_timer_->start();
    }
  });
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

  connect(preview_, &QPushButton::clicked, this,
          &CoveragePanel::PreviewCoverage);
  connect(start_, &QPushButton::clicked, this, &CoveragePanel::StartCoverage);
  connect(cancel_, &QPushButton::clicked, this, &CoveragePanel::CancelCoverage);
  timer_ = new QTimer(this);
  timer_->setInterval(100);
  connect(timer_, &QTimer::timeout, this, &CoveragePanel::PollActions);
  timer_->start();

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
    config.properties["color"] = "0;220;150";
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
  if (!sketch_writer_) {
    sketch_writer_ = node->CreateWriter<
        automsgs::msgs::visualization_msgs::MarkerArray>("/coverage/sketch");
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
        obstacles == 0 ? tr("No obstacle rings · optional")
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
  loading_ring_ = true;
  points_->setPlainText(valid ? FormatPoints(rings_[row].points) : QString());
  loading_ring_ = false;
  points_->setEnabled(valid && !busy_);
  points_->setStyleSheet(QString());
}

void CoveragePanel::ApplyPointsEditor() {
  const int row = ring_list_->currentRow();
  if (row < 0 || row >= static_cast<int>(rings_.size())) {
    return;
  }
  QString error;
  auto points = ParsePoints(points_->toPlainText(), &error);
  if (points.size() < 3) {
    points_->setStyleSheet(QStringLiteral("border:1px solid #c0392b;"));
    SetStatus(error.isEmpty() ? tr("A ring needs at least three points") : error,
              QStringLiteral("error"));
    return;
  }
  points_->setStyleSheet(QString());
  rings_[row].points = std::move(points);
  if (auto* item = ring_list_->item(row)) {
    const QSignalBlocker blocker(ring_list_);
    item->setText(RingLabel(rings_[row].name, rings_[row].points.size()));
  }
  PublishSketch();
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
  points_->setFocus();
  points_->selectAll();
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
  if (points_->hasFocus()) {
    ApplyPointsEditor();
  }
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
  preview_->setEnabled(!busy);
  start_->setEnabled(!busy);
  cancel_->setEnabled(busy);
  frame_->setEnabled(!busy);
  radius_->setEnabled(!busy);
  headland_->setEnabled(!busy);
  route_mode_->setEnabled(!busy);
  path_mode_->setEnabled(!busy);
  approach_->setEnabled(!busy);
  ring_list_->setEnabled(!busy);
  add_obstacle_->setEnabled(!busy);
  draw_obstacle_->setEnabled(!busy);
  clear_obstacles_->setEnabled(!busy);
  live_obstacles_->setEnabled(!busy);
  obstacle_margin_->setEnabled(!busy);
  refresh_obstacles_->setEnabled(!busy);
  advanced_->setEnabled(!busy);
  polygons_->setEnabled(!busy);
  auto_replan_->setEnabled(!busy && live_obstacles_->isChecked());
  if (!busy) {
    LoadSelectedRing(ring_list_->currentRow());
  } else {
    points_->setEnabled(false);
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
          preview_writer_->Write(result.result->nav_path());
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
          SetProgress(false);
          if (progress_value_ != nullptr) {
            progress_value_->setText(QStringLiteral("100%"));
          }
          if (remaining_value_ != nullptr) {
            remaining_value_->setText(QStringLiteral("0 m"));
          }
          SetStatus(tr("Complete coverage succeeded · %1 segment(s)")
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
  draw_obstacle_->setText(tr("Draw polygon (Ctrl+Alt+O)"));
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
}

void CoveragePanel::CancelObstacleDraft() {
  if (!drawing_) {
    return;
  }
  drawing_ = false;
  draft_.clear();
  finish_shortcut_->setEnabled(false);
  cancel_shortcut_->setEnabled(false);
  draw_obstacle_->setText(tr("Draw polygon (Ctrl+Alt+O)"));
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
    if (ring.kind == QLatin1String(kObstacleKind)) {
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
}  // namespace autoviz::navigation
#endif
