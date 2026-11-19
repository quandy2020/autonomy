// Copyright 2026 The Openbot Authors (duyongquan).
// SPDX-License-Identifier: Apache-2.0
#include "autoviz/ui/navigation/coverage_panel.hpp"
#ifdef AUTONOMY_BUILD_COVERAGE
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPointer>
#include <QPushButton>
#include <QTimer>
#include <QUuid>
#include <QVBoxLayout>
#include <cmath>

#include "autoviz/common/session_config.hpp"
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/plugins/displays/display.hpp"
namespace autoviz::navigation {
CoveragePanel::CoveragePanel(common::VisualizationManager* manager,
                             QWidget* parent)
    : QWidget(parent), manager_(manager) {
  setObjectName("CoveragePanel");
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 2, 0, 0);
  auto* hint = new QLabel(tr("Cover a complete region. First ring is the outer "
                             "boundary; following rings are obstacles. "
                             "Swath width equals twice the robot radius."),
                          this);
  hint->setWordWrap(true);
  layout->addWidget(hint);
  auto* form = new QFormLayout();
  frame_ = new QLineEdit("map", this);
  frame_->setObjectName("coverageFrame");
  form->addRow(tr("Frame"), frame_);
  radius_ = new QDoubleSpinBox(this);
  radius_->setObjectName("coverageRobotRadius");
  radius_->setRange(.05, 5.0);
  radius_->setSingleStep(.05);
  radius_->setDecimals(2);
  radius_->setValue(.4);
  radius_->setSuffix(" m");
  radius_->setToolTip(
      tr("Cleaning radius; every pass covers twice this width."));
  form->addRow(tr("Robot radius"), radius_);
  headland_ = new QDoubleSpinBox(this);
  headland_->setRange(.1, 20);
  headland_->setValue(1.2);
  headland_->setSuffix(" m");
  form->addRow(tr("Headland"), headland_);
  route_mode_ = new QComboBox(this);
  route_mode_->addItems({"BOUSTROPHEDON", "SNAKE", "SPIRAL", "TSP"});
  form->addRow(tr("Route"), route_mode_);
  path_mode_ = new QComboBox(this);
  path_mode_->addItems({"DUBIN", "REEDS_SHEPP"});
  form->addRow(tr("Turns"), path_mode_);
  layout->addLayout(form);
  polygons_ = new QPlainTextEdit(this);
  polygons_->setObjectName("coveragePolygons");
  polygons_->setPlainText("[[[-3,-2],[3,-2],[3,2],[-3,2],[-3,-2]]]");
  polygons_->setMinimumHeight(80);
  polygons_->setMaximumHeight(140);
  layout->addWidget(polygons_);
  auto* format = new QLabel(tr("Rings as JSON: [[[x,y], ...], [obstacle "
                               "points, ...]]. Coordinates in metres."),
                            this);
  format->setWordWrap(true);
  layout->addWidget(format);
  approach_ = new QCheckBox(tr("Navigate to coverage start"), this);
  approach_->setChecked(true);
  layout->addWidget(approach_);
  auto* buttons = new QHBoxLayout();
  preview_ = new QPushButton(tr("Preview full path"), this);
  preview_->setObjectName("coveragePreview");
  start_ = new QPushButton(tr("Start coverage"), this);
  start_->setObjectName("coverageStart");
  buttons->addWidget(preview_);
  buttons->addWidget(start_);
  layout->addLayout(buttons);
  cancel_ = new QPushButton(tr("Cancel coverage"), this);
  cancel_->setObjectName("coverageCancel");
  cancel_->setEnabled(false);
  layout->addWidget(cancel_);
  status_ = new QLabel(tr("Waiting for coverage server"), this);
  status_->setObjectName("coverageStatus");
  status_->setWordWrap(true);
  layout->addWidget(status_);
  layout->addStretch();
  connect(preview_, &QPushButton::clicked, this,
          &CoveragePanel::PreviewCoverage);
  connect(start_, &QPushButton::clicked, this, &CoveragePanel::StartCoverage);
  connect(cancel_, &QPushButton::clicked, this, &CoveragePanel::CancelCoverage);
  timer_ = new QTimer(this);
  timer_->setInterval(100);
  connect(timer_, &QTimer::timeout, this, &CoveragePanel::PollActions);
  timer_->start();
}
CoveragePanel::~CoveragePanel() { CancelCoverage(); }
void CoveragePanel::EnsureClients() {
  if (!manager_ || !manager_->autolinkNode()) return;
  auto node = manager_->autolinkNode();
  if (!compute_client_)
    compute_client_ =
        std::make_shared<ComputeClient>(node, "/compute_coverage_path");
  if (!navigate_client_)
    navigate_client_ =
        std::make_shared<NavigateClient>(node, "/navigate_complete_coverage");
  if (!preview_writer_)
    preview_writer_ =
        node->CreateWriter<automsgs::msgs::nav_msgs::Path>("/coverage/plan");
  bool exists = false;
  for (const auto* display : manager_->displays())
    if (display && display->channel() == "/coverage/plan") exists = true;
  if (!exists) {
    common::DisplayConfig c;
    c.type = "Path";
    c.name = "Full coverage path";
    c.channel = "/coverage/plan";
    c.enabled = true;
    c.properties["color"] = "0;220;150";
    manager_->addDisplay(c);
  }
  bool markers_exist = false;
  for (const auto* display : manager_->displays())
    if (display && display->channel() == "/coverage/markers") markers_exist = true;
  if (!markers_exist) {
    common::DisplayConfig config;
    config.type = "MarkerArray";
    config.name = "Coverage field and swaths";
    config.channel = "/coverage/markers";
    config.enabled = true;
    config.properties["use_custom_color"] = "false";
    manager_->addDisplay(config);
  }

}
autonomy::task::coverage::proto::CoverageRequest CoveragePanel::BuildRequest()
    const {
  auto document = QJsonDocument::fromJson(polygons_->toPlainText().toUtf8());
  if (!document.isArray() || document.array().isEmpty())
    throw std::invalid_argument("Provide at least one polygon ring");
  autonomy::task::coverage::proto::CoverageRequest request;
  request.set_frame_id(frame_->text().trimmed().toStdString());
  if (request.frame_id().empty()) throw std::invalid_argument("Frame is empty");
  for (const auto& value : document.array()) {
    if (!value.isArray() || value.toArray().size() < 3)
      throw std::invalid_argument("Each ring needs at least three points");
    auto* ring = request.add_polygons();
    for (const auto& coordinate : value.toArray()) {
      const auto p = coordinate.toArray();
      if (p.size() != 2 || !p[0].isDouble() || !p[1].isDouble())
        throw std::invalid_argument("Coordinates must be [x,y] numbers");
      auto* out = ring->add_coordinates();
      out->set_axis1(p[0].toDouble());
      out->set_axis2(p[1].toDouble());
    }
  }
  request.set_robot_radius(radius_->value());
  request.mutable_headland_mode()->set_mode("CONSTANT");
  request.mutable_headland_mode()->set_width(headland_->value());
  request.mutable_route_mode()->set_mode(
      route_mode_->currentText().toStdString());
  request.mutable_route_mode()->set_spiral_n(4);
  request.mutable_path_mode()->set_mode(
      path_mode_->currentText().toStdString());
  request.mutable_path_mode()->set_continuity_mode("DISCONTINUOUS");
  request.mutable_path_mode()->set_turn_point_distance(.05);
  request.set_coordinates_in_cartesian_frame(true);
  return request;
}
void CoveragePanel::SetBusy(bool busy) {
  busy_ = busy;
  preview_->setEnabled(!busy);
  start_->setEnabled(!busy);
  cancel_->setEnabled(busy);
  polygons_->setEnabled(!busy);
}
void CoveragePanel::PreviewCoverage() {
  try {
    EnsureClients();
    if (!compute_client_ || !compute_client_->ActionServerIsReady())
      throw std::runtime_error("Coverage planner is not ready");
    Compute::Goal goal;
    *goal.mutable_request() = BuildRequest();
    cancel_requested_ = false;
    compute_acceptance_ = compute_client_->AsyncSendGoal(goal);
    SetBusy(true);
    status_->setText(tr("Computing full coverage path…"));
  } catch (const std::exception& e) {
    status_->setText(QString::fromUtf8(e.what()));
  }
}
void CoveragePanel::StartCoverage() {
  try {
    EnsureClients();
    if (!navigate_client_ || !navigate_client_->ActionServerIsReady())
      throw std::runtime_error("Coverage navigator is not ready");
    Navigate::Goal goal;
    *goal.mutable_request() = BuildRequest();
    goal.set_task_id(
        QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString());
    goal.set_client_id("autoviz-navigation-panel");
    goal.set_navigate_to_start(approach_->isChecked());
    QPointer<CoveragePanel> self(this);
    NavigateClient::SendGoalOptions options;
    options.feedback_callback = [self](auto, const auto& feedback) {
      if (!self || !feedback) return;
      const double progress = feedback->progress(),
                   distance = feedback->distance_remaining();
      QMetaObject::invokeMethod(
          self,
          [self, progress, distance] {
            if (self)
              self->status_->setText(tr("Covering %1% · %2 m remaining")
                                         .arg(progress * 100, 0, 'f', 1)
                                         .arg(distance, 0, 'f', 1));
          },
          Qt::QueuedConnection);
    };
    cancel_requested_ = false;
    navigate_acceptance_ = navigate_client_->AsyncSendGoal(goal, options);
    SetBusy(true);
    status_->setText(tr("Starting complete coverage…"));
  } catch (const std::exception& e) {
    status_->setText(QString::fromUtf8(e.what()));
  }
}
void CoveragePanel::CancelCoverage() {
  cancel_requested_ = true;
  if (compute_goal_ && compute_client_)
    compute_client_->AsyncCancelGoal(compute_goal_);
  if (navigate_goal_ && navigate_client_)
    navigate_client_->AsyncCancelGoal(navigate_goal_);
  if (busy_) status_->setText(tr("Canceling coverage…"));
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
        status_->setText(tr("Coverage plan rejected"));
        SetBusy(false);
      } else if (cancel_requested_)
        compute_client_->AsyncCancelGoal(compute_goal_);
    }
    if (navigate_acceptance_.valid() &&
        navigate_acceptance_.wait_for(std::chrono::seconds(0)) ==
            std::future_status::ready) {
      navigate_goal_ = navigate_acceptance_.get();
      navigate_acceptance_ = {};
      if (!navigate_goal_) {
        status_->setText(tr("Coverage navigation rejected"));
        SetBusy(false);
      } else if (cancel_requested_)
        navigate_client_->AsyncCancelGoal(navigate_goal_);
    }
    if (compute_goal_) {
      auto future = compute_goal_->AsyncGetResult();
      if (future.wait_for(std::chrono::seconds(0)) ==
          std::future_status::ready) {
        const auto result = future.get();
        if (result.result && result.result->error_code() == 0) {
          preview_writer_->Write(result.result->nav_path());
          status_->setText(
              tr("Plan ready · %1 swaths · %2% core coverage")
                  .arg(result.result->coverage_path().swaths_size())
                  .arg(result.result->core_coverage() * 100, 0, 'f', 2));
        } else
          status_->setText(
              result.result ? QString::fromStdString(result.result->error_msg())
                            : tr("Coverage planning failed"));
        compute_goal_.reset();
        SetBusy(false);
      }
    }
    if (navigate_goal_) {
      auto future = navigate_goal_->AsyncGetResult();
      if (future.wait_for(std::chrono::seconds(0)) ==
          std::future_status::ready) {
        const auto result = future.get();
        status_->setText(
            result.result && result.result->error_code() == 0
                ? tr("Complete coverage succeeded")
            : result.result ? QString::fromStdString(result.result->error_msg())
                            : tr("Coverage navigation failed"));
        navigate_goal_.reset();
        SetBusy(false);
      }
    }
  } catch (const std::exception& e) {
    status_->setText(QString::fromUtf8(e.what()));
    SetBusy(false);
  }
}
}  // namespace autoviz::navigation
#endif
