/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <QApplication>
#include <QColor>
#include <QFont>
#include <QHBoxLayout>
#include <QLabel>
#include <QScrollArea>
#include <QTimer>
#include <QVBoxLayout>
#include <QVector>
#include <QVector3D>
#include <QWidget>

#include "autoviz/common/widget/widget.hpp"

namespace {

using autoviz::widget::ApplyWidgetTheme;
using autoviz::widget::Breadcrumb;
using autoviz::widget::BreadcrumbItem;
using autoviz::widget::Button;
using autoviz::widget::CollapsiblePanel;
using autoviz::widget::EmptyState;
using autoviz::widget::FeedbackLevel;
using autoviz::widget::FlowLayout;
using autoviz::widget::InfoBar;
using autoviz::widget::NumberField;
using autoviz::widget::Pagination;
using autoviz::widget::ProgressRing;
using autoviz::widget::SearchBox;
using autoviz::widget::SegmentedControl;
using autoviz::widget::Sparkline;
using autoviz::widget::StatusBadge;
using autoviz::widget::StepIndicator;
using autoviz::widget::StepItem;
using autoviz::widget::Surface;
using autoviz::widget::TimelineMarker;
using autoviz::widget::TimelineScrubber;
using autoviz::widget::ToggleSwitch;
using autoviz::widget::Vector3Editor;

QLabel* MakeTitle(const QString& text, QWidget* parent) {
  auto* title = new QLabel(text, parent);
  QFont font = title->font();
  font.setBold(true);
  font.setPointSizeF(font.pointSizeF() + 2.0);
  title->setFont(font);
  return title;
}

Surface* MakeSection(const QString& title, QWidget* parent,
                     QVBoxLayout** section_layout) {
  auto* section = new Surface(parent);
  section->SetElevation(Surface::Elevation::kRaised);
  section->setMinimumWidth(360);
  auto* layout = new QVBoxLayout(section);
  layout->setContentsMargins(16, 16, 16, 16);
  layout->setSpacing(12);
  layout->addWidget(MakeTitle(title, section));
  *section_layout = layout;
  return section;
}

QWidget* MakeInputSection(QWidget* parent) {
  QVBoxLayout* layout = nullptr;
  Surface* section = MakeSection(QObject::tr("Inputs"), parent, &layout);

  auto* search = new SearchBox(section);
  search->setPlaceholderText(QObject::tr("Filter channels"));
  layout->addWidget(search);

  auto* segments = new SegmentedControl(section);
  segments->AddSegment(QObject::tr("Live"));
  segments->AddSegment(QObject::tr("Paused"));
  segments->AddSegment(QObject::tr("Recorded"));
  segments->SetCurrentIndex(0);
  layout->addWidget(segments);

  auto* switch_row = new QWidget(section);
  auto* switch_layout = new QHBoxLayout(switch_row);
  switch_layout->setContentsMargins(0, 0, 0, 0);
  switch_layout->addWidget(new QLabel(QObject::tr("Follow latest frame"),
                                      switch_row));
  switch_layout->addStretch(1);
  auto* toggle = new ToggleSwitch(switch_row);
  toggle->setChecked(true);
  switch_layout->addWidget(toggle);
  layout->addWidget(switch_row);

  auto* number = new NumberField(section);
  number->setAccessibleName(QObject::tr("Point size"));
  number->setRange(0.1, 20.0);
  number->setSingleStep(0.1);
  number->setValue(2.5);
  number->setSuffix(QObject::tr(" px"));
  layout->addWidget(number);

  auto* vector = new Vector3Editor(section);
  vector->setAccessibleName(QObject::tr("Map origin"));
  vector->SetRange(-1000.0, 1000.0);
  vector->SetValue(QVector3D(1.0F, 2.0F, 0.0F));
  layout->addWidget(vector);

  auto* actions = new QWidget(section);
  auto* action_layout = new FlowLayout(actions, 0, 8, 8);
  auto* primary = new Button(QObject::tr("Apply"), actions);
  primary->SetVariant(Button::Variant::kPrimary);
  action_layout->addWidget(primary);
  action_layout->addWidget(new Button(QObject::tr("Reset"), actions));
  auto* danger = new Button(QObject::tr("Disconnect"), actions);
  danger->SetVariant(Button::Variant::kDanger);
  action_layout->addWidget(danger);
  layout->addWidget(actions);
  layout->addStretch(1);
  return section;
}

QWidget* MakeFeedbackSection(QWidget* parent) {
  QVBoxLayout* layout = nullptr;
  Surface* section = MakeSection(QObject::tr("Feedback and status"), parent,
                                 &layout);

  layout->addWidget(new InfoBar(QObject::tr("Connected"),
                                QObject::tr("Receiving localization data"),
                                FeedbackLevel::kSuccess, section));

  auto* badges = new QWidget(section);
  auto* badge_layout = new FlowLayout(badges, 0, 8, 8);
  badge_layout->addWidget(new StatusBadge(
      QObject::tr("Online"), StatusBadge::Status::kSuccess, badges));
  badge_layout->addWidget(new StatusBadge(
      QObject::tr("Degraded"), StatusBadge::Status::kWarning, badges));
  badge_layout->addWidget(new StatusBadge(
      QObject::tr("Offline"), StatusBadge::Status::kError, badges));
  layout->addWidget(badges);

  auto* metrics = new QWidget(section);
  auto* metric_layout = new QHBoxLayout(metrics);
  metric_layout->setContentsMargins(0, 0, 0, 0);
  auto* progress = new ProgressRing(metrics);
  progress->SetValue(72);
  metric_layout->addWidget(progress, 0, Qt::AlignTop);
  auto* sparkline = new Sparkline(metrics);
  sparkline->setAccessibleName(QObject::tr("Frame rate history"));
  sparkline->SetFillEnabled(true);
  sparkline->AppendSamples(
      {59.0, 58.0, 60.0, 57.0, 61.0, 60.0, 62.0, 59.0, 60.0});
  metric_layout->addWidget(sparkline, 1);
  layout->addWidget(metrics);

  auto* empty = new EmptyState(section);
  empty->SetTitle(QObject::tr("No selected topic"));
  empty->SetDescription(
      QObject::tr("Choose a topic to preview its latest message."));
  empty->setMaximumHeight(180);
  layout->addWidget(empty);
  return section;
}

QWidget* MakeNavigationSection(QWidget* parent) {
  QVBoxLayout* layout = nullptr;
  Surface* section = MakeSection(QObject::tr("Navigation and data"), parent,
                                 &layout);

  auto* breadcrumb = new Breadcrumb(section);
  breadcrumb->SetItems({{QObject::tr("Robot"), QStringLiteral("robot"), true},
                        {QObject::tr("Sensors"), QStringLiteral("sensors"),
                         true},
                        {QObject::tr("LiDAR"), QStringLiteral("lidar"), true}});
  breadcrumb->SetCurrentIndex(2);
  layout->addWidget(breadcrumb);

  auto* steps = new StepIndicator(section);
  steps->SetSteps({{QObject::tr("Connect"), true},
                   {QObject::tr("Calibrate"), true},
                   {QObject::tr("Map"), true}});
  steps->SetCurrentIndex(1);
  steps->SetInteractive(true);
  layout->addWidget(steps);

  auto* timeline = new TimelineScrubber(section);
  timeline->SetRange(0, 1000);
  timeline->SetValue(420);
  timeline->SetMarkers(
      {TimelineMarker{120, QColor(),
                      QObject::tr("Keyframe: Initial alignment")},
       TimelineMarker{760, QColor(),
                      QObject::tr("Loop: Loop closure accepted")}});
  layout->addWidget(timeline);

  auto* pagination = new Pagination(section);
  pagination->SetPageSizes({25, 50, 100});
  pagination->SetTotalItems(1284);
  pagination->SetCurrentPage(3);
  layout->addWidget(pagination);

  auto* details = new CollapsiblePanel(section);
  details->SetTitle(QObject::tr("Runtime details"));
  details->SetAnimationEnabled(false);
  auto* content = new QLabel(
      QObject::tr("Components use semantic palette roles and local painting."),
      details);
  content->setWordWrap(true);
  content->setContentsMargins(8, 8, 8, 8);
  details->SetContentWidget(content);
  layout->addWidget(details);
  layout->addStretch(1);
  return section;
}

class WidgetGallery final : public QScrollArea {
 public:
  WidgetGallery() {
    setWindowTitle(tr("Autoviz Widget Gallery"));
    setWidgetResizable(true);
    setMinimumSize(900, 640);
    resize(1280, 820);

    auto* content = new QWidget(this);
    auto* root_layout = new QVBoxLayout(content);
    root_layout->setContentsMargins(24, 24, 24, 24);
    root_layout->setSpacing(16);

    auto* heading = MakeTitle(tr("Autoviz component gallery"), content);
    root_layout->addWidget(heading);
    auto* summary = new QLabel(
        tr("Keyboard-accessible Qt 6 widgets using the shared Autoviz theme."),
        content);
    summary->setWordWrap(true);
    root_layout->addWidget(summary);

    auto* sections = new QWidget(content);
    auto* flow = new FlowLayout(sections, 0, 16, 16);
    flow->addWidget(MakeInputSection(sections));
    flow->addWidget(MakeFeedbackSection(sections));
    flow->addWidget(MakeNavigationSection(sections));
    root_layout->addWidget(sections, 1);
    setWidget(content);
    ApplyWidgetTheme(content);
  }
};

}  // namespace

int main(int argc, char* argv[]) {
  QApplication application(argc, argv);
  application.setApplicationName(QStringLiteral("Autoviz Widget Gallery"));

  WidgetGallery gallery;
  gallery.show();
  if (application.arguments().contains(QStringLiteral("--smoke-test"))) {
    QTimer::singleShot(0, &application, &QCoreApplication::quit);
  }
  return application.exec();
}
