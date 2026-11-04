/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <QApplication>
#include <QByteArray>
#include <QColor>
#include <QDateTime>
#include <QFont>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QObject>
#include <QScrollArea>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QStringList>
#include <QStyle>
#include <QTimer>
#include <QVBoxLayout>
#include <QVector3D>
#include <QVector>
#include <QWidget>
#include <memory>
#include <utility>

#include "autoviz/common/widget/widget.hpp"
#include "autoviz/ui/theme/application.hpp"

namespace {

using autoviz::widget::ApplyWidgetTheme;
using autoviz::widget::AutoSuggestBox;
using autoviz::widget::Avatar;
using autoviz::widget::Breadcrumb;
using autoviz::widget::BreadcrumbItem;
using autoviz::widget::BuildWidgetPalette;
using autoviz::widget::Button;
using autoviz::widget::CollapsiblePanel;
using autoviz::widget::ColorPicker;
using autoviz::widget::CommandBar;
using autoviz::widget::CommandPalette;
using autoviz::widget::DataGrid;
using autoviz::widget::DockArea;
using autoviz::widget::DockManager;
using autoviz::widget::Drawer;
using autoviz::widget::EmptyState;
using autoviz::widget::FeedbackLevel;
using autoviz::widget::FileDropZone;
using autoviz::widget::FilterBar;
using autoviz::widget::FlowLayout;
using autoviz::widget::FormController;
using autoviz::widget::IconButton;
using autoviz::widget::InfoBar;
using autoviz::widget::ListDetailView;
using autoviz::widget::LoadingOverlay;
using autoviz::widget::LogEntry;
using autoviz::widget::LogLevel;
using autoviz::widget::LogView;
using autoviz::widget::MultiSelectComboBox;
using autoviz::widget::NavigationItem;
using autoviz::widget::NavigationRail;
using autoviz::widget::NumberField;
using autoviz::widget::Pagination;
using autoviz::widget::PasswordBox;
using autoviz::widget::PipsPager;
using autoviz::widget::Popover;
using autoviz::widget::ProgressBar;
using autoviz::widget::ProgressRing;
using autoviz::widget::PropertyEditor;
using autoviz::widget::PropertySpec;
using autoviz::widget::PropertyType;
using autoviz::widget::Rating;
using autoviz::widget::SearchBox;
using autoviz::widget::SegmentedControl;
using autoviz::widget::Separator;
using autoviz::widget::ShortcutRecorder;
using autoviz::widget::Skeleton;
using autoviz::widget::Sparkline;
using autoviz::widget::StatusBadge;
using autoviz::widget::StepIndicator;
using autoviz::widget::StepItem;
using autoviz::widget::Surface;
using autoviz::widget::Tag;
using autoviz::widget::TeachingTip;
using autoviz::widget::ThemeAppearance;
using autoviz::widget::TimelineMarker;
using autoviz::widget::TimelineScrubber;
using autoviz::widget::Toast;
using autoviz::widget::ToggleSwitch;
using autoviz::widget::TreeSelect;
using autoviz::widget::Vector3Editor;
using autoviz::widget::Watermark;

enum class CaptureTarget {
  kGallery,
  kDock,
  kCommandPalette,
  kToast,
  kDrawer,
  kPopover,
  kTeachingTip,
  kStates,
};

class DockSmokeHarness final : public QObject {
 public:
  DockSmokeHarness(ThemeAppearance appearance, QObject* parent)
      : QObject(parent),
        host_(std::make_unique<QMainWindow>()),
        manager_(std::make_unique<DockManager>(host_.get())) {
    host_->setObjectName(QStringLiteral("AutovizGalleryDockHost"));
    host_->setWindowTitle(tr("Autoviz docking preview"));
    host_->setAttribute(Qt::WA_DontShowOnScreen);
    host_->resize(720, 480);

    auto* canvas = new QLabel(tr("Map canvas"), host_.get());
    canvas->setAlignment(Qt::AlignCenter);
    host_->setCentralWidget(canvas);

    auto* content = new QLabel(tr("Inspector controls"));
    content->setAlignment(Qt::AlignCenter);
    manager_->AddPanel(QStringLiteral("gallery.inspector"), tr("Inspector"),
                       content, DockArea::kRight);
    auto* timeline = new QLabel(tr("Timeline and playback controls"));
    timeline->setAlignment(Qt::AlignCenter);
    manager_->AddPanel(QStringLiteral("gallery.timeline"), tr("Timeline"),
                       timeline, DockArea::kBottom);
    ApplyWidgetTheme(host_.get(), autoviz::style::Tone::Shell, true,
                     appearance);

    const QByteArray layout_state = manager_->SaveLayout(1);
    manager_->RestoreLayout(layout_state, 1);
    manager_->SetPanelVisible(QStringLiteral("gallery.inspector"), false);
    manager_->SetPanelVisible(QStringLiteral("gallery.timeline"), false);
    host_->hide();
  }

  QWidget* ShowForCapture() {
    manager_->SetPanelVisible(QStringLiteral("gallery.inspector"), true);
    manager_->SetPanelVisible(QStringLiteral("gallery.timeline"), true);
    host_->show();
    host_->raise();
    return host_.get();
  }

  QWidget* widget() const noexcept { return host_.get(); }

 private:
  std::unique_ptr<QMainWindow> host_;
  std::unique_ptr<DockManager> manager_;
};

class StateCaptureHarness final : public QObject {
 public:
  StateCaptureHarness(ThemeAppearance appearance, QObject* parent)
      : QObject(parent), host_(std::make_unique<Surface>()) {
    host_->setObjectName(QStringLiteral("AutovizGalleryStateHost"));
    host_->setWindowTitle(tr("Autoviz interaction states"));
    host_->SetElevation(Surface::Elevation::kRaised);
    host_->resize(760, 240);

    auto* layout = new QGridLayout(host_.get());
    layout->setContentsMargins(24, 24, 24, 24);
    layout->setHorizontalSpacing(16);
    layout->setVerticalSpacing(8);

    auto* normal = new Button(tr("Apply"), host_.get());
    focused_search_ = new SearchBox(host_.get());
    focused_search_->setPlaceholderText(tr("Focused search"));
    auto* selected = new ToggleSwitch(host_.get());
    selected->setAccessibleName(tr("Selected toggle"));
    selected->setChecked(true);

    auto* disabled = new PasswordBox(host_.get());
    disabled->setAccessibleName(tr("Disabled password"));
    disabled->setText(QStringLiteral("disabled"));
    disabled->setEnabled(false);

    auto* invalid_search = new SearchBox(host_.get());
    invalid_search->setAccessibleName(tr("Invalid frame"));
    invalid_search->setText(tr("Unknown frame"));
    auto* invalid_number = new NumberField(host_.get());
    invalid_number->setAccessibleName(tr("Invalid voxel size"));
    invalid_number->setRange(0.01, 1.0);
    invalid_number->setValue(0.05);

    auto* form = new FormController(host_.get());
    form->RegisterField(invalid_search);
    form->RegisterField(invalid_number);
    form->SetFieldError(invalid_search, tr("Frame is unavailable"));
    form->SetFieldError(invalid_number, tr("Minimum supported size is 0.1"));

    const auto add_specimen = [layout, this](int row, int column,
                                             const QString& caption,
                                             QWidget* control) {
      auto* label = new QLabel(caption, host_.get());
      layout->addWidget(label, row * 2, column);
      layout->addWidget(control, row * 2 + 1, column);
      layout->setColumnStretch(column, 1);
    };
    add_specimen(0, 0, tr("Normal"), normal);
    add_specimen(0, 1, tr("Focused"), focused_search_);
    add_specimen(0, 2, tr("Selected"), selected);
    add_specimen(1, 0, tr("Disabled"), disabled);
    add_specimen(1, 1, tr("Invalid text"), invalid_search);
    add_specimen(1, 2, tr("Invalid number"), invalid_number);

    ApplyWidgetTheme(host_.get(), autoviz::style::Tone::Shell, true,
                     appearance);
    host_->hide();
  }

  QWidget* ShowForCapture() {
    host_->show();
    host_->raise();
    host_->activateWindow();
    focused_search_->setFocus(Qt::OtherFocusReason);
    return host_.get();
  }

  QWidget* widget() const noexcept { return host_.get(); }

 private:
  std::unique_ptr<Surface> host_;
  SearchBox* focused_search_ = nullptr;
};

struct OverlayCaptureTargets {
  CommandPalette* command_palette = nullptr;
  Toast* toast = nullptr;
  Drawer* drawer = nullptr;
  Popover* popover = nullptr;
  Button* popover_anchor = nullptr;
  TeachingTip* teaching_tip = nullptr;
  Button* teaching_tip_anchor = nullptr;
};

QString CapturePath(const QStringList& arguments) {
  for (int index = 1; index < arguments.size(); ++index) {
    const QString& argument = arguments[index];
    if (argument == QStringLiteral("--capture") &&
        index + 1 < arguments.size()) {
      return arguments[index + 1];
    }
    constexpr auto kCapturePrefix = "--capture=";
    if (argument.startsWith(QLatin1String(kCapturePrefix))) {
      return argument.mid(QLatin1String(kCapturePrefix).size());
    }
  }
  return {};
}

CaptureTarget CaptureTargetFromArguments(const QStringList& arguments) {
  QString value;
  for (int index = 1; index < arguments.size(); ++index) {
    const QString& argument = arguments[index];
    if (argument == QStringLiteral("--capture-target") &&
        index + 1 < arguments.size()) {
      value = arguments[++index];
      break;
    }
    constexpr auto kTargetPrefix = "--capture-target=";
    if (argument.startsWith(QLatin1String(kTargetPrefix))) {
      value = argument.mid(QLatin1String(kTargetPrefix).size());
      break;
    }
  }

  if (value.compare(QStringLiteral("dock"), Qt::CaseInsensitive) == 0) {
    return CaptureTarget::kDock;
  }
  if (value.compare(QStringLiteral("command-palette"), Qt::CaseInsensitive) ==
      0) {
    return CaptureTarget::kCommandPalette;
  }
  if (value.compare(QStringLiteral("toast"), Qt::CaseInsensitive) == 0) {
    return CaptureTarget::kToast;
  }
  if (value.compare(QStringLiteral("drawer"), Qt::CaseInsensitive) == 0) {
    return CaptureTarget::kDrawer;
  }
  if (value.compare(QStringLiteral("popover"), Qt::CaseInsensitive) == 0) {
    return CaptureTarget::kPopover;
  }
  if (value.compare(QStringLiteral("teaching-tip"), Qt::CaseInsensitive) == 0) {
    return CaptureTarget::kTeachingTip;
  }
  if (value.compare(QStringLiteral("states"), Qt::CaseInsensitive) == 0) {
    return CaptureTarget::kStates;
  }
  return CaptureTarget::kGallery;
}

ThemeAppearance AppearanceFromArguments(const QStringList& arguments) {
  for (int index = 1; index < arguments.size(); ++index) {
    const QString& argument = arguments[index];
    QString value;
    if (argument == QStringLiteral("--appearance") &&
        index + 1 < arguments.size()) {
      value = arguments[++index];
    } else {
      constexpr auto kAppearancePrefix = "--appearance=";
      if (argument.startsWith(QLatin1String(kAppearancePrefix))) {
        value = argument.mid(QLatin1String(kAppearancePrefix).size());
      }
    }

    if (value.compare(QStringLiteral("light"), Qt::CaseInsensitive) == 0) {
      return ThemeAppearance::kLight;
    }
    if (value.compare(QStringLiteral("dark"), Qt::CaseInsensitive) == 0) {
      return ThemeAppearance::kDark;
    }
    if (value.compare(QStringLiteral("system"), Qt::CaseInsensitive) == 0) {
      return ThemeAppearance::kSystem;
    }
  }
  return ThemeAppearance::kSystem;
}

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

  auto* suggestions = new AutoSuggestBox(section);
  suggestions->setAccessibleName(QObject::tr("Suggested channel"));
  suggestions->setPlaceholderText(QObject::tr("Choose a channel"));
  auto* suggestion_model = new QStandardItemModel(suggestions);
  for (const QString& channel :
       {QStringLiteral("/camera/image"), QStringLiteral("/localization/pose"),
        QStringLiteral("/tf")}) {
    auto* item = new QStandardItem(channel);
    item->setData(channel, Qt::UserRole);
    suggestion_model->appendRow(item);
  }
  suggestions->SetModel(suggestion_model);
  layout->addWidget(suggestions);

  auto* password = new PasswordBox(section);
  password->setAccessibleName(QObject::tr("Robot access token"));
  password->setPlaceholderText(QObject::tr("Robot access token"));
  layout->addWidget(password);

  auto* segments = new SegmentedControl(section);
  segments->AddSegment(QObject::tr("Live"));
  segments->AddSegment(QObject::tr("Paused"));
  segments->AddSegment(QObject::tr("Recorded"));
  segments->SetCurrentIndex(0);
  layout->addWidget(segments);

  auto* switch_row = new QWidget(section);
  auto* switch_layout = new QHBoxLayout(switch_row);
  switch_layout->setContentsMargins(0, 0, 0, 0);
  switch_layout->addWidget(
      new QLabel(QObject::tr("Follow latest frame"), switch_row));
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

  auto* color = new ColorPicker(section);
  color->setAccessibleName(QObject::tr("Trajectory color"));
  color->SetColor(QColor(QStringLiteral("#2F80ED")));
  layout->addWidget(color);

  auto* drop_zone = new FileDropZone(section);
  drop_zone->SetPrompt(QObject::tr("Drop a bag or map file here"));
  drop_zone->SetAllowedExtensions(
      {QStringLiteral("bag"), QStringLiteral("db3"), QStringLiteral("pcd")});
  layout->addWidget(drop_zone);

  auto* multi_select = new MultiSelectComboBox(section);
  multi_select->SetItems(
      {{QObject::tr("Camera"), QStringLiteral("camera"), true},
       {QObject::tr("LiDAR"), QStringLiteral("lidar"), true},
       {QObject::tr("IMU"), QStringLiteral("imu"), true}});
  multi_select->SetSelectedIndices({0, 1});
  layout->addWidget(multi_select);

  auto* tree_select = new TreeSelect(section);
  tree_select->SetItems(
      {{QStringLiteral("sensors"),
        QString(),
        QObject::tr("Sensors"),
        {},
        true,
        false},
       {QStringLiteral("camera"), QStringLiteral("sensors"),
        QObject::tr("Camera"), QStringLiteral("camera"), true, true},
       {QStringLiteral("lidar"), QStringLiteral("sensors"),
        QObject::tr("LiDAR"), QStringLiteral("lidar"), true, true}});
  tree_select->SetCurrentId(QStringLiteral("lidar"));
  layout->addWidget(tree_select);

  auto* rating = new Rating(section);
  rating->setAccessibleName(QObject::tr("Map quality"));
  rating->SetValue(4);
  layout->addWidget(rating);

  auto* shortcut = new ShortcutRecorder(section);
  shortcut->setAccessibleName(QObject::tr("Reset view shortcut"));
  shortcut->SetKeySequence(QKeySequence(Qt::CTRL | Qt::Key_R));
  shortcut->SetReservedSequences({QKeySequence(QKeySequence::Quit)});
  layout->addWidget(shortcut);

  auto* actions = new QWidget(section);
  auto* action_layout = new FlowLayout(actions, 0, 8, 8);
  auto* refresh =
      new IconButton(section->style()->standardIcon(QStyle::SP_BrowserReload),
                     QObject::tr("Refresh"), actions);
  action_layout->addWidget(refresh);
  auto* primary = new Button(QObject::tr("Apply"), actions);
  primary->SetVariant(Button::Variant::kPrimary);
  action_layout->addWidget(primary);
  action_layout->addWidget(new Button(QObject::tr("Reset"), actions));
  auto* danger = new Button(QObject::tr("Disconnect"), actions);
  danger->SetVariant(Button::Variant::kDanger);
  action_layout->addWidget(danger);
  layout->addWidget(actions);

  auto* form = new FormController(section);
  form->RegisterField(search, true);
  form->RegisterField(number, true);
  form->SetSubmitButton(primary);
  layout->addStretch(1);
  return section;
}

QWidget* MakeFeedbackSection(QWidget* parent) {
  QVBoxLayout* layout = nullptr;
  Surface* section =
      MakeSection(QObject::tr("Feedback and status"), parent, &layout);

  auto* information =
      new InfoBar(QObject::tr("Localization ready"),
                  QObject::tr("Select a map source to begin streaming."),
                  FeedbackLevel::kInformation, section);
  information->SetClosable(false);
  layout->addWidget(information);
  layout->addWidget(new InfoBar(QObject::tr("Connected"),
                                QObject::tr("Receiving localization data"),
                                FeedbackLevel::kSuccess, section));
  layout->addWidget(
      new InfoBar(QObject::tr("Frame delay"),
                  QObject::tr("LiDAR frames are arriving 120 ms late."),
                  FeedbackLevel::kWarning, section));
  layout->addWidget(
      new InfoBar(QObject::tr("Sensor unavailable"),
                  QObject::tr("Camera stream stopped. Check the connection."),
                  FeedbackLevel::kError, section));

  auto* identity = new QWidget(section);
  auto* identity_layout = new FlowLayout(identity, 0, 8, 8);
  auto* avatar = new Avatar(identity);
  avatar->SetName(QObject::tr("Autonomy Robot"));
  avatar->SetDiameter(44);
  identity_layout->addWidget(avatar);
  auto* mapping_tag = new Tag(QObject::tr("Mapping"), identity);
  mapping_tag->SetKind(Tag::Kind::kAccent);
  mapping_tag->setCheckable(true);
  mapping_tag->setChecked(true);
  identity_layout->addWidget(mapping_tag);
  auto* removable_tag = new Tag(QObject::tr("Loop closure"), identity);
  removable_tag->SetKind(Tag::Kind::kSuccess);
  removable_tag->SetRemovable(true);
  identity_layout->addWidget(removable_tag);
  layout->addWidget(identity);

  auto* badges = new QWidget(section);
  auto* badge_layout = new FlowLayout(badges, 0, 8, 8);
  badge_layout->addWidget(new StatusBadge(
      QObject::tr("Online"), StatusBadge::Status::kSuccess, badges));
  badge_layout->addWidget(new StatusBadge(
      QObject::tr("Degraded"), StatusBadge::Status::kWarning, badges));
  badge_layout->addWidget(new StatusBadge(QObject::tr("Offline"),
                                          StatusBadge::Status::kError, badges));
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

  auto* linear_progress = new ProgressBar(section);
  linear_progress->SetState(ProgressBar::State::kSuccess);
  linear_progress->setRange(0, 100);
  linear_progress->setValue(72);
  linear_progress->setFormat(QObject::tr("Map optimization %p%"));
  layout->addWidget(linear_progress);

  layout->addWidget(new Separator(Qt::Horizontal, section));

  auto* skeleton = new Skeleton(section);
  skeleton->setAccessibleName(QObject::tr("Loading placeholder"));
  skeleton->setFixedHeight(20);
  layout->addWidget(skeleton);

  auto* watermark = new Watermark(section);
  watermark->SetText(QObject::tr("AUTOVIZ PREVIEW"));
  watermark->SetOpacity(0.12);
  watermark->setFixedHeight(96);
  layout->addWidget(watermark);

  auto* loading_target = new Surface(section);
  loading_target->SetElevation(Surface::Elevation::kFlat);
  loading_target->setMinimumHeight(96);
  auto* loading_layout = new QVBoxLayout(loading_target);
  auto* loading_label =
      new QLabel(QObject::tr("Map layer preview"), loading_target);
  loading_label->setAlignment(Qt::AlignCenter);
  loading_layout->addWidget(loading_label);
  layout->addWidget(loading_target);
  auto* loading_overlay = new LoadingOverlay(loading_target);
  loading_overlay->SetMessage(QObject::tr("Loading map layer…"));
  loading_overlay->SetLoading(true);

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
  Surface* section =
      MakeSection(QObject::tr("Navigation and data"), parent, &layout);

  auto* rail = new NavigationRail(section);
  rail->SetItems(
      {{QObject::tr("Overview"),
        section->style()->standardIcon(QStyle::SP_ComputerIcon),
        QStringLiteral("overview"), true},
       {QObject::tr("Sensors"),
        section->style()->standardIcon(QStyle::SP_DriveHDIcon),
        QStringLiteral("sensors"), true},
       {QObject::tr("Diagnostics"),
        section->style()->standardIcon(QStyle::SP_MessageBoxInformation),
        QStringLiteral("diagnostics"), true}});
  rail->SetCurrentIndex(0);
  rail->setMinimumHeight(148);
  layout->addWidget(rail);

  auto* breadcrumb = new Breadcrumb(section);
  breadcrumb->SetItems(
      {{QObject::tr("Robot"), QStringLiteral("robot"), true},
       {QObject::tr("Sensors"), QStringLiteral("sensors"), true},
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

  auto* pips = new PipsPager(section);
  pips->setAccessibleName(QObject::tr("Camera page"));
  pips->SetPageCount(9);
  pips->SetCurrentPage(3);
  layout->addWidget(pips);

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

QWidget* MakeDataSection(QWidget* parent) {
  QVBoxLayout* layout = nullptr;
  Surface* section =
      MakeSection(QObject::tr("Data and properties"), parent, &layout);

  auto* filter = new FilterBar(section);
  filter->setAccessibleName(QObject::tr("Filter data grid"));
  layout->addWidget(filter);

  auto* grid = new DataGrid(section);
  grid->setMinimumHeight(160);
  auto* grid_model = new QStandardItemModel(0, 3, grid);
  grid_model->setHorizontalHeaderLabels(
      {QObject::tr("Channel"), QObject::tr("Rate"), QObject::tr("State")});
  grid_model->appendRow({new QStandardItem(QStringLiteral("/camera/image")),
                         new QStandardItem(QStringLiteral("30 Hz")),
                         new QStandardItem(QObject::tr("Online"))});
  grid_model->appendRow(
      {new QStandardItem(QStringLiteral("/localization/pose")),
       new QStandardItem(QStringLiteral("100 Hz")),
       new QStandardItem(QObject::tr("Online"))});
  grid_model->appendRow({new QStandardItem(QStringLiteral("/diagnostics")),
                         new QStandardItem(QStringLiteral("1 Hz")),
                         new QStandardItem(QObject::tr("Degraded"))});
  grid->SetSourceModel(grid_model);
  QObject::connect(
      filter, &FilterBar::QueryChanged, grid,
      [grid](const QString& query) { grid->SetFilterText(query); });
  layout->addWidget(grid);

  auto* log = new LogView(section);
  log->setMinimumHeight(180);
  log->SetMaximumEntries(100);
  log->AppendBatch(
      {{QDateTime::fromSecsSinceEpoch(1), LogLevel::kInformation,
        QObject::tr("Localization"), QObject::tr("Atlas initialized")},
       {QDateTime::fromSecsSinceEpoch(2), LogLevel::kWarning,
        QObject::tr("Mapping"), QObject::tr("Loop candidate pending")},
       {QDateTime::fromSecsSinceEpoch(3), LogLevel::kError,
        QObject::tr("Sensor"), QObject::tr("LiDAR packet delayed")}});
  layout->addWidget(log);

  PropertySpec voxel_size;
  voxel_size.id = QStringLiteral("voxel_size");
  voxel_size.name = QObject::tr("Voxel size");
  voxel_size.description = QObject::tr("Dense map voxel size in meters");
  voxel_size.type = PropertyType::kDouble;
  voxel_size.value = 0.05;
  voxel_size.minimum = 0.01;
  voxel_size.maximum = 1.0;
  voxel_size.step = 0.01;

  PropertySpec publish_tf;
  publish_tf.id = QStringLiteral("publish_tf");
  publish_tf.name = QObject::tr("Publish TF");
  publish_tf.type = PropertyType::kBoolean;
  publish_tf.value = true;

  PropertySpec mapping;
  mapping.id = QStringLiteral("mapping");
  mapping.name = QObject::tr("Mapping");
  mapping.type = PropertyType::kGroup;
  mapping.children = {voxel_size, publish_tf};

  auto* properties = new PropertyEditor(section);
  properties->setMinimumHeight(170);
  properties->SetProperties({mapping});
  layout->addWidget(properties);

  auto* list_detail = new ListDetailView(section);
  list_detail->setMinimumHeight(150);
  auto* list_model = new QStandardItemModel(list_detail);
  list_model->appendRow(new QStandardItem(QObject::tr("Front camera")));
  list_model->appendRow(new QStandardItem(QObject::tr("Roof LiDAR")));
  list_model->appendRow(new QStandardItem(QObject::tr("Localization")));
  list_detail->SetModel(list_model);
  auto* detail_label =
      new QLabel(QObject::tr("Live sensor details"), list_detail);
  detail_label->setAlignment(Qt::AlignCenter);
  list_detail->SetDetailWidget(detail_label);
  list_detail->SetCurrentIndex(list_model->index(0, 0));
  layout->addWidget(list_detail);
  return section;
}

QWidget* MakeOverlaySection(QWidget* parent,
                            OverlayCaptureTargets* capture_targets) {
  QVBoxLayout* layout = nullptr;
  Surface* section =
      MakeSection(QObject::tr("Overlays and commands"), parent, &layout);

  auto* description = new QLabel(
      QObject::tr("These controls stay hidden until their trigger is used."),
      section);
  description->setWordWrap(true);
  layout->addWidget(description);

  auto* command_bar = new CommandBar(section);
  command_bar->AddCommand(
      QObject::tr("Open"),
      section->style()->standardIcon(QStyle::SP_DialogOpenButton));
  command_bar->AddCommand(
      QObject::tr("Save"),
      section->style()->standardIcon(QStyle::SP_DialogSaveButton));
  command_bar->AddCommand(
      QObject::tr("Refresh"),
      section->style()->standardIcon(QStyle::SP_BrowserReload));
  layout->addWidget(command_bar);

  auto* actions = new QWidget(section);
  auto* action_layout = new FlowLayout(actions, 0, 8, 8);

  auto* command_button = new Button(QObject::tr("Commands"), actions);
  action_layout->addWidget(command_button);
  auto* command_palette = new CommandPalette(section);
  auto* command_model = new QStandardItemModel(command_palette);
  for (const auto& command :
       {std::pair{QStringLiteral("reset_view"), QObject::tr("Reset view")},
        std::pair{QStringLiteral("save_layout"), QObject::tr("Save layout")},
        std::pair{QStringLiteral("open_bag"), QObject::tr("Open bag")}}) {
    auto* item = new QStandardItem(command.second);
    item->setData(command.first, Qt::UserRole + 1);
    item->setData(command.second, Qt::UserRole + 2);
    command_model->appendRow(item);
  }
  command_palette->SetModel(command_model);
  QObject::connect(command_button, &Button::clicked, command_palette,
                   &CommandPalette::ShowPalette);
  capture_targets->command_palette = command_palette;

  auto* toast_button = new Button(QObject::tr("Toast"), actions);
  action_layout->addWidget(toast_button);
  auto* toast = new Toast(section);
  toast->SetMessage(QObject::tr("Map export completed"));
  toast->SetLevel(FeedbackLevel::kSuccess);
  toast->SetDurationMs(0);
  toast->hide();
  QObject::connect(toast_button, &Button::clicked, toast, &Toast::Show);
  capture_targets->toast = toast;

  auto* drawer_button = new Button(QObject::tr("Drawer"), actions);
  action_layout->addWidget(drawer_button);
  auto* drawer = new Drawer(section);
  drawer->SetPanelExtent(280);
  auto* drawer_content =
      new QLabel(QObject::tr("Drawer settings content"), drawer);
  drawer_content->setAlignment(Qt::AlignCenter);
  drawer->SetContentWidget(drawer_content);
  QObject::connect(drawer_button, &Button::clicked, drawer, &Drawer::Show);
  capture_targets->drawer = drawer;

  auto* popover_button = new Button(QObject::tr("Popover"), actions);
  action_layout->addWidget(popover_button);
  auto* popover = new Popover(section);
  auto* popover_content =
      new QLabel(QObject::tr("Anchored popover content"), popover);
  popover_content->setWordWrap(true);
  popover->SetContentWidget(popover_content);
  QObject::connect(
      popover_button, &Button::clicked, popover,
      [popover, popover_button]() { popover->ShowFor(popover_button); });
  capture_targets->popover = popover;
  capture_targets->popover_anchor = popover_button;

  auto* teaching_button = new Button(QObject::tr("Teaching tip"), actions);
  action_layout->addWidget(teaching_button);
  auto* teaching_tip = new TeachingTip(section);
  teaching_tip->SetTitle(QObject::tr("Channel browser"));
  teaching_tip->SetMessage(
      QObject::tr("Drag numeric fields from Channels into Plot."));
  teaching_tip->SetActionText(QObject::tr("Got it"));
  teaching_tip->SetDismissText(QObject::tr("Dismiss"));
  QObject::connect(teaching_button, &Button::clicked, teaching_tip,
                   [teaching_tip, teaching_button]() {
                     teaching_tip->ShowFor(teaching_button);
                   });
  capture_targets->teaching_tip = teaching_tip;
  capture_targets->teaching_tip_anchor = teaching_button;

  layout->addWidget(actions);
  layout->addStretch(1);
  return section;
}

class WidgetGallery final : public QScrollArea {
 public:
  explicit WidgetGallery(ThemeAppearance appearance) {
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
    auto* summary = new QLabel(tr("Keyboard-accessible Qt 6 widgets using "
                                  "the shared Autoviz theme."),
                               content);
    summary->setWordWrap(true);
    root_layout->addWidget(summary);

    auto* sections = new QWidget(content);
    auto* flow = new FlowLayout(sections, 0, 16, 16);
    flow->addWidget(MakeInputSection(sections));
    flow->addWidget(MakeFeedbackSection(sections));
    flow->addWidget(MakeNavigationSection(sections));
    flow->addWidget(MakeDataSection(sections));
    flow->addWidget(MakeOverlaySection(sections, &overlay_capture_targets_));
    root_layout->addWidget(sections, 1);
    setWidget(content);
    ApplyWidgetTheme(this, autoviz::style::Tone::Shell, true, appearance);

    // DockManager is window-oriented. Keep its host hidden for the default
    // gallery, and reveal it only for an explicit dock capture.
    dock_harness_ = new DockSmokeHarness(appearance, this);
    state_harness_ = new StateCaptureHarness(appearance, this);
  }

  void ShowCaptureTarget(CaptureTarget target) {
    switch (target) {
      case CaptureTarget::kDock:
        dock_harness_->ShowForCapture();
        break;
      case CaptureTarget::kCommandPalette:
        overlay_capture_targets_.command_palette->ShowPalette();
        break;
      case CaptureTarget::kToast:
        overlay_capture_targets_.toast->Show();
        break;
      case CaptureTarget::kDrawer:
        overlay_capture_targets_.drawer->Show();
        break;
      case CaptureTarget::kPopover:
        overlay_capture_targets_.popover->ShowFor(
            overlay_capture_targets_.popover_anchor);
        break;
      case CaptureTarget::kTeachingTip:
        overlay_capture_targets_.teaching_tip->ShowFor(
            overlay_capture_targets_.teaching_tip_anchor);
        break;
      case CaptureTarget::kStates:
        state_harness_->ShowForCapture();
        break;
      case CaptureTarget::kGallery:
        break;
    }
  }

  QWidget* CaptureWidget(CaptureTarget target) const {
    switch (target) {
      case CaptureTarget::kDock:
        return dock_harness_->widget();
      case CaptureTarget::kCommandPalette:
        return overlay_capture_targets_.command_palette;
      case CaptureTarget::kToast:
        return overlay_capture_targets_.toast->parentWidget();
      case CaptureTarget::kDrawer:
        return overlay_capture_targets_.drawer->parentWidget();
      case CaptureTarget::kPopover:
        return overlay_capture_targets_.popover;
      case CaptureTarget::kTeachingTip:
        if (QWidget* active_popup = QApplication::activePopupWidget();
            active_popup != nullptr) {
          return active_popup;
        }
        for (QWidget* top_level : QApplication::topLevelWidgets()) {
          if (top_level->isVisible() &&
              top_level->objectName() == QStringLiteral("AutovizPopover")) {
            return top_level;
          }
        }
        return nullptr;
      case CaptureTarget::kStates:
        return state_harness_->widget();
      case CaptureTarget::kGallery:
        return widget();
    }
    return nullptr;
  }

 private:
  OverlayCaptureTargets overlay_capture_targets_;
  DockSmokeHarness* dock_harness_ = nullptr;
  StateCaptureHarness* state_harness_ = nullptr;
};

}  // namespace

int main(int argc, char* argv[]) {
  QApplication application(argc, argv);
  application.setApplicationName(QStringLiteral("Autoviz Widget Gallery"));
  autoviz::ApplyAppTheme(application);

  const QString capture_path = CapturePath(application.arguments());
  if (!capture_path.isEmpty()) {
    autoviz::widget::MotionPolicy::SetPreference(
        autoviz::widget::MotionPreference::kReducedMotion);
  }

  const ThemeAppearance appearance =
      AppearanceFromArguments(application.arguments());
  application.setPalette(BuildWidgetPalette(autoviz::style::Tone::Shell,
                                            application.palette(), appearance));
  WidgetGallery gallery(appearance);
  gallery.show();
  const CaptureTarget capture_target =
      CaptureTargetFromArguments(application.arguments());
  if (!capture_path.isEmpty()) {
    if (capture_target == CaptureTarget::kGallery) {
      QTimer::singleShot(
          120, &application, [&application, &gallery, capture_path]() {
            QWidget* content = gallery.widget();
            const bool saved =
                content != nullptr && content->grab().save(capture_path);
            application.exit(saved ? 0 : 2);
          });
    } else {
      QTimer::singleShot(
          120, &application,
          [&application, &gallery, capture_path, capture_target]() {
            gallery.ShowCaptureTarget(capture_target);
            QTimer::singleShot(
                350, &application,
                [&application, &gallery, capture_path, capture_target]() {
                  QWidget* target = gallery.CaptureWidget(capture_target);
                  const bool saved =
                      target != nullptr && target->grab().save(capture_path);
                  application.exit(saved ? 0 : 2);
                });
          });
    }
  } else if (application.arguments().contains(QStringLiteral("--smoke-test"))) {
    QTimer::singleShot(0, &application, &QCoreApplication::quit);
  }
  return application.exec();
}
