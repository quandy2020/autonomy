/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <QApplication>
#include <QCoreApplication>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QImage>
#include <QLabel>
#include <QListView>
#include <QMainWindow>
#include <QPointer>
#include <QSignalSpy>
#include <QSortFilterProxyModel>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QStyle>
#include <QStyleOptionViewItem>
#include <QTest>
#include <QWheelEvent>
#include <memory>

#include "autoviz/common/widget/data/filter_bar.hpp"
#include "autoviz/common/widget/data/log_view.hpp"
#include "autoviz/common/widget/data/property_editor.hpp"
#include "autoviz/common/widget/docking/dock_manager.hpp"
#include "autoviz/common/widget/feedback/progress_bar.hpp"
#include "autoviz/common/widget/feedback/skeleton.hpp"
#include "autoviz/common/widget/input/button.hpp"
#include "autoviz/common/widget/input/number_field.hpp"
#include "autoviz/common/widget/input/search_box.hpp"
#include "autoviz/common/widget/input/segmented_control.hpp"
#include "autoviz/common/widget/input/toggle_switch.hpp"
#include "autoviz/common/widget/layout/flow_layout.hpp"
#include "autoviz/common/widget/layout/separator.hpp"
#include "autoviz/common/widget/layout/surface.hpp"
#include "autoviz/common/widget/navigation/command_palette.hpp"
#include "autoviz/common/widget/navigation/step_indicator.hpp"

namespace autoviz::widget {
namespace {

class WidgetComponentsTest final : public QObject {
  Q_OBJECT

 private slots:
  void FilterBarDebouncesAndOwnsFilterWidgets();
  void FlowLayoutWrapsItemsWithoutPerRowContainers();
  void PresentationPrimitivesExposeStateAndActivation();
  void IosControlsPreserveTouchGeometryAndVisualState();
  void NumberFieldRejectsUnfocusedWheelInput();
  void LogViewBoundsAndFiltersEntries();
  void PropertyEditorValidatesAndCreatesTypedEditors();
  void StepIndicatorSkipsDisabledStepsAndActivatesFromKeyboard();
  void CommandPaletteFiltersAndActivatesCommands();
  void DockManagerOwnsPanelsAndPersistsLayout();
};

void WidgetComponentsTest::FilterBarDebouncesAndOwnsFilterWidgets() {
  FilterBar filter;
  filter.SetDebounceIntervalMs(0);
  QSignalSpy query_spy(&filter, &FilterBar::QueryChanged);
  QSignalSpy search_spy(&filter, &FilterBar::SearchRequested);
  QSignalSpy cleared_spy(&filter, &FilterBar::Cleared);

  auto* chip = new QLabel(QStringLiteral("Images"));
  filter.AddFilterWidget(chip);
  filter.AddFilterWidget(chip);
  QCOMPARE(filter.filter_count(), 1);

  filter.SetQuery(QStringLiteral("  lidar  "));
  QCOMPARE(filter.query(), QStringLiteral("lidar"));
  QCOMPARE(query_spy.count(), 1);
  QCOMPARE(query_spy.last().at(0).toString(), QStringLiteral("lidar"));

  filter.Submit();
  QCOMPARE(search_spy.count(), 1);
  QCOMPARE(search_spy.last().at(0).toString(), QStringLiteral("lidar"));
  filter.Clear();
  QCOMPARE(filter.query(), QString());
  QCOMPARE(cleared_spy.count(), 1);

  QCOMPARE(filter.TakeFilterWidget(chip), chip);
  QCOMPARE(filter.filter_count(), 0);
  delete chip;
}

void WidgetComponentsTest::FlowLayoutWrapsItemsWithoutPerRowContainers() {
  QWidget host;
  host.resize(124, 80);
  auto* flow = new FlowLayout(&host, 0, 4, 4);
  for (int index = 0; index < 3; ++index) {
    auto* item = new QLabel(QString::number(index), &host);
    item->setFixedSize(56, 20);
    flow->addWidget(item);
  }
  flow->setGeometry(host.rect());

  QCOMPARE(flow->count(), 3);
  QVERIFY(flow->hasHeightForWidth());
  QVERIFY(flow->itemAt(2)->geometry().top() >
          flow->itemAt(0)->geometry().top());
  QVERIFY(flow->heightForWidth(host.width()) >= 44);

  QLayoutItem* removed = flow->takeAt(1);
  QVERIFY(removed != nullptr);
  QWidget* removed_widget = removed->widget();
  delete removed;
  delete removed_widget;
  QCOMPARE(flow->count(), 2);
}

void WidgetComponentsTest::PresentationPrimitivesExposeStateAndActivation() {
  IconButton icon_button(
      QApplication::style()->standardIcon(QStyle::SP_BrowserReload),
      QStringLiteral("Refresh"));
  QCOMPARE(icon_button.accessibleName(), QStringLiteral("Refresh"));
  QCOMPARE(icon_button.sizeHint().width(), icon_button.sizeHint().height());

  Separator separator;
  QCOMPARE(separator.orientation(), Qt::Horizontal);
  QCOMPARE(separator.sizeHint().height(), 1);
  separator.SetOrientation(Qt::Vertical);
  QCOMPARE(separator.orientation(), Qt::Vertical);
  QCOMPARE(separator.sizeHint().width(), 1);

  Skeleton skeleton;
  skeleton.SetCornerRadius(1000);
  skeleton.SetTone(style::Tone::Frost);
  QCOMPARE(skeleton.corner_radius(), 64);
  skeleton.SetActive(false);
  QVERIFY(!skeleton.is_active());
  skeleton.resize(skeleton.sizeHint());
  skeleton.show();
  QVERIFY(!skeleton.grab().isNull());
  skeleton.hide();

  Surface surface;
  surface.resize(160, 80);
  surface.SetElevation(Surface::Elevation::kOverlay);
  surface.SetInteractive(true);
  surface.SetSelected(true);
  QCOMPARE(surface.elevation(), Surface::Elevation::kOverlay);
  QVERIFY(surface.is_interactive());
  QVERIFY(surface.is_selected());
  QSignalSpy clicked_spy(&surface, &Surface::Clicked);
  surface.show();
  QTest::mouseClick(&surface, Qt::LeftButton, Qt::NoModifier,
                    surface.rect().center());
  QTest::keyClick(&surface, Qt::Key_Space);
  QCOMPARE(clicked_spy.count(), 2);
}

void WidgetComponentsTest::IosControlsPreserveTouchGeometryAndVisualState() {
  const auto render = [](QWidget* widget, const QSize& minimum_size) {
    widget->ensurePolished();
    widget->resize(widget->sizeHint().expandedTo(minimum_size));
    QImage image(widget->size(), QImage::Format_ARGB32_Premultiplied);
    image.fill(Qt::transparent);
    widget->render(&image);
    return image;
  };

  ToggleSwitch toggle;
  QCOMPARE(toggle.sizeHint(), QSize(51, 44));
  toggle.setEnabled(false);
  toggle.setChecked(false);
  const QImage disabled_off = render(&toggle, toggle.sizeHint());
  toggle.setChecked(true);
  const QImage disabled_on = render(&toggle, toggle.sizeHint());
  QVERIFY(disabled_off != disabled_on);

  SegmentedControl segments;
  segments.AddSegment(QStringLiteral("Live"));
  segments.AddSegment(QStringLiteral("Paused"));
  const QImage first_segment = render(&segments, QSize(180, 36));
  segments.SetCurrentIndex(1);
  const QImage second_segment = render(&segments, QSize(180, 36));
  QVERIFY(first_segment != second_segment);

  ProgressBar progress;
  progress.setRange(0, 100);
  progress.setValue(25);
  const QImage quarter = render(&progress, QSize(220, 28));
  progress.setValue(75);
  const QImage three_quarters = render(&progress, QSize(220, 28));
  QVERIFY(quarter != three_quarters);
}

void WidgetComponentsTest::NumberFieldRejectsUnfocusedWheelInput() {
  NumberField field;
  field.setRange(-10.0, 10.0);
  field.setSingleStep(1.0);
  field.setValue(4.0);
  field.SetTone(style::Tone::Frost);
  field.SetWheelRequiresFocus(true);
  QVERIFY(!field.hasFocus());

  QWheelEvent wheel(QPointF(10.0, 10.0), QPointF(10.0, 10.0), QPoint(),
                    QPoint(0, 120), Qt::NoButton, Qt::NoModifier,
                    Qt::NoScrollPhase, false);
  QApplication::sendEvent(&field, &wheel);
  QCOMPARE(field.value(), 4.0);
  QVERIFY(!wheel.isAccepted());
  QVERIFY(field.sizeHint().width() >= 112);
}

void WidgetComponentsTest::LogViewBoundsAndFiltersEntries() {
  LogView logs;
  logs.SetMaximumEntries(2);
  logs.Append({QDateTime::currentDateTime(), LogLevel::kInformation,
               QStringLiteral("localization"), QStringLiteral("started")});
  logs.Append({QDateTime::currentDateTime(), LogLevel::kWarning,
               QStringLiteral("mapping"), QStringLiteral("queue high")});
  logs.Append({QDateTime::currentDateTime(), LogLevel::kError,
               QStringLiteral("localization"), QStringLiteral("lost")});

  QCOMPARE(logs.log_model()->rowCount(), 2);
  const std::optional<LogEntry> first = logs.log_model()->EntryAt(0);
  QVERIFY(first.has_value());
  QCOMPARE(first->message, QStringLiteral("queue high"));

  logs.SetMinimumLevel(LogLevel::kError);
  QCOMPARE(logs.proxy_model()->rowCount(), 1);
  logs.SetMinimumLevel(LogLevel::kTrace);
  logs.SetFilterText(QStringLiteral("mapping"));
  QCOMPARE(logs.proxy_model()->rowCount(), 1);

  QSignalSpy scroll_spy(&logs, &LogView::AutoScrollChanged);
  logs.SetAutoScroll(false);
  QCOMPARE(scroll_spy.count(), 1);
  QVERIFY(!logs.auto_scroll());
  logs.Clear();
  QCOMPARE(logs.log_model()->rowCount(), 0);
}

void WidgetComponentsTest::PropertyEditorValidatesAndCreatesTypedEditors() {
  PropertySpec gain;
  gain.id = QStringLiteral("gain");
  gain.name = QStringLiteral("Gain");
  gain.description = QStringLiteral("Localization gain");
  gain.type = PropertyType::kDouble;
  gain.value = 1.0;
  gain.minimum = 0.0;
  gain.maximum = 2.0;
  gain.step = 0.1;

  PropertyEditor editor;
  editor.SetProperties({gain});
  QSignalSpy value_spy(&editor, &PropertyEditor::PropertyValueChanged);
  QSignalSpy validation_spy(&editor, &PropertyEditor::PropertyValidationFailed);

  QVERIFY(editor.property_model()->SetValue(QStringLiteral("gain"), 1.5));
  QCOMPARE(editor.property_model()->Value(QStringLiteral("gain")).toDouble(),
           1.5);
  QCOMPARE(value_spy.count(), 1);
  QVERIFY(!editor.property_model()->SetValue(QStringLiteral("gain"), 3.0));
  QCOMPARE(validation_spy.count(), 1);

  const QModelIndex value_index = editor.property_model()->index(0, 1);
  QVERIFY(value_index.isValid());
  QWidget editor_host;
  QStyleOptionViewItem option;
  option.rect = QRect(0, 0, 160, 32);
  std::unique_ptr<QWidget> typed_editor(
      editor.itemDelegate()->createEditor(&editor_host, option, value_index));
  QVERIFY(qobject_cast<QDoubleSpinBox*>(typed_editor.get()) != nullptr);
  editor.itemDelegate()->setEditorData(typed_editor.get(), value_index);
  QCOMPARE(qobject_cast<QDoubleSpinBox*>(typed_editor.get())->value(), 1.5);
}

void WidgetComponentsTest::
    StepIndicatorSkipsDisabledStepsAndActivatesFromKeyboard() {
  StepIndicator steps;
  steps.SetSteps({{QStringLiteral("Connect"), true},
                  {QStringLiteral("Unavailable"), false},
                  {QStringLiteral("Map"), true}});
  steps.SetInteractive(true);
  steps.resize(steps.sizeHint());
  steps.show();
  QCOMPARE(steps.current_index(), 0);

  QSignalSpy changed_spy(&steps, &StepIndicator::CurrentIndexChanged);
  QSignalSpy activated_spy(&steps, &StepIndicator::StepActivated);
  QTest::keyClick(&steps, Qt::Key_Right);
  QCOMPARE(steps.current_index(), 2);
  QCOMPARE(changed_spy.count(), 1);
  QTest::keyClick(&steps, Qt::Key_Space);
  QCOMPARE(activated_spy.count(), 1);
  QCOMPARE(activated_spy.last().at(0).toInt(), 2);
  QVERIFY(steps.accessibleDescription().contains(QStringLiteral("Map")));
}

void WidgetComponentsTest::CommandPaletteFiltersAndActivatesCommands() {
  constexpr int kIdRole = Qt::UserRole + 1;
  constexpr int kDataRole = Qt::UserRole + 2;
  QStandardItemModel model;
  auto* open = new QStandardItem(QStringLiteral("Open map"));
  open->setData(QStringLiteral("open-map"), kIdRole);
  open->setData(7, kDataRole);
  model.appendRow(open);
  auto* save = new QStandardItem(QStringLiteral("Save layout"));
  save->setData(QStringLiteral("save-layout"), kIdRole);
  save->setData(9, kDataRole);
  model.appendRow(save);

  CommandPalette palette;
  palette.SetRoles(kIdRole, kDataRole);
  palette.SetModel(&model);
  palette.SetMatchMode(CommandPalette::MatchMode::kFuzzy);
  QSignalSpy command_spy(&palette, &CommandPalette::CommandActivated);
  QSignalSpy index_spy(&palette, &CommandPalette::SourceIndexActivated);
  palette.ShowPalette();
  QCoreApplication::processEvents();

  auto* search = palette.findChild<SearchBox*>();
  auto* list = palette.findChild<QListView*>();
  QVERIFY(search != nullptr);
  QVERIFY(list != nullptr);
  search->setText(QStringLiteral("sl"));
  QCOMPARE(list->model()->rowCount(), 1);
  QTest::keyClick(search, Qt::Key_Return);

  QCOMPARE(command_spy.count(), 1);
  QCOMPARE(command_spy.first().at(0).toString(), QStringLiteral("save-layout"));
  QCOMPARE(command_spy.first().at(1).toInt(), 9);
  QCOMPARE(index_spy.count(), 1);
  QVERIFY(!palette.isVisible());
}

void WidgetComponentsTest::DockManagerOwnsPanelsAndPersistsLayout() {
  QMainWindow host;
  host.setObjectName(QStringLiteral("WidgetComponentsDockHost"));
  DockManager docks(&host);
  QPointer<QWidget> content = new QLabel(QStringLiteral("Inspector"));
  DockPanelOptions options;
  options.closable = false;
  options.movable = true;
  options.floatable = false;

  QDockWidget* panel =
      docks.AddPanel(QStringLiteral("inspector"), QStringLiteral("Inspector"),
                     content, DockArea::kRight, options);
  QVERIFY(panel != nullptr);
  QCOMPARE(docks.Panel(QStringLiteral("inspector")), panel);
  QVERIFY(!(panel->features() & QDockWidget::DockWidgetClosable));
  QVERIFY(panel->features() & QDockWidget::DockWidgetMovable);
  QVERIFY(!(panel->features() & QDockWidget::DockWidgetFloatable));
  auto duplicate_content = std::make_unique<QLabel>();
  QVERIFY(docks.AddPanel(QStringLiteral("inspector"), QStringLiteral("Dup"),
                         duplicate_content.get(), DockArea::kLeft) == nullptr);

  host.show();
  const QByteArray state = docks.SaveLayout(42);
  QVERIFY(!state.isEmpty());
  QVERIFY(docks.RestoreLayout(state, 42));
  docks.SetPanelVisible(QStringLiteral("inspector"), false);
  QVERIFY(panel->isHidden());

  QVERIFY(docks.RemovePanel(QStringLiteral("inspector")));
  QVERIFY(docks.Panel(QStringLiteral("inspector")) == nullptr);
  QVERIFY(!docks.RemovePanel(QStringLiteral("inspector")));
  QCoreApplication::sendPostedEvents(nullptr, QEvent::DeferredDelete);
  QVERIFY(content.isNull());
}

}  // namespace
}  // namespace autoviz::widget

QTEST_MAIN(autoviz::widget::WidgetComponentsTest)
#include "widget_components_test.moc"
