/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <QAbstractItemModelTester>
#include <QCoreApplication>
#include <QFrame>
#include <QHeaderView>
#include <QImage>
#include <QPixmap>
#include <QProxyStyle>
#include <QSignalSpy>
#include <QTableView>
#include <QTest>
#include <QVariant>
#include <limits>
#include <thread>

#include "autoviz/common/widget/data/data_grid.hpp"
#include "autoviz/common/widget/data/empty_state.hpp"
#include "autoviz/common/widget/data/log_view.hpp"
#include "autoviz/common/widget/data/pagination.hpp"
#include "autoviz/common/widget/data/property_editor.hpp"
#include "autoviz/common/widget/data/timeline_scrubber.hpp"

namespace autoviz::widget {
namespace {

class SolidStandardIconStyle final : public QProxyStyle {
 public:
  explicit SolidStandardIconStyle(const QColor& color) : color_(color) {}

  QIcon standardIcon(StandardPixmap standard_pixmap,
                     const QStyleOption* option = nullptr,
                     const QWidget* widget = nullptr) const override {
    if (standard_pixmap != QStyle::SP_FileDialogContentsView) {
      return QProxyStyle::standardIcon(standard_pixmap, option, widget);
    }
    QPixmap pixmap(16, 16);
    pixmap.fill(color_);
    return QIcon(pixmap);
  }

 private:
  QColor color_;
};

LogEntry MakeLogEntry(int sequence) {
  LogEntry entry;
  entry.timestamp = QDateTime::fromMSecsSinceEpoch(sequence);
  entry.level = LogLevel::kInformation;
  entry.source = QStringLiteral("source");
  entry.message = QString::number(sequence);
  return entry;
}

class WidgetDataTest final : public QObject {
  Q_OBJECT

 private slots:
  void PaginationSupportsQint64Boundaries();
  void TimelineSupportsTheFullQint64Range();
  void PropertyModelValidatesAndKeeps64BitValues();
  void PropertyModelRejectsNonFiniteValues();
  void EmptyStateUsesSemanticToneAndAccessibility();
  void EmptyStateRefreshesOnlyTheDefaultIconAfterStyleChange();
  void DataViewsUseCleanNativePresentation();
  void LogModelBoundsAndQueuesCrossThreadMutations();
};

void WidgetDataTest::PaginationSupportsQint64Boundaries() {
  Pagination pagination;
  QSignalSpy request_spy(&pagination, &Pagination::PageRequested);

  const qint64 maximum = std::numeric_limits<qint64>::max();
  pagination.SetPageSize(1);
  pagination.SetTotalItems(maximum);
  QCOMPARE(pagination.page_count(), maximum);

  pagination.GoToLastPage();
  QCOMPARE(pagination.current_page(), maximum - 1);
  QCOMPARE(pagination.offset(), maximum - 1);

  pagination.GoToNextPage();
  QCOMPARE(pagination.current_page(), maximum - 1);

  pagination.SetTotalItems(10);
  QCOMPARE(pagination.current_page(), qint64{9});
  pagination.SetPageSize(4);
  QCOMPARE(pagination.current_page(), qint64{2});
  QCOMPARE(pagination.offset(), qint64{8});

  pagination.SetTotalItems(-1);
  QCOMPARE(pagination.total_items(), qint64{0});
  QCOMPARE(pagination.page_count(), qint64{0});
  QCOMPARE(pagination.current_page(), qint64{0});
  QVERIFY(!request_spy.isEmpty());
}

void WidgetDataTest::TimelineSupportsTheFullQint64Range() {
  TimelineScrubber scrubber;
  scrubber.resize(400, 44);
  scrubber.show();

  const qint64 minimum = std::numeric_limits<qint64>::min();
  const qint64 maximum = std::numeric_limits<qint64>::max();
  scrubber.SetRange(maximum, minimum);
  QCOMPARE(scrubber.minimum(), minimum);
  QCOMPARE(scrubber.maximum(), maximum);

  scrubber.SetValue(maximum);
  QCOMPARE(scrubber.value(), maximum);
  const QString full_span =
      QString::number(std::numeric_limits<quint64>::max());
  QCOMPARE(scrubber.accessibleDescription(),
           QStringLiteral("Position %1 of %1").arg(full_span));

  QTest::keyClick(&scrubber, Qt::Key_Home);
  QCOMPARE(scrubber.value(), minimum);
  QTest::keyClick(&scrubber, Qt::Key_Right);
  QVERIFY(scrubber.value() > minimum);
  QTest::keyClick(&scrubber, Qt::Key_End);
  QCOMPARE(scrubber.value(), maximum);

  scrubber.SetMarkers({
      {.position = 12,
       .color = QColor(),
       .description = QStringLiteral("later")},
      {.position = -7,
       .color = QColor(),
       .description = QStringLiteral("earlier")},
  });
  QCOMPARE(scrubber.markers().at(0).position, qint64{-7});
  QCOMPARE(scrubber.markers().at(1).position, qint64{12});
}

void WidgetDataTest::PropertyModelValidatesAndKeeps64BitValues() {
  PropertyModel model;
  QAbstractItemModelTester model_tester(
      &model, QAbstractItemModelTester::FailureReportingMode::QtTest);
  QSignalSpy validation_spy(&model, &PropertyModel::ValidationFailed);
  QSignalSpy value_spy(&model, &PropertyModel::ValueChanged);

  PropertySpec integer;
  integer.id = QStringLiteral("integer");
  integer.name = QStringLiteral("Integer");
  integer.type = PropertyType::kInteger;
  integer.value = QVariant::fromValue<qlonglong>(17);
  integer.minimum =
      QVariant::fromValue<qlonglong>(std::numeric_limits<qlonglong>::min());
  integer.maximum =
      QVariant::fromValue<qlonglong>(std::numeric_limits<qlonglong>::max());

  PropertySpec duplicate = integer;
  duplicate.name = QStringLiteral("Duplicate");

  PropertySpec invalid;
  invalid.id = QStringLiteral("bounded");
  invalid.name = QStringLiteral("Bounded");
  invalid.type = PropertyType::kInteger;
  invalid.value = QStringLiteral("not-a-number");
  invalid.minimum = QVariant::fromValue<qlonglong>(-5);
  invalid.maximum = QVariant::fromValue<qlonglong>(5);

  PropertySpec child;
  child.id = QStringLiteral("child");
  child.name = QStringLiteral("Child");
  child.type = PropertyType::kString;
  child.value = QStringLiteral("value");
  PropertySpec group;
  group.id = QStringLiteral("group");
  group.name = QStringLiteral("Group");
  group.type = PropertyType::kGroup;
  group.children.push_back(child);

  model.SetProperties({integer, duplicate, invalid, group});
  QCOMPARE(validation_spy.count(), 2);
  QCOMPARE(model.Value(QStringLiteral("integer")).toLongLong(), qlonglong{17});
  QCOMPARE(model.Value(QStringLiteral("bounded")).toLongLong(), qlonglong{-5});

  const QModelIndex group_index = model.index(3, 0);
  QCOMPARE(model.rowCount(group_index), 1);
  QCOMPARE(model.parent(model.index(0, 0, group_index)), group_index);

  const qlonglong maximum = std::numeric_limits<qlonglong>::max();
  QVERIFY(model.SetValue(QStringLiteral("integer"),
                         QVariant::fromValue<qlonglong>(maximum)));
  QCOMPARE(model.Value(QStringLiteral("integer")).toLongLong(), maximum);
  QCOMPARE(value_spy.count(), 1);

  QVERIFY(!model.SetValue(QStringLiteral("bounded"), 6));
  QCOMPARE(validation_spy.count(), 3);
  QCOMPARE(model.Value(QStringLiteral("bounded")).toLongLong(), qlonglong{-5});
}

void WidgetDataTest::PropertyModelRejectsNonFiniteValues() {
  PropertyModel model;
  QSignalSpy validation_spy(&model, &PropertyModel::ValidationFailed);
  QSignalSpy value_spy(&model, &PropertyModel::ValueChanged);

  PropertySpec floating;
  floating.id = QStringLiteral("floating");
  floating.name = QStringLiteral("Floating point");
  floating.type = PropertyType::kDouble;
  floating.value = 1.5;
  floating.minimum = -10.0;
  floating.maximum = 10.0;
  model.SetProperties({floating});
  QCOMPARE(validation_spy.count(), 0);

  const double invalid_values[] = {
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity(),
  };
  for (const double invalid_value : invalid_values) {
    QVERIFY(!model.SetValue(QStringLiteral("floating"), invalid_value));
    QCOMPARE(model.Value(QStringLiteral("floating")).toDouble(), 1.5);
  }
  QCOMPARE(validation_spy.count(), 3);
  QCOMPARE(value_spy.count(), 0);
}

void WidgetDataTest::EmptyStateUsesSemanticToneAndAccessibility() {
  EmptyState empty;
  QCOMPARE(empty.property("autovizRole").toString(),
           QStringLiteral("emptyState"));
  QCOMPARE(empty.tone(), style::Tone::Shell);

  empty.SetTitle(QStringLiteral("No data"));
  empty.SetDescription(QStringLiteral("Connect a source"));
  empty.SetActionText(QStringLiteral("Connect"));
  QCOMPARE(empty.accessibleName(), QStringLiteral("No data. Connect a source"));
  QCOMPARE(empty.accessibleDescription(), empty.accessibleName());

  empty.SetDescription(QString());
  QCOMPARE(empty.accessibleName(), QStringLiteral("No data"));
  QCOMPARE(empty.accessibleDescription(), empty.accessibleName());
  empty.SetTitle(QString());
  empty.SetDescription(QStringLiteral("Connect a source"));
  QCOMPARE(empty.accessibleName(), QStringLiteral("Connect a source"));
  QCOMPARE(empty.accessibleDescription(), empty.accessibleName());
  empty.SetTitle(QStringLiteral("No data"));

  empty.SetTone(style::Tone::Frost);
  QCOMPARE(empty.tone(), style::Tone::Frost);
  QCOMPARE(empty.property("autovizTone").toString(), QStringLiteral("frost"));

  empty.resize(360, 240);
  empty.setEnabled(false);
  empty.show();
  QImage image(empty.size(), QImage::Format_ARGB32_Premultiplied);
  image.fill(Qt::transparent);
  QImage blank(empty.size(), QImage::Format_ARGB32_Premultiplied);
  blank.fill(Qt::transparent);
  empty.render(&image);
  QVERIFY(!image.isNull());
  QVERIFY(image != blank);
}

void WidgetDataTest::EmptyStateRefreshesOnlyTheDefaultIconAfterStyleChange() {
  SolidStandardIconStyle first_style(QColor(12, 90, 210));
  SolidStandardIconStyle second_style(QColor(220, 52, 68));
  EmptyState empty;

  empty.setStyle(&first_style);
  const QImage first_icon = empty.icon().pixmap(QSize(16, 16)).toImage();
  empty.setStyle(&second_style);
  const QImage second_icon = empty.icon().pixmap(QSize(16, 16)).toImage();
  QVERIFY(first_icon != second_icon);

  QPixmap custom_pixmap(16, 16);
  custom_pixmap.fill(QColor(20, 180, 90));
  const QIcon custom_icon(custom_pixmap);
  empty.SetIcon(custom_icon);
  const qint64 custom_cache_key = empty.icon().cacheKey();
  empty.setStyle(&first_style);
  QCOMPARE(empty.icon().cacheKey(), custom_cache_key);
}

void WidgetDataTest::DataViewsUseCleanNativePresentation() {
  DataGrid grid;
  auto* grid_view =
      grid.findChild<QTableView*>(QStringLiteral("AutovizDataGridView"));
  QVERIFY(grid_view != nullptr);
  QVERIFY(!grid_view->showGrid());
  QCOMPARE(grid_view->frameShape(), QFrame::NoFrame);
  QVERIFY(!grid_view->horizontalHeader()->highlightSections());

  PropertyEditor property_editor;
  QCOMPARE(property_editor.frameShape(), QFrame::NoFrame);
  QVERIFY(property_editor.allColumnsShowFocus());
  QVERIFY(!property_editor.header()->highlightSections());

  LogView log_view;
  auto* log_table = log_view.findChild<QTableView*>();
  QVERIFY(log_table != nullptr);
  QVERIFY(!log_table->showGrid());
  QCOMPARE(log_table->frameShape(), QFrame::NoFrame);
  QVERIFY(!log_table->horizontalHeader()->highlightSections());
}

void WidgetDataTest::LogModelBoundsAndQueuesCrossThreadMutations() {
  LogModel model;
  model.SetMaximumEntries(3);
  model.AppendBatch({MakeLogEntry(0), MakeLogEntry(1), MakeLogEntry(2),
                     MakeLogEntry(3), MakeLogEntry(4)});
  QCOMPARE(model.rowCount(), 3);
  QVERIFY(model.EntryAt(0).has_value());
  QCOMPARE(model.EntryAt(0)->message, QStringLiteral("2"));

  std::thread producer([&model]() {
    model.Append(MakeLogEntry(5));
    model.SetMaximumEntries(2);
  });
  producer.join();

  QTRY_COMPARE(model.maximum_entries(), 2);
  QTRY_COMPARE(model.rowCount(), 2);
  QCOMPARE(model.EntryAt(0)->message, QStringLiteral("4"));
  QCOMPARE(model.EntryAt(1)->message, QStringLiteral("5"));

  auto* short_lived = new LogModel;
  std::thread late_producer(
      [short_lived]() { short_lived->Append(MakeLogEntry(9)); });
  late_producer.join();
  delete short_lived;
  QCoreApplication::processEvents();
}

}  // namespace
}  // namespace autoviz::widget

QTEST_MAIN(autoviz::widget::WidgetDataTest)

#include "widget_data_test.moc"
