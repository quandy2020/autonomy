/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <QAbstractItemModelTester>
#include <QSignalSpy>
#include <QSortFilterProxyModel>
#include <QTableView>
#include <QTest>
#include <utility>

#include "autoviz/common/widget/data/data_grid.hpp"
#include "autoviz/ui/table/table_model.hpp"

namespace autoviz::table_panel {
namespace {

class WidgetTableTest final : public QObject {
  Q_OBJECT

 private slots:
  void ModelReplacesSnapshotsAtomically();
  void DataGridFiltersSortsAndBatchesUpdates();
};

void WidgetTableTest::ModelReplacesSnapshotsAtomically() {
  TableModel model;
  QAbstractItemModelTester model_tester(
      &model, QAbstractItemModelTester::FailureReportingMode::QtTest);
  QSignalSpy reset_spy(&model, &QAbstractItemModel::modelReset);

  TableSnapshot snapshot;
  snapshot.headers = {QStringLiteral("name"), QStringLiteral("value")};
  snapshot.cells = {QStringLiteral("alpha"), QStringLiteral("1"),
                    QStringLiteral("beta"), QStringLiteral("2")};
  snapshot.row_count = 2;
  snapshot.capped = true;
  QVERIFY(model.Replace(std::move(snapshot)));
  QCOMPARE(reset_spy.count(), 1);
  QCOMPARE(model.rowCount(), 2);
  QCOMPARE(model.columnCount(), 2);
  QCOMPARE(model.index(1, 0).data().toString(), QStringLiteral("beta"));
  QCOMPARE(model.headerData(1, Qt::Horizontal).toString(),
           QStringLiteral("value"));
  QCOMPARE(model.headerData(1, Qt::Vertical).toInt(), 2);
  QVERIFY(!(model.flags(model.index(0, 0)) & Qt::ItemIsEditable));

  TableSnapshot invalid;
  invalid.headers = {QStringLiteral("only")};
  invalid.cells = {QStringLiteral("one")};
  invalid.row_count = 2;
  QVERIFY(!model.Replace(std::move(invalid)));
  QCOMPARE(reset_spy.count(), 1);
  QCOMPARE(model.rowCount(), 2);

  model.Clear();
  QCOMPARE(reset_spy.count(), 2);
  QCOMPARE(model.rowCount(), 0);
  QCOMPARE(model.columnCount(), 0);
}

void WidgetTableTest::DataGridFiltersSortsAndBatchesUpdates() {
  TableModel model;
  TableSnapshot snapshot;
  snapshot.headers = {QStringLiteral("name"), QStringLiteral("state")};
  snapshot.cells = {
      QStringLiteral("alpha"), QStringLiteral("ready"),
      QStringLiteral("beta"),  QStringLiteral("waiting"),
      QStringLiteral("gamma"), QStringLiteral("ready"),
  };
  snapshot.row_count = 3;
  QVERIFY(model.Replace(std::move(snapshot)));

  widget::DataGrid grid;
  grid.SetSourceModel(&model);
  grid.resize(480, 240);
  grid.show();
  QCOMPARE(grid.proxy_model()->rowCount(), 3);

  grid.SetFilterText(QStringLiteral("waiting"));
  QTRY_COMPARE(grid.proxy_model()->rowCount(), 1);
  QCOMPARE(grid.proxy_model()->index(0, 0).data().toString(),
           QStringLiteral("beta"));
  grid.SetFilterText(QString());
  QTRY_COMPARE(grid.proxy_model()->rowCount(), 3);

  grid.BeginBatchUpdate();
  grid.BeginBatchUpdate();
  QVERIFY(!grid.proxy_model()->dynamicSortFilter());
  QVERIFY(!grid.view()->updatesEnabled());
  grid.EndBatchUpdate();
  QVERIFY(!grid.proxy_model()->dynamicSortFilter());
  grid.EndBatchUpdate();
  QVERIFY(grid.proxy_model()->dynamicSortFilter());
  QVERIFY(grid.view()->updatesEnabled());

  grid.view()->sortByColumn(0, Qt::DescendingOrder);
  QTRY_COMPARE(grid.proxy_model()->index(0, 0).data().toString(),
               QStringLiteral("gamma"));
  QCOMPARE(grid.view()->indexWidget(grid.proxy_model()->index(0, 0)), nullptr);
}

}  // namespace
}  // namespace autoviz::table_panel

QTEST_MAIN(autoviz::table_panel::WidgetTableTest)
#include "widget_table_test.moc"
