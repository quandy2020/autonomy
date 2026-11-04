/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <QCoreApplication>
#include <QEvent>
#include <QPoint>
#include <QTest>

#include "autoviz/common/widget/navigation/breadcrumb.hpp"
#include "autoviz/common/widget/navigation/pips_pager.hpp"
#include "autoviz/common/widget/navigation/step_indicator.hpp"

namespace autoviz::widget {
namespace {

class WidgetRtlNavigationTest final : public QObject {
  Q_OBJECT

 private slots:
  void BreadcrumbMirrorsChevronAndHorizontalKeys();
  void StepIndicatorMirrorsKeysAndHitTesting();
  void PipsPagerMirrorsKeysAndHitTesting();
};

void WidgetRtlNavigationTest::BreadcrumbMirrorsChevronAndHorizontalKeys() {
  Breadcrumb breadcrumb;
  breadcrumb.SetItems({
      {QStringLiteral("Home"), QStringLiteral("home"), true},
      {QStringLiteral("Sensors"), QStringLiteral("sensors"), true},
      {QStringLiteral("Map"), QStringLiteral("map"), true},
  });
  breadcrumb.show();
  breadcrumb.setFocus();
  QTRY_VERIFY(breadcrumb.hasFocus());

  QList<QWidget*> separators = breadcrumb.findChildren<QWidget*>(
      QStringLiteral("AutovizBreadcrumbSeparator"), Qt::FindDirectChildrenOnly);
  QCOMPARE(separators.size(), 2);
  QCOMPARE(separators.constFirst()->property("autovizGlyph").toString(),
           QStringLiteral("›"));

  breadcrumb.SetCurrentIndex(1);
  QTest::keyClick(&breadcrumb, Qt::Key_Left);
  QCOMPARE(breadcrumb.current_index(), 0);
  QTest::keyClick(&breadcrumb, Qt::Key_Right);
  QCOMPARE(breadcrumb.current_index(), 1);

  breadcrumb.setLayoutDirection(Qt::RightToLeft);
  QCoreApplication::sendPostedEvents(nullptr, QEvent::DeferredDelete);
  separators = breadcrumb.findChildren<QWidget*>(
      QStringLiteral("AutovizBreadcrumbSeparator"), Qt::FindDirectChildrenOnly);
  QCOMPARE(separators.size(), 2);
  QCOMPARE(separators.constFirst()->property("autovizGlyph").toString(),
           QStringLiteral("‹"));

  QTest::keyClick(&breadcrumb, Qt::Key_Left);
  QCOMPARE(breadcrumb.current_index(), 2);
  QTest::keyClick(&breadcrumb, Qt::Key_Right);
  QCOMPARE(breadcrumb.current_index(), 1);
}

void WidgetRtlNavigationTest::StepIndicatorMirrorsKeysAndHitTesting() {
  StepIndicator steps;
  steps.SetSteps({
      {QStringLiteral("Connect"), true},
      {QStringLiteral("Calibrate"), true},
      {QStringLiteral("Map"), true},
  });
  steps.SetInteractive(true);
  steps.resize(300, steps.sizeHint().height());
  steps.show();
  steps.setFocus();
  QTRY_VERIFY(steps.hasFocus());

  steps.SetCurrentIndex(1);
  QTest::keyClick(&steps, Qt::Key_Left);
  QCOMPARE(steps.current_index(), 0);
  QTest::keyClick(&steps, Qt::Key_Right);
  QCOMPARE(steps.current_index(), 1);

  steps.setLayoutDirection(Qt::RightToLeft);
  steps.SetCurrentIndex(1);
  QTest::keyClick(&steps, Qt::Key_Left);
  QCOMPARE(steps.current_index(), 2);
  QTest::keyClick(&steps, Qt::Key_Right);
  QCOMPARE(steps.current_index(), 1);

  const int center_y = steps.height() / 2;
  QTest::mouseClick(&steps, Qt::LeftButton, Qt::NoModifier,
                    QPoint(4, center_y));
  QCOMPARE(steps.current_index(), 2);
  QTest::mouseClick(&steps, Qt::LeftButton, Qt::NoModifier,
                    QPoint(steps.width() - 4, center_y));
  QCOMPARE(steps.current_index(), 0);
}

void WidgetRtlNavigationTest::PipsPagerMirrorsKeysAndHitTesting() {
  PipsPager pager;
  pager.SetPageCount(3);
  pager.resize(pager.sizeHint());
  pager.show();
  pager.setFocus();
  QTRY_VERIFY(pager.hasFocus());

  pager.SetCurrentPage(1);
  QTest::keyClick(&pager, Qt::Key_Left);
  QCOMPARE(pager.current_page(), 0);
  QTest::keyClick(&pager, Qt::Key_Right);
  QCOMPARE(pager.current_page(), 1);

  pager.setLayoutDirection(Qt::RightToLeft);
  pager.SetCurrentPage(1);
  QTest::keyClick(&pager, Qt::Key_Left);
  QCOMPARE(pager.current_page(), 2);
  QTest::keyClick(&pager, Qt::Key_Right);
  QCOMPARE(pager.current_page(), 1);

  const int center_y = pager.height() / 2;
  QTest::mouseClick(&pager, Qt::LeftButton, Qt::NoModifier,
                    QPoint(10, center_y));
  QCOMPARE(pager.current_page(), 2);
  QTest::mouseClick(&pager, Qt::LeftButton, Qt::NoModifier,
                    QPoint(pager.width() - 10, center_y));
  QCOMPARE(pager.current_page(), 0);
}

}  // namespace
}  // namespace autoviz::widget

QTEST_MAIN(autoviz::widget::WidgetRtlNavigationTest)

#include "widget_rtl_navigation_test.moc"
