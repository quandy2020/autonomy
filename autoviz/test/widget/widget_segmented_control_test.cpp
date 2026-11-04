/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <QAbstractButton>
#include <QCoreApplication>
#include <QSignalSpy>
#include <QTest>

#include "autoviz/common/widget/input/segmented_control.hpp"

namespace autoviz::widget {
namespace {

QList<QAbstractButton*> DirectButtons(SegmentedControl* control) {
  return control->findChildren<QAbstractButton*>(QString(),
                                                 Qt::FindDirectChildrenOnly);
}

void ExpectSingleTabStop(const QList<QAbstractButton*>& buttons,
                         int tab_index) {
  for (int index = 0; index < buttons.size(); ++index) {
    QCOMPARE(buttons[index]->focusPolicy(),
             index == tab_index ? Qt::StrongFocus : Qt::ClickFocus);
  }
}

class WidgetSegmentedControlTest final : public QObject {
  Q_OBJECT

 private slots:
  void RovingTabStopFollowsEnabledSelection();
  void KeyboardNavigationMirrorsAndSkipsDisabledSegments();
  void ActivationKeyRetainsClickedSignal();
  void SegmentEdgesFollowVisualOrder();
};

void WidgetSegmentedControlTest::RovingTabStopFollowsEnabledSelection() {
  SegmentedControl control;
  control.AddSegment(QStringLiteral("Overview"));
  control.AddSegment(QStringLiteral("Diagnostics"));
  control.AddSegment(QStringLiteral("Map"));
  control.show();
  QCoreApplication::processEvents();
  const QList<QAbstractButton*> buttons = DirectButtons(&control);
  QCOMPARE(buttons.size(), 3);

  ExpectSingleTabStop(buttons, 0);
  control.SetCurrentIndex(2);
  ExpectSingleTabStop(buttons, 2);
  buttons[2]->setFocus();
  QTRY_VERIFY(buttons[2]->hasFocus());

  control.SetSegmentEnabled(2, false);
  ExpectSingleTabStop(buttons, 0);
  QTRY_VERIFY(buttons[0]->hasFocus());
  control.SetSegmentEnabled(0, false);
  ExpectSingleTabStop(buttons, 1);

  control.SetSegmentEnabled(2, true);
  ExpectSingleTabStop(buttons, 2);
  control.SetCurrentIndex(-1);
  ExpectSingleTabStop(buttons, 1);
}

void WidgetSegmentedControlTest::
    KeyboardNavigationMirrorsAndSkipsDisabledSegments() {
  SegmentedControl control;
  control.AddSegment(QStringLiteral("Overview"), 0);
  control.AddSegment(QStringLiteral("Disabled"), 1);
  control.AddSegment(QStringLiteral("Map"), 2);
  control.AddSegment(QStringLiteral("Logs"), 3);
  control.SetSegmentEnabled(1, false);
  control.resize(420, control.sizeHint().height());
  control.show();
  QCoreApplication::processEvents();

  const QList<QAbstractButton*> buttons = DirectButtons(&control);
  QCOMPARE(buttons.size(), 4);
  QSignalSpy index_spy(&control, &SegmentedControl::CurrentIndexChanged);
  QSignalSpy data_spy(&control, &SegmentedControl::CurrentDataChanged);

  buttons[0]->setFocus();
  QTRY_VERIFY(buttons[0]->hasFocus());
  QTest::keyClick(buttons[0], Qt::Key_Right);
  QCOMPARE(control.current_index(), 2);
  QCOMPARE(control.current_data().toInt(), 2);
  QTRY_VERIFY(buttons[2]->hasFocus());
  QCOMPARE(index_spy.count(), 1);
  QCOMPARE(data_spy.count(), 1);

  QTest::keyClick(buttons[2], Qt::Key_Home);
  QCOMPARE(control.current_index(), 0);
  QTest::keyClick(buttons[0], Qt::Key_End);
  QCOMPARE(control.current_index(), 3);

  control.setLayoutDirection(Qt::RightToLeft);
  control.SetCurrentIndex(0);
  buttons[0]->setFocus();
  QTRY_VERIFY(buttons[0]->hasFocus());
  index_spy.clear();
  data_spy.clear();

  QTest::keyClick(buttons[0], Qt::Key_Left);
  QCOMPARE(control.current_index(), 2);
  QCOMPARE(control.current_data().toInt(), 2);
  QTRY_VERIFY(buttons[2]->hasFocus());
  QCOMPARE(index_spy.count(), 1);
  QCOMPARE(data_spy.count(), 1);

  QTest::keyClick(buttons[2], Qt::Key_Right);
  QCOMPARE(control.current_index(), 0);
  QTRY_VERIFY(buttons[0]->hasFocus());
}

void WidgetSegmentedControlTest::ActivationKeyRetainsClickedSignal() {
  SegmentedControl control;
  control.AddSegment(QStringLiteral("Overview"), 0);
  control.AddSegment(QStringLiteral("Map"), 1);
  control.show();
  QCoreApplication::processEvents();

  const QList<QAbstractButton*> buttons = DirectButtons(&control);
  QSignalSpy clicked_spy(&control, &SegmentedControl::SegmentClicked);
  QSignalSpy index_spy(&control, &SegmentedControl::CurrentIndexChanged);
  buttons[1]->setFocus();
  QTRY_VERIFY(buttons[1]->hasFocus());

  QTest::keyClick(buttons[1], Qt::Key_Space);
  QCOMPARE(control.current_index(), 1);
  QCOMPARE(clicked_spy.count(), 1);
  QCOMPARE(index_spy.count(), 1);
}

void WidgetSegmentedControlTest::SegmentEdgesFollowVisualOrder() {
  SegmentedControl control;
  control.AddSegment(QStringLiteral("Overview"));
  control.AddSegment(QStringLiteral("Map"));
  control.AddSegment(QStringLiteral("Logs"));
  control.resize(360, control.sizeHint().height());
  control.show();
  QCoreApplication::processEvents();

  const QList<QAbstractButton*> buttons = DirectButtons(&control);
  QCOMPARE(buttons.size(), 3);
  QVERIFY(buttons[0]->geometry().center().x() <
          buttons[2]->geometry().center().x());
  QCOMPARE(buttons[0]->property("autovizLeftEdge").toBool(), true);
  QCOMPARE(buttons[0]->property("autovizRightEdge").toBool(), false);
  QCOMPARE(buttons[2]->property("autovizLeftEdge").toBool(), false);
  QCOMPARE(buttons[2]->property("autovizRightEdge").toBool(), true);

  control.setLayoutDirection(Qt::RightToLeft);
  QCoreApplication::processEvents();
  QTRY_VERIFY(buttons[0]->geometry().center().x() >
              buttons[2]->geometry().center().x());
  QCOMPARE(buttons[0]->property("autovizLeftEdge").toBool(), false);
  QCOMPARE(buttons[0]->property("autovizRightEdge").toBool(), true);
  QCOMPARE(buttons[2]->property("autovizLeftEdge").toBool(), true);
  QCOMPARE(buttons[2]->property("autovizRightEdge").toBool(), false);
}

}  // namespace
}  // namespace autoviz::widget

QTEST_MAIN(autoviz::widget::WidgetSegmentedControlTest)

#include "widget_segmented_control_test.moc"
