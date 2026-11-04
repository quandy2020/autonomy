/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <QAbstractButton>
#include <QPalette>
#include <QPushButton>
#include <QSignalSpy>
#include <QTest>
#include <QToolButton>

#include "autoviz/common/widget/feedback/info_bar.hpp"
#include "autoviz/common/widget/navigation/breadcrumb.hpp"
#include "autoviz/common/widget/navigation/navigation_rail.hpp"

namespace autoviz::widget {
namespace {

class WidgetNavigationTest final : public QObject {
  Q_OBJECT

 private slots:
  void InfoBarUsesToneAndAccessibleText();
  void BreadcrumbTracksKeyboardSelectionAndDecorativeSeparators();
  void NavigationRailTracksKeyboardSelectionAndTone();
};

void WidgetNavigationTest::InfoBarUsesToneAndAccessibleText() {
  InfoBar bar;
  QCOMPARE(bar.property("autovizRole").toString(), QStringLiteral("infoBar"));
  QCOMPARE(bar.tone(), style::Tone::Shell);

  bar.SetTitle(QStringLiteral("Connection"));
  bar.SetMessage(QString());
  QCOMPARE(bar.accessibleName(), QStringLiteral("Connection"));
  QCOMPARE(bar.accessibleDescription(), bar.accessibleName());

  bar.SetTitle(QString());
  bar.SetMessage(QStringLiteral("Receiving localization data"));
  QCOMPARE(bar.accessibleName(), QStringLiteral("Receiving localization data"));

  bar.SetTone(style::Tone::Frost);
  QCOMPARE(bar.tone(), style::Tone::Frost);
  QCOMPARE(bar.property("autovizTone").toString(), QStringLiteral("frost"));
  QCOMPARE(bar.palette().color(QPalette::Base),
           GetThemeTokens(style::Tone::Frost).surface);

  auto* close_button =
      bar.findChild<QPushButton*>(QStringLiteral("AutovizInfoBarCloseButton"));
  QVERIFY(close_button != nullptr);
  bar.SetClosable(false);
  QVERIFY(close_button->isHidden());

  QSignalSpy close_spy(&bar, &InfoBar::CloseRequested);
  bar.SetClosable(true);
  bar.show();
  QTRY_VERIFY(bar.isVisible());
  close_button->click();
  QCOMPARE(close_spy.count(), 1);
  QTRY_VERIFY(!bar.isVisible());
}

void WidgetNavigationTest::
    BreadcrumbTracksKeyboardSelectionAndDecorativeSeparators() {
  Breadcrumb breadcrumb;
  QSignalSpy current_spy(&breadcrumb, &Breadcrumb::CurrentIndexChanged);
  QSignalSpy activated_spy(&breadcrumb, &Breadcrumb::ItemActivated);
  breadcrumb.SetItems({
      {QStringLiteral("Home"), QStringLiteral("home"), true},
      {QStringLiteral("Disabled"), QStringLiteral("disabled"), false},
      {QStringLiteral("Map"), QStringLiteral("map"), true},
  });

  QCOMPARE(breadcrumb.current_index(), 2);
  QCOMPARE(current_spy.count(), 1);
  QVERIFY(breadcrumb.accessibleDescription().contains(QStringLiteral("Map")));

  breadcrumb.show();
  breadcrumb.setFocus();
  QTRY_VERIFY(breadcrumb.hasFocus());
  QTest::keyClick(&breadcrumb, Qt::Key_Home);
  QCOMPARE(breadcrumb.current_index(), 0);
  QVERIFY(breadcrumb.accessibleDescription().contains(QStringLiteral("Home")));
  QTest::keyClick(&breadcrumb, Qt::Key_Right);
  QCOMPARE(breadcrumb.current_index(), 2);
  QTest::keyClick(&breadcrumb, Qt::Key_Return);
  QCOMPARE(activated_spy.count(), 1);
  QCOMPARE(activated_spy.at(0).at(1).toString(), QStringLiteral("map"));

  const QList<QWidget*> separators = breadcrumb.findChildren<QWidget*>(
      QStringLiteral("AutovizBreadcrumbSeparator"));
  QCOMPARE(separators.size(), 2);
  for (const QWidget* separator : separators) {
    QVERIFY(separator->accessibleName().isEmpty());
    QCOMPARE(separator->focusPolicy(), Qt::NoFocus);
    QCOMPARE(separator->property("autovizRole").toString(),
             QStringLiteral("decorative"));
  }

  breadcrumb.SetTone(style::Tone::Frost);
  QCOMPARE(breadcrumb.tone(), style::Tone::Frost);
  QCOMPARE(breadcrumb.property("autovizTone").toString(),
           QStringLiteral("frost"));
  QCOMPARE(current_spy.count(), 3);
}

void WidgetNavigationTest::NavigationRailTracksKeyboardSelectionAndTone() {
  NavigationRail rail;
  QSignalSpy current_spy(&rail, &NavigationRail::CurrentChanged);
  QSignalSpy activated_spy(&rail, &NavigationRail::ItemActivated);
  rail.SetItems({
      {QStringLiteral("Overview"), QIcon(), QStringLiteral("overview"), true},
      {QStringLiteral("Disabled"), QIcon(), QStringLiteral("disabled"), false},
      {QStringLiteral("Diagnostics"), QIcon(), QStringLiteral("diagnostics"),
       true},
  });

  QCOMPARE(rail.current_index(), 0);
  QCOMPARE(current_spy.count(), 1);
  QVERIFY(rail.accessibleDescription().contains(QStringLiteral("Overview")));

  rail.show();
  rail.setFocus();
  QTRY_VERIFY(rail.hasFocus());
  QTest::keyClick(&rail, Qt::Key_End);
  QCOMPARE(rail.current_index(), 2);
  QVERIFY(rail.accessibleDescription().contains(QStringLiteral("Diagnostics")));
  QTest::keyClick(&rail, Qt::Key_Up);
  QCOMPARE(rail.current_index(), 0);
  QTest::keyClick(&rail, Qt::Key_Space);
  QCOMPARE(activated_spy.count(), 1);
  QCOMPARE(activated_spy.at(0).at(1).toString(), QStringLiteral("overview"));

  rail.SetLabelsVisible(false);
  const QList<QToolButton*> buttons =
      rail.findChildren<QToolButton*>(QString(), Qt::FindDirectChildrenOnly);
  QCOMPARE(buttons.size(), 3);
  for (int i = 0; i < buttons.size(); ++i) {
    QVERIFY(buttons.at(i)->text().isEmpty());
    QCOMPARE(buttons.at(i)->accessibleName(), rail.items().at(i).text);
  }

  rail.SetTone(style::Tone::Frost);
  QCOMPARE(rail.tone(), style::Tone::Frost);
  QCOMPARE(rail.property("autovizTone").toString(), QStringLiteral("frost"));
  for (const QToolButton* button : buttons) {
    QCOMPARE(button->minimumHeight(),
             GetThemeTokens(style::Tone::Frost).control_height_large);
  }

  rail.Clear();
  QCOMPARE(rail.current_index(), -1);
  QCOMPARE(rail.accessibleDescription(), QStringLiteral("No navigation items"));
  QCOMPARE(current_spy.count(), 3);
}

}  // namespace
}  // namespace autoviz::widget

QTEST_MAIN(autoviz::widget::WidgetNavigationTest)

#include "widget_navigation_test.moc"
