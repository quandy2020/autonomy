/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <QCoreApplication>
#include <QImage>
#include <QLabel>
#include <QPushButton>
#include <QSignalSpy>
#include <QTest>
#include <cmath>
#include <limits>

#include "autoviz/common/widget/data/loading_overlay.hpp"
#include "autoviz/common/widget/display/sparkline.hpp"
#include "autoviz/common/widget/display/status_badge.hpp"
#include "autoviz/common/widget/feedback/progress_ring.hpp"
#include "autoviz/common/widget/feedback/toast.hpp"
#include "autoviz/common/widget/layout/collapsible_panel.hpp"
#include "autoviz/common/widget/overlay/drawer.hpp"
#include "autoviz/common/widget/overlay/popover.hpp"
#include "autoviz/common/widget/overlay/teaching_tip.hpp"

namespace autoviz::widget {
namespace {

class WidgetLifecycleTest final : public QObject {
  Q_OBJECT

 private slots:
  void CollapsiblePanelTransfersContentAndSignalsOnce();
  void LoadingOverlayTracksTargetAndBlocksInput();
  void DrawerHandlesRepeatedShowDismissAndReparent();
  void SparklineKeepsABoundedFiniteHistory();
  void StatusBadgeElidesWithoutLosingMeaning();
  void ProgressRingRendersTheFullIntegerRange();
  void PopoverAndTeachingTipHaveStableSessions();
  void ToastTimerFollowsVisibleLifetime();
};

void WidgetLifecycleTest::CollapsiblePanelTransfersContentAndSignalsOnce() {
  QWidget host;
  host.resize(420, 240);
  CollapsiblePanel panel(&host);
  panel.resize(360, 180);
  panel.SetAnimationEnabled(false);
  panel.SetTitle(QStringLiteral("Details"));
  auto* content = new QLabel(QStringLiteral("Content"));
  panel.SetContentWidget(content);
  host.show();
  panel.show();

  QSignalSpy expanded_spy(&panel, &CollapsiblePanel::ExpandedChanged);
  panel.SetExpanded(false);
  QCOMPARE(expanded_spy.count(), 1);
  QVERIFY(panel.content_widget()->parentWidget()->isHidden());
  panel.SetExpanded(false);
  QCOMPARE(expanded_spy.count(), 1);

  panel.SetExpanded(true);
  QCOMPARE(expanded_spy.count(), 2);
  QVERIFY(!panel.content_widget()->parentWidget()->isHidden());
  QCOMPARE(panel.TakeContentWidget(), content);
  QCOMPARE(content->parentWidget(), nullptr);
  delete content;
}

void WidgetLifecycleTest::LoadingOverlayTracksTargetAndBlocksInput() {
  QWidget target;
  target.resize(320, 180);
  QPushButton child(QStringLiteral("Action"), &target);
  child.setGeometry(20, 20, 100, 32);
  LoadingOverlay overlay(&target);
  QSignalSpy click_spy(&child, &QPushButton::clicked);
  QSignalSpy loading_spy(&overlay, &LoadingOverlay::LoadingChanged);
  target.show();
  child.show();

  overlay.SetLoading(true);
  QCOMPARE(loading_spy.count(), 1);
  overlay.SetLoading(true);
  QCOMPARE(loading_spy.count(), 1);
  QTRY_VERIFY(overlay.isVisible());
  QCOMPARE(overlay.geometry(), target.rect());
  QCOMPARE(overlay.focusPolicy(), Qt::StrongFocus);

  QTest::mouseClick(&child, Qt::LeftButton);
  QCOMPARE(click_spy.count(), 0);

  overlay.SetBlocksInput(false);
  QCOMPARE(overlay.focusPolicy(), Qt::NoFocus);
  QTest::mouseClick(&child, Qt::LeftButton);
  QCOMPARE(click_spy.count(), 1);

  target.resize(480, 260);
  QCoreApplication::processEvents();
  QCOMPARE(overlay.geometry(), target.rect());
  target.hide();
  QTRY_VERIFY(!overlay.isVisible());
  target.show();
  QTRY_VERIFY(overlay.isVisible());

  QWidget second_target;
  second_target.resize(240, 160);
  QPushButton second_child(QStringLiteral("Second action"), &second_target);
  second_child.setGeometry(20, 20, 120, 32);
  QSignalSpy second_click_spy(&second_child, &QPushButton::clicked);
  second_target.show();
  second_child.show();

  overlay.SetBlocksInput(true);
  overlay.AttachTo(&second_target);
  QCOMPARE(overlay.target_widget(), &second_target);
  QCOMPARE(overlay.parentWidget(), &second_target);
  QTRY_VERIFY(overlay.isVisible());
  QCOMPARE(overlay.geometry(), second_target.rect());

  QTest::mouseClick(&child, Qt::LeftButton);
  QCOMPARE(click_spy.count(), 2);
  QTest::mouseClick(&second_child, Qt::LeftButton);
  QCOMPARE(second_click_spy.count(), 0);

  overlay.AttachTo(nullptr);
  QCOMPARE(overlay.target_widget(), nullptr);
  QCOMPARE(overlay.parentWidget(), nullptr);
  QVERIFY(!overlay.isVisible());
  QTest::mouseClick(&second_child, Qt::LeftButton);
  QCOMPARE(second_click_spy.count(), 1);

  overlay.SetLoading(false);
  QCOMPARE(loading_spy.count(), 2);
  QVERIFY(!overlay.isVisible());
}

void WidgetLifecycleTest::DrawerHandlesRepeatedShowDismissAndReparent() {
  QWidget first_host;
  first_host.resize(500, 300);
  first_host.show();
  QWidget second_host;
  second_host.resize(640, 360);
  second_host.show();
  Drawer drawer(&first_host);
  drawer.SetContentWidget(new QLabel(QStringLiteral("Inspector")));
  QSignalSpy opened_spy(&drawer, &Drawer::Opened);
  QSignalSpy closed_spy(&drawer, &Drawer::Closed);

  drawer.Show();
  drawer.Show();
  QTRY_COMPARE(opened_spy.count(), 1);
  drawer.Show();
  QCOMPARE(opened_spy.count(), 1);
  drawer.Dismiss();
  drawer.Dismiss();
  QTRY_COMPARE(closed_spy.count(), 1);

  drawer.setParent(&second_host);
  drawer.Show();
  QTRY_COMPARE(opened_spy.count(), 2);
  QCOMPARE(drawer.geometry(), second_host.rect());
  drawer.Dismiss();
  QTRY_COMPARE(closed_spy.count(), 2);
}

void WidgetLifecycleTest::SparklineKeepsABoundedFiniteHistory() {
  Sparkline sparkline;
  sparkline.SetCapacity(3);
  sparkline.AppendSamples({1.0, std::numeric_limits<double>::quiet_NaN(), 2.0,
                           std::numeric_limits<double>::infinity(), 3.0, 4.0});
  QCOMPARE(sparkline.sample_count(), 3);
  QCOMPARE(sparkline.capacity(), 3);

  sparkline.SetCapacity(2);
  QCOMPARE(sparkline.sample_count(), 2);
  sparkline.resize(180, 56);
  sparkline.show();
  QTRY_VERIFY(sparkline.accessibleDescription().contains(
      QStringLiteral("latest value 4")));

  sparkline.hide();
  const QString hidden_description = sparkline.accessibleDescription();
  sparkline.AppendSamples({5.0, std::numeric_limits<double>::quiet_NaN(), 6.0});
  QCOMPARE(sparkline.sample_count(), 2);
  QCOMPARE(sparkline.accessibleDescription(), hidden_description);

  sparkline.show();
  QTRY_VERIFY(sparkline.accessibleDescription().contains(
      QStringLiteral("latest value 6")));
  sparkline.Clear();
  QCOMPARE(sparkline.sample_count(), 0);
  QCOMPARE(sparkline.accessibleDescription(), QStringLiteral("No samples"));
}

void WidgetLifecycleTest::StatusBadgeElidesWithoutLosingMeaning() {
  const QString full_text = QStringLiteral(
      "Waiting for /localization/atlas/very_long_status_channel");
  StatusBadge badge(full_text, StatusBadge::Status::kInformation);
  const int preferred_width = badge.sizeHint().width();
  const int preferred_height = badge.sizeHint().height();

  QVERIFY(badge.minimumSizeHint().width() < preferred_width);
  badge.resize(72, preferred_height);
  badge.show();
  QTRY_COMPARE(badge.toolTip(), full_text);
  QCOMPARE(badge.accessibleName(), full_text);

  const QString custom_tool_tip =
      QStringLiteral("Caller-provided diagnostic details");
  badge.setToolTip(custom_tool_tip);
  badge.resize(68, preferred_height);
  QCoreApplication::processEvents();
  QCOMPARE(badge.toolTip(), custom_tool_tip);

  badge.resize(preferred_width, preferred_height);
  QCoreApplication::processEvents();
  QCOMPARE(badge.toolTip(), custom_tool_tip);

  badge.setToolTip(QString());
  badge.resize(70, preferred_height);
  QTRY_COMPARE(badge.toolTip(), full_text);
  badge.resize(preferred_width, preferred_height);
  QTRY_VERIFY(badge.toolTip().isEmpty());
  QCOMPARE(badge.accessibleName(), full_text);
}

void WidgetLifecycleTest::ProgressRingRendersTheFullIntegerRange() {
  ProgressRing ring;
  QCOMPARE(ring.accessibleDescription(),
           QStringLiteral("0 percent; value 0; range 0 to 100"));
  ring.SetRange(-50, 50);
  QCOMPARE(ring.accessibleDescription(),
           QStringLiteral("50 percent; value 0; range -50 to 50"));
  ring.SetIndeterminate(true);
  QCOMPARE(ring.accessibleDescription(),
           QStringLiteral("Progress is indeterminate"));
  ring.SetIndeterminate(false);
  QCOMPARE(ring.accessibleDescription(),
           QStringLiteral("50 percent; value 0; range -50 to 50"));

  ring.resize(64, 64);
  ring.SetRange(std::numeric_limits<int>::min(),
                std::numeric_limits<int>::max());
  ring.SetValue(0);
  ring.SetIndeterminate(false);

  const QString full_range_description = ring.accessibleDescription();
  QVERIFY(full_range_description.startsWith(QStringLiteral("50 percent")));
  QVERIFY(full_range_description.contains(
      QString::number(std::numeric_limits<int>::min())));
  QVERIFY(full_range_description.contains(
      QString::number(std::numeric_limits<int>::max())));

  QImage image(ring.size(), QImage::Format_ARGB32_Premultiplied);
  image.fill(Qt::transparent);
  QImage blank(ring.size(), QImage::Format_ARGB32_Premultiplied);
  blank.fill(Qt::transparent);
  ring.render(&image);
  QVERIFY(!image.isNull());
  QVERIFY(image != blank);
}

void WidgetLifecycleTest::PopoverAndTeachingTipHaveStableSessions() {
  QWidget host;
  host.resize(480, 320);
  QPushButton anchor(QStringLiteral("Anchor"), &host);
  anchor.setGeometry(20, 20, 100, 32);
  host.show();

  Popover popover(&host);
  auto* content = new QLabel(QStringLiteral("Popover content"));
  popover.SetContentWidget(content);
  QCOMPARE(popover.TakeContentWidget(), content);
  QCOMPARE(content->parentWidget(), nullptr);
  delete content;
  popover.SetContentWidget(new QLabel(QStringLiteral("Replacement content")));
  popover.ShowFor(&anchor);
  popover.ShowFor(&anchor, Popover::Placement::kBottomEnd);
  QTRY_VERIFY(popover.isVisible());
  QTest::keyClick(&popover, Qt::Key_Escape);
  QTRY_VERIFY(!popover.isVisible());
  popover.Dismiss();
  QVERIFY(!popover.isVisible());

  TeachingTip tip(&host);
  tip.SetTitle(QStringLiteral("Navigation"));
  tip.SetMessage(QStringLiteral("Use the map controls."));
  QSignalSpy dismissed_spy(&tip, &TeachingTip::Dismissed);
  tip.ShowFor(&anchor);
  tip.ShowFor(&anchor, Popover::Placement::kTopStart);
  tip.Dismiss();
  QTRY_COMPARE(dismissed_spy.count(), 1);
  tip.Dismiss();
  QCOMPARE(dismissed_spy.count(), 1);

  tip.ShowFor(&anchor);
  tip.Dismiss();
  QTRY_COMPARE(dismissed_spy.count(), 2);
}

void WidgetLifecycleTest::ToastTimerFollowsVisibleLifetime() {
  QWidget host;
  host.resize(480, 320);
  host.show();

  Toast toast(&host);
  toast.SetMessage(QStringLiteral("Notification"));
  toast.SetDurationMs(50);
  QSignalSpy dismissed_spy(&toast, &Toast::Dismissed);
  toast.Show();
  QTRY_VERIFY(toast.isVisible());

  toast.SetDurationMs(0);
  QTest::qWait(80);
  QVERIFY(toast.isVisible());
  QCOMPARE(dismissed_spy.count(), 0);

  toast.SetDurationMs(30);
  QTRY_COMPARE_WITH_TIMEOUT(dismissed_spy.count(), 1, 200);
  QVERIFY(!toast.isVisible());
  QTest::qWait(50);
  QCOMPARE(dismissed_spy.count(), 1);

  toast.SetDurationMs(50);
  toast.Show();
  QTRY_VERIFY(toast.isVisible());
  QTest::qWait(10);
  toast.hide();
  QTest::qWait(70);
  QCOMPARE(dismissed_spy.count(), 1);

  toast.Show();
  toast.SetDurationMs(20);
  QTRY_COMPARE_WITH_TIMEOUT(dismissed_spy.count(), 2, 200);
  QTest::qWait(40);
  QCOMPARE(dismissed_spy.count(), 2);
}

}  // namespace
}  // namespace autoviz::widget

QTEST_MAIN(autoviz::widget::WidgetLifecycleTest)

#include "widget_lifecycle_test.moc"
