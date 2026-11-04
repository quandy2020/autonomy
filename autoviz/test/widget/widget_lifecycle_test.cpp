/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <QApplication>
#include <QCoreApplication>
#include <QIcon>
#include <QImage>
#include <QLabel>
#include <QPalette>
#include <QPixmap>
#include <QPointer>
#include <QPushButton>
#include <QSignalSpy>
#include <QTest>
#include <QVBoxLayout>
#include <cmath>
#include <limits>

#include "autoviz/common/widget/data/loading_overlay.hpp"
#include "autoviz/common/widget/display/sparkline.hpp"
#include "autoviz/common/widget/display/status_badge.hpp"
#include "autoviz/common/widget/feedback/progress_ring.hpp"
#include "autoviz/common/widget/feedback/toast.hpp"
#include "autoviz/common/widget/input/button.hpp"
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
  void CollapsiblePanelRefreshesSemanticHeaderIcon();
  void LoadingOverlayTracksTargetAndBlocksInput();
  void LoadingOverlayRestoresFocusSafelyAcrossLifecycle();
  void DrawerHandlesRepeatedShowDismissAndReparent();
  void DrawerTrapsTabFocusAndRestoresOpener();
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

void WidgetLifecycleTest::CollapsiblePanelRefreshesSemanticHeaderIcon() {
  CollapsiblePanel panel;
  Button* header = panel.findChild<Button*>();
  QVERIFY(header != nullptr);

  QPalette palette = panel.palette();
  palette.setColor(QPalette::Active, QPalette::ButtonText, QColor(12, 90, 210));
  panel.setPalette(palette);
  QCoreApplication::processEvents();
  const QIcon first_icon = header->icon();

  palette.setColor(QPalette::Active, QPalette::ButtonText, QColor(220, 52, 68));
  panel.setPalette(palette);
  QCoreApplication::processEvents();
  const QIcon second_icon = header->icon();
  QVERIFY(!first_icon.isNull());
  QVERIFY(!second_icon.isNull());

  for (const qreal device_pixel_ratio : {1.0, 2.0, 3.0}) {
    const QPixmap first =
        first_icon.pixmap(QSize(18, 18), device_pixel_ratio, QIcon::Normal);
    const QPixmap second =
        second_icon.pixmap(QSize(18, 18), device_pixel_ratio, QIcon::Normal);
    QCOMPARE(first.devicePixelRatio(), device_pixel_ratio);
    QCOMPARE(second.devicePixelRatio(), device_pixel_ratio);
    QVERIFY(!first.isNull());
    QVERIFY(!second.isNull());
    QVERIFY(first.toImage() != second.toImage());
  }
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

void WidgetLifecycleTest::LoadingOverlayRestoresFocusSafelyAcrossLifecycle() {
  QWidget target;
  target.resize(320, 180);
  auto* original = new QPushButton(QStringLiteral("Original"), &target);
  original->show();
  target.show();
  target.activateWindow();
  original->setFocus();
  QTRY_COMPARE(QApplication::focusWidget(), original);

  LoadingOverlay overlay(&target);
  overlay.SetLoading(true);
  QTRY_COMPARE(QApplication::focusWidget(), &overlay);
  overlay.SetLoading(true);
  overlay.SetLoading(false);
  QTRY_COMPARE(QApplication::focusWidget(), original);

  overlay.SetLoading(true);
  QTRY_COMPARE(QApplication::focusWidget(), &overlay);
  overlay.AttachTo(nullptr);
  QTRY_COMPARE(QApplication::focusWidget(), original);
  overlay.SetLoading(false);
  overlay.AttachTo(&target);

  auto* doomed = new QPushButton(QStringLiteral("Temporary"), &target);
  doomed->show();
  doomed->setFocus();
  QTRY_COMPARE(QApplication::focusWidget(), doomed);
  QPointer<QPushButton> doomed_guard(doomed);
  overlay.SetLoading(true);
  QTRY_COMPARE(QApplication::focusWidget(), &overlay);
  delete doomed;
  QVERIFY(doomed_guard.isNull());
  overlay.SetLoading(false);
  QTRY_VERIFY(!overlay.hasFocus());

  original->setFocus();
  QTRY_COMPARE(QApplication::focusWidget(), original);
  overlay.SetLoading(true);
  QTRY_COMPARE(QApplication::focusWidget(), &overlay);
  target.hide();
  QTRY_VERIFY(!overlay.isVisible());

  QWidget outside;
  QPushButton outside_button(QStringLiteral("Outside"), &outside);
  outside_button.show();
  outside.show();
  outside.activateWindow();
  outside_button.setFocus();
  QTRY_COMPARE(QApplication::focusWidget(), &outside_button);
  overlay.SetLoading(false);
  QCOMPARE(QApplication::focusWidget(), &outside_button);
}

void WidgetLifecycleTest::DrawerHandlesRepeatedShowDismissAndReparent() {
  QWidget first_host;
  first_host.resize(500, 300);
  QWidget second_host;
  second_host.resize(640, 360);
  QPushButton opener(QStringLiteral("Open inspector"), &first_host);
  opener.show();
  first_host.show();
  first_host.activateWindow();
  opener.setFocus();
  QTRY_COMPARE(QApplication::focusWidget(), &opener);

  Drawer drawer(&first_host);
  auto* inspector = new QPushButton(QStringLiteral("Inspector"));
  drawer.SetContentWidget(inspector);
  QSignalSpy opened_spy(&drawer, &Drawer::Opened);
  QSignalSpy closed_spy(&drawer, &Drawer::Closed);

  drawer.Show();
  drawer.Show();
  QTRY_COMPARE(opened_spy.count(), 1);
  QTRY_COMPARE(QApplication::focusWidget(), inspector);
  drawer.Show();
  QCOMPARE(opened_spy.count(), 1);
  drawer.Dismiss();
  drawer.Dismiss();
  QTRY_COMPARE(closed_spy.count(), 1);
  QTRY_COMPARE(QApplication::focusWidget(), &opener);

  second_host.show();
  drawer.setParent(&second_host);
  drawer.Show();
  QTRY_COMPARE(opened_spy.count(), 2);
  QCOMPARE(drawer.geometry(), second_host.rect());
  drawer.Dismiss();
  QTRY_COMPARE(closed_spy.count(), 2);
}

void WidgetLifecycleTest::DrawerTrapsTabFocusAndRestoresOpener() {
  QWidget host;
  host.resize(560, 360);
  QPushButton before(QStringLiteral("Before"), &host);
  QPushButton opener(QStringLiteral("Open drawer"), &host);
  QPushButton after(QStringLiteral("After"), &host);
  before.setGeometry(20, 20, 120, 32);
  opener.setGeometry(20, 64, 120, 32);
  after.setGeometry(20, 108, 120, 32);
  before.show();
  opener.show();
  after.show();
  host.show();
  host.activateWindow();
  opener.setFocus();
  QTRY_COMPARE(QApplication::focusWidget(), &opener);

  Drawer drawer(&host);
  auto* content = new QWidget;
  auto* content_layout = new QVBoxLayout(content);
  auto* first = new QPushButton(QStringLiteral("First"), content);
  auto* last = new QPushButton(QStringLiteral("Last"), content);
  content_layout->addWidget(first);
  content_layout->addWidget(last);
  QWidget::setTabOrder(first, last);
  drawer.SetContentWidget(content);

  drawer.Show();
  QTRY_COMPARE(QApplication::focusWidget(), first);
  QTest::keyClick(first, Qt::Key_Tab);
  QTRY_COMPARE(QApplication::focusWidget(), last);
  QTest::keyClick(last, Qt::Key_Tab);
  QTRY_COMPARE(QApplication::focusWidget(), first);
  QVERIFY(QApplication::focusWidget() != &after);
  QVERIFY(QApplication::focusWidget() != &before);

  QTest::keyClick(first, Qt::Key_Backtab);
  QTRY_COMPARE(QApplication::focusWidget(), last);
  QTest::keyClick(last, Qt::Key_Backtab);
  QTRY_COMPARE(QApplication::focusWidget(), first);
  QVERIFY(QApplication::focusWidget() != &after);
  QVERIFY(QApplication::focusWidget() != &before);

  drawer.Dismiss();
  QTRY_COMPARE(QApplication::focusWidget(), &opener);

  QCOMPARE(drawer.TakeContentWidget(), content);
  delete content;
  drawer.Show();
  QTRY_COMPARE(QApplication::focusWidget(), &drawer);
  QTest::keyClick(&drawer, Qt::Key_Tab);
  QCOMPARE(QApplication::focusWidget(), &drawer);
  QTest::keyClick(&drawer, Qt::Key_Backtab);
  QCOMPARE(QApplication::focusWidget(), &drawer);
  drawer.Dismiss();
  QTRY_COMPARE(QApplication::focusWidget(), &opener);
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
