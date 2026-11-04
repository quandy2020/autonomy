/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <QApplication>
#include <QImage>
#include <QSignalSpy>
#include <QStyle>
#include <QTest>
#include <QVector>
#include <atomic>
#include <cmath>
#include <limits>
#include <thread>

#include "autoviz/common/widget/data/timeline_scrubber.hpp"
#include "autoviz/common/widget/display/status_badge.hpp"
#include "autoviz/common/widget/feedback/progress_ring.hpp"
#include "autoviz/common/widget/foundation/motion_policy.hpp"
#include "autoviz/common/widget/input/toggle_switch.hpp"
#include "autoviz/common/widget/layout/surface.hpp"

namespace autoviz::widget {
namespace {

class MotionPolicyGuard final {
 public:
  MotionPolicyGuard()
      : preference_(MotionPolicy::preference()),
        autoviz_was_set_(qEnvironmentVariableIsSet("AUTOVIZ_REDUCED_MOTION")),
        autoviz_value_(qgetenv("AUTOVIZ_REDUCED_MOTION")),
        generic_was_set_(qEnvironmentVariableIsSet("PREFERS_REDUCED_MOTION")),
        generic_value_(qgetenv("PREFERS_REDUCED_MOTION")) {}

  ~MotionPolicyGuard() {
    Restore("AUTOVIZ_REDUCED_MOTION", autoviz_was_set_, autoviz_value_);
    Restore("PREFERS_REDUCED_MOTION", generic_was_set_, generic_value_);
    MotionPolicy::SetPreference(preference_);
    MotionPolicy::RefreshSystemPreference();
  }

 private:
  static void Restore(const char* name, bool was_set, const QByteArray& value) {
    if (was_set) {
      qputenv(name, value);
    } else {
      qunsetenv(name);
    }
  }

  MotionPreference preference_;
  bool autoviz_was_set_ = false;
  QByteArray autoviz_value_;
  bool generic_was_set_ = false;
  QByteArray generic_value_;
};

QImage RenderAtDevicePixelRatio(QWidget* widget, qreal device_pixel_ratio) {
  const QSize logical_size = widget->size();
  const QSize device_size(
      static_cast<int>(std::ceil(logical_size.width() * device_pixel_ratio)),
      static_cast<int>(std::ceil(logical_size.height() * device_pixel_ratio)));
  QImage image(device_size, QImage::Format_ARGB32_Premultiplied);
  image.setDevicePixelRatio(device_pixel_ratio);
  image.fill(Qt::transparent);
  widget->render(&image);
  return image;
}

class WidgetFoundationTest final : public QObject {
  Q_OBJECT

 private slots:
  void MotionPolicyHonorsOverridesEnvironmentAndStyle();
  void TimelineKeepsNearMaximumMarkersPrecise();
  void RepresentativeWidgetsRenderAtCommonDevicePixelRatios();
};

void WidgetFoundationTest::MotionPolicyHonorsOverridesEnvironmentAndStyle() {
  MotionPolicyGuard guard;

  MotionPolicy::SetPreference(MotionPreference::kFullMotion);
  QVERIFY(!MotionPolicy::ReducedMotionEnabled());
  QCOMPARE(MotionPolicy::ResolveDuration(180), 180);
  QCOMPARE(MotionPolicy::ResolveDuration(-1), 0);

  MotionPolicy::SetPreference(MotionPreference::kReducedMotion);
  QVERIFY(MotionPolicy::ReducedMotionEnabled());
  QCOMPARE(MotionPolicy::ResolveDuration(180), 0);

  std::atomic_bool worker_observed_policy{false};
  std::thread worker([&worker_observed_policy]() {
    worker_observed_policy.store(MotionPolicy::ReducedMotionEnabled() &&
                                     MotionPolicy::ResolveDuration(180) == 0,
                                 std::memory_order_release);
  });
  worker.join();
  QVERIFY(worker_observed_policy.load(std::memory_order_acquire));

  MotionPolicy::SetPreference(MotionPreference::kSystem);
  QVERIFY(qputenv("AUTOVIZ_REDUCED_MOTION", "true"));
  MotionPolicy::RefreshSystemPreference();
  QVERIFY(MotionPolicy::ReducedMotionEnabled());

  QVERIFY(qputenv("AUTOVIZ_REDUCED_MOTION", "false"));
  MotionPolicy::RefreshSystemPreference();
  QVERIFY(!MotionPolicy::ReducedMotionEnabled());

  QVERIFY(qunsetenv("AUTOVIZ_REDUCED_MOTION"));
  QVERIFY(qunsetenv("PREFERS_REDUCED_MOTION"));
  MotionPolicy::RefreshSystemPreference();
  QCOMPARE(MotionPolicy::ReducedMotionEnabled(),
           QApplication::style()->styleHint(QStyle::SH_Widget_Animate) == 0);
}

void WidgetFoundationTest::TimelineKeepsNearMaximumMarkersPrecise() {
  TimelineScrubber timeline;
  timeline.resize(432, 44);
  const qint64 maximum = std::numeric_limits<qint64>::max();
  const qint64 minimum = maximum - 100;
  const qint64 midpoint = minimum + 50;
  timeline.SetRange(minimum, maximum);
  timeline.SetValue(minimum);
  timeline.SetMarkers(
      {TimelineMarker{midpoint, QColor(), QStringLiteral("near maximum")}});
  timeline.show();
  QCoreApplication::processEvents();

  QSignalSpy marker_spy(&timeline, &TimelineScrubber::MarkerActivated);
  const QPoint center(timeline.width() / 2, timeline.height() / 2);
  QTest::mousePress(&timeline, Qt::LeftButton, Qt::NoModifier, center);
  QTest::mouseRelease(&timeline, Qt::LeftButton, Qt::NoModifier, center);

  QCOMPARE(timeline.value(), midpoint);
  QCOMPARE(marker_spy.count(), 1);
  QCOMPARE(marker_spy.first().at(0).toInt(), 0);
}

void WidgetFoundationTest::
    RepresentativeWidgetsRenderAtCommonDevicePixelRatios() {
  StatusBadge badge(QStringLiteral("Connected"), StatusBadge::Status::kSuccess);
  badge.resize(badge.sizeHint());
  ProgressRing progress;
  progress.resize(64, 64);
  progress.SetValue(67);
  ToggleSwitch toggle;
  toggle.resize(toggle.sizeHint());
  toggle.setChecked(true);
  Surface surface;
  surface.resize(160, 80);
  surface.SetElevation(Surface::Elevation::kRaised);
  TimelineScrubber timeline;
  timeline.resize(360, 44);
  timeline.SetRange(0, 100);
  timeline.SetValue(45);

  const QVector<QWidget*> widgets = {&badge, &progress, &toggle, &surface,
                                     &timeline};
  for (QWidget* widget : widgets) {
    widget->show();
  }
  QCoreApplication::processEvents();

  const QVector<qreal> device_pixel_ratios = {1.0, 1.5, 2.0};
  for (QWidget* widget : widgets) {
    for (const qreal device_pixel_ratio : device_pixel_ratios) {
      QImage blank = RenderAtDevicePixelRatio(widget, device_pixel_ratio);
      blank.fill(Qt::transparent);
      const QImage rendered =
          RenderAtDevicePixelRatio(widget, device_pixel_ratio);
      QCOMPARE(rendered.devicePixelRatio(), device_pixel_ratio);
      QVERIFY(rendered != blank);
    }
  }
}

}  // namespace
}  // namespace autoviz::widget

QTEST_MAIN(autoviz::widget::WidgetFoundationTest)
#include "widget_foundation_test.moc"
