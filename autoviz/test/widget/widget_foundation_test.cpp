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
#include "autoviz/common/widget/foundation/theme_tokens.hpp"
#include "autoviz/common/widget/foundation/widget_theme.hpp"
#include "autoviz/common/widget/input/button.hpp"
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

qreal LinearChannel(qreal channel) {
  return channel <= 0.04045 ? channel / 12.92
                            : std::pow((channel + 0.055) / 1.055, 2.4);
}

qreal ContrastRatio(const QColor& foreground, const QColor& background) {
  const qreal foreground_luminance =
      0.2126 * LinearChannel(foreground.redF()) +
      0.7152 * LinearChannel(foreground.greenF()) +
      0.0722 * LinearChannel(foreground.blueF());
  const qreal background_luminance =
      0.2126 * LinearChannel(background.redF()) +
      0.7152 * LinearChannel(background.greenF()) +
      0.0722 * LinearChannel(background.blueF());
  return (qMax(foreground_luminance, background_luminance) + 0.05) /
         (qMin(foreground_luminance, background_luminance) + 0.05);
}

class WidgetFoundationTest final : public QObject {
  Q_OBJECT

 private slots:
  void MotionPolicyHonorsOverridesEnvironmentAndStyle();
  void SemanticThemeMapsNativePaletteAndStateGroups();
  void ExplicitAppearanceChangesSelfPaintedWidgets();
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

void WidgetFoundationTest::SemanticThemeMapsNativePaletteAndStateGroups() {
  const ThemeTokens& light =
      GetThemeTokens(style::Tone::Shell, ThemeAppearance::kLight);
  const ThemeTokens& dark =
      GetThemeTokens(style::Tone::Shell, ThemeAppearance::kDark);
  const ThemeTokens& frost_light =
      GetThemeTokens(style::Tone::Frost, ThemeAppearance::kLight);
  const ThemeTokens& frost_dark =
      GetThemeTokens(style::Tone::Frost, ThemeAppearance::kDark);
  QVERIFY(!light.is_dark);
  QVERIFY(dark.is_dark);
  QVERIFY(!frost_light.is_dark);
  QVERIFY(frost_dark.is_dark);
  QVERIFY(qGray(light.window.rgb()) > qGray(dark.window.rgb()));
  QVERIFY(qGray(light.text.rgb()) < qGray(dark.text.rgb()));
  QCOMPARE(light.border, light.border_subtle);
  QCOMPARE(dark.border, dark.border_subtle);

  QCOMPARE(light.window, QColor(242, 242, 247));
  QCOMPARE(light.surface, QColor(255, 255, 255));
  QCOMPARE(light.accent, QColor(0, 122, 255));
  QCOMPARE(light.accent_filled, QColor(0, 113, 227));
  QCOMPARE(light.accent_filled_hovered, QColor(0, 110, 220));
  QCOMPARE(light.accent_filled_pressed, QColor(0, 91, 181));
  QCOMPARE(light.on_accent_filled, QColor(Qt::white));
  QCOMPARE(light.success, QColor(52, 199, 89));
  QCOMPARE(light.warning, QColor(255, 149, 0));
  QCOMPARE(light.danger, QColor(255, 59, 48));
  QCOMPARE(light.danger_filled, QColor(215, 0, 21));
  QCOMPARE(light.danger_filled_hovered, QColor(204, 0, 20));
  QCOMPARE(light.danger_filled_pressed, QColor(184, 0, 18));
  QCOMPARE(light.on_danger_filled, QColor(Qt::white));
  QCOMPARE(dark.window, QColor(0, 0, 0));
  QCOMPARE(dark.surface, QColor(28, 28, 30));
  QCOMPARE(dark.surface_raised, QColor(44, 44, 46));
  QCOMPARE(dark.surface_material, QColor(58, 58, 60));
  QCOMPARE(dark.accent, QColor(10, 132, 255));
  QCOMPARE(dark.accent_filled, QColor(0, 113, 227));
  QCOMPARE(dark.on_accent_filled, QColor(Qt::white));
  QCOMPARE(dark.danger_filled, QColor(215, 0, 21));
  QCOMPARE(dark.on_danger_filled, QColor(Qt::white));
  QCOMPARE(light.control_height_compact, 28);
  QCOMPARE(light.control_height_small, 32);
  QCOMPARE(light.control_height_medium, 44);
  QCOMPARE(light.control_height_large, 50);

  for (const ThemeTokens* tokens : {&light, &dark, &frost_light, &frost_dark}) {
    QVERIFY(tokens->surface_sunken.isValid());
    QVERIFY(tokens->surface_material.isValid());
    QVERIFY(tokens->surface_selected.isValid());
    QVERIFY(tokens->text_secondary.isValid());
    QVERIFY(tokens->placeholder.isValid());
    QVERIFY(tokens->border_strong.isValid());
    QVERIFY(tokens->focus_ring_outer.isValid());
    QVERIFY(tokens->radius_small < tokens->radius_medium);
    QVERIFY(tokens->radius_medium < tokens->radius_large);
    QVERIFY(tokens->focus_ring_width >= 3);
    QVERIFY(ContrastRatio(tokens->text, tokens->surface) >= 4.5);
    QVERIFY(ContrastRatio(tokens->text_secondary, tokens->surface) >= 4.5);
    QVERIFY(ContrastRatio(tokens->focus_ring, tokens->surface) >= 3.0);
    QVERIFY(ContrastRatio(tokens->border_strong, tokens->surface) >= 3.0);
    QVERIFY(ContrastRatio(tokens->accent_text, tokens->surface) >= 4.5);
    QVERIFY(ContrastRatio(tokens->on_accent, tokens->accent) >= 4.5);
    QVERIFY(ContrastRatio(tokens->on_accent_hovered, tokens->accent_hovered) >=
            4.5);
    QVERIFY(ContrastRatio(tokens->on_accent_pressed, tokens->accent_pressed) >=
            4.5);
    QVERIFY(ContrastRatio(tokens->on_accent_filled, tokens->accent_filled) >=
            4.5);
    QVERIFY(ContrastRatio(tokens->on_accent_filled,
                          tokens->accent_filled_hovered) >= 4.5);
    QVERIFY(ContrastRatio(tokens->on_accent_filled,
                          tokens->accent_filled_pressed) >= 4.5);
    QVERIFY(ContrastRatio(tokens->on_danger, tokens->danger) >= 4.5);
    QVERIFY(ContrastRatio(tokens->on_danger_hovered, tokens->danger_hovered) >=
            4.5);
    QVERIFY(ContrastRatio(tokens->on_danger_pressed, tokens->danger_pressed) >=
            4.5);
    QVERIFY(ContrastRatio(tokens->on_danger_filled, tokens->danger_filled) >=
            4.5);
    QVERIFY(ContrastRatio(tokens->on_danger_filled,
                          tokens->danger_filled_hovered) >= 4.5);
    QVERIFY(ContrastRatio(tokens->on_danger_filled,
                          tokens->danger_filled_pressed) >= 4.5);
  }

  QPalette light_base;
  light_base.setColor(QPalette::Window, Qt::white);
  const QPalette light_palette = BuildWidgetPalette(
      style::Tone::Shell, light_base, ThemeAppearance::kSystem);
  QCOMPARE(light_palette.color(QPalette::Active, QPalette::Base),
           light.surface);
  QCOMPARE(light_palette.color(QPalette::Active, QPalette::AlternateBase),
           light.surface_sunken);
  QCOMPARE(light_palette.color(QPalette::Active, QPalette::Button),
           light.surface_material);
  QCOMPARE(light_palette.color(QPalette::Active, QPalette::PlaceholderText),
           light.placeholder);
  QCOMPARE(light_palette.color(QPalette::Active, QPalette::Link),
           light.accent_text);

  QPalette dark_base;
  dark_base.setColor(QPalette::Window, Qt::black);
  const QPalette dark_palette = BuildWidgetPalette(
      style::Tone::Shell, dark_base, ThemeAppearance::kSystem);
  QCOMPARE(dark_palette.color(QPalette::Active, QPalette::Base), dark.surface);
  QCOMPARE(dark_palette.color(QPalette::Inactive, QPalette::Highlight),
           dark.surface_selected);
  QCOMPARE(dark_palette.color(QPalette::Disabled, QPalette::Text),
           dark.disabled);

  QWidget themed;
  QWidget existing_child(&themed);
  ApplyWidgetTheme(&themed, style::Tone::Shell, false, ThemeAppearance::kDark);
  QCOMPARE(themed.property("autovizTone").toString(), QStringLiteral("shell"));
  QCOMPARE(themed.property("autovizAppearance").toString(),
           QStringLiteral("dark"));
  QVERIFY(GetThemeTokens(&themed).is_dark);

  ApplyWidgetTheme(&themed, style::Tone::Shell, true, ThemeAppearance::kDark);
  QVERIFY(GetThemeTokens(&existing_child).is_dark);
  QWidget late_child(&themed);
  QVERIFY(GetThemeTokens(&late_child).is_dark);
}

void WidgetFoundationTest::ExplicitAppearanceChangesSelfPaintedWidgets() {
  Button light_button(QStringLiteral("Continue"));
  light_button.SetVariant(Button::Variant::kPrimary);
  light_button.resize(light_button.sizeHint());
  ApplyWidgetTheme(&light_button, style::Tone::Shell, false,
                   ThemeAppearance::kLight);

  Button dark_button(QStringLiteral("Continue"));
  dark_button.SetVariant(Button::Variant::kPrimary);
  dark_button.resize(dark_button.sizeHint());
  ApplyWidgetTheme(&dark_button, style::Tone::Shell, false,
                   ThemeAppearance::kDark);

  Surface light_surface;
  light_surface.SetElevation(Surface::Elevation::kRaised);
  light_surface.resize(160, 80);
  ApplyWidgetTheme(&light_surface, style::Tone::Shell, false,
                   ThemeAppearance::kLight);

  Surface dark_surface;
  dark_surface.SetElevation(Surface::Elevation::kRaised);
  dark_surface.resize(160, 80);
  ApplyWidgetTheme(&dark_surface, style::Tone::Shell, false,
                   ThemeAppearance::kDark);

  const QVector<QWidget*> themed_widgets = {&light_button, &dark_button,
                                            &light_surface, &dark_surface};
  for (QWidget* widget : themed_widgets) {
    widget->show();
  }
  QCoreApplication::processEvents();

  QVERIFY(!GetThemeTokens(&light_button).is_dark);
  QVERIFY(GetThemeTokens(&dark_button).is_dark);
  QVERIFY(RenderAtDevicePixelRatio(&light_button, 1.0) !=
          RenderAtDevicePixelRatio(&dark_button, 1.0));
  QVERIFY(RenderAtDevicePixelRatio(&light_surface, 1.0) !=
          RenderAtDevicePixelRatio(&dark_surface, 1.0));
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
