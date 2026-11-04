/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <QImage>
#include <QPainter>
#include <QTest>
#include <QtMath>
#include <limits>

#include "autoviz/common/widget/display/watermark.hpp"

namespace autoviz::widget {
namespace {

class WidgetCapabilityClosureTest final : public QObject {
  Q_OBJECT

 private slots:
  void WatermarkBoundsPropertiesAndExposesAccessibleText();
  void WatermarkRendersAtCommonDevicePixelRatios();
};

void WidgetCapabilityClosureTest::
    WatermarkBoundsPropertiesAndExposesAccessibleText() {
  Watermark watermark;
  watermark.SetText(QStringLiteral("INTERNAL"));
  watermark.SetOpacity(-1.0);
  QCOMPARE(watermark.opacity(), 0.0);
  watermark.SetOpacity(2.0);
  QCOMPARE(watermark.opacity(), 1.0);
  watermark.SetOpacity(std::numeric_limits<qreal>::quiet_NaN());
  QCOMPARE(watermark.opacity(), 1.0);
  watermark.SetAngleDegrees(270.0);
  QCOMPARE(watermark.angle_degrees(), 180.0);
  watermark.SetTileSpacing(QSize(-20, 1024));
  QCOMPARE(watermark.tile_spacing(), QSize(0, 512));
  QCOMPARE(watermark.accessibleName(), QStringLiteral("Watermark"));
  QVERIFY(
      watermark.accessibleDescription().contains(QStringLiteral("INTERNAL")));
  QVERIFY(watermark.testAttribute(Qt::WA_TransparentForMouseEvents));
}

void WidgetCapabilityClosureTest::WatermarkRendersAtCommonDevicePixelRatios() {
  Watermark watermark;
  watermark.SetText(QStringLiteral("AUTOVIZ"));
  watermark.SetOpacity(0.5);
  watermark.resize(320, 180);
  watermark.show();

  for (const qreal device_pixel_ratio : {1.0, 1.5, 2.0}) {
    const QSize device_size(qCeil(watermark.width() * device_pixel_ratio),
                            qCeil(watermark.height() * device_pixel_ratio));
    QImage image(device_size, QImage::Format_ARGB32_Premultiplied);
    image.setDevicePixelRatio(device_pixel_ratio);
    image.fill(Qt::transparent);
    watermark.render(&image);
    QCOMPARE(image.devicePixelRatio(), device_pixel_ratio);

    bool has_painted_pixel = false;
    for (int y = 0; y < image.height() && !has_painted_pixel; ++y) {
      const QRgb* row = reinterpret_cast<const QRgb*>(image.constScanLine(y));
      for (int x = 0; x < image.width(); ++x) {
        if (qAlpha(row[x]) != 0) {
          has_painted_pixel = true;
          break;
        }
      }
    }
    QVERIFY(has_painted_pixel);
  }
}

}  // namespace
}  // namespace autoviz::widget

QTEST_MAIN(autoviz::widget::WidgetCapabilityClosureTest)
#include "widget_capability_closure_test.moc"
