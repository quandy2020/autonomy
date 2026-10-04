/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/viewport_toolbar.hpp"

#include <QApplication>
#include <QFile>
#include <QFont>
#include <QFrame>
#include <QPainter>
#include <QPainterPath>
#include <QRegion>
#include <QResizeEvent>
#include <QShowEvent>
#include <QSizePolicy>
#include <QSvgRenderer>
#include <QToolButton>
#include <QHash>
#include <QVBoxLayout>

#include <cmath>

#include "autoviz/ui/theme/glass.hpp"
#include "autoviz/ui/theme/style.hpp"

namespace autoviz {

namespace {

constexpr int kLogicalIcon = 20;
constexpr int kButton = 36;

const glass::OverlayTokens& O() {
  static const glass::OverlayTokens tokens = glass::Overlay();
  return tokens;
}

QImage TintStraightAlpha(QImage image, const QColor& color) {
  image = image.convertToFormat(QImage::Format_ARGB32);
  for (int y = 0; y < image.height(); ++y) {
    QRgb* line = reinterpret_cast<QRgb*>(image.scanLine(y));
    for (int x = 0; x < image.width(); ++x) {
      const int alpha = qAlpha(line[x]);
      if (alpha <= 10) {
        line[x] = qRgba(0, 0, 0, 0);
      } else {
        line[x] = qRgba(color.red(), color.green(), color.blue(), alpha);
      }
    }
  }
  return image;
}

QIcon LoadViewportStrokeIcon(const QString& resource_path) {
  QFile file(resource_path);
  if (!file.open(QIODevice::ReadOnly)) {
    return {};
  }
  const QByteArray bytes = file.readAll();
  QSvgRenderer renderer(bytes);
  if (!renderer.isValid()) {
    return {};
  }

  QIcon icon;
  const qreal dpr = qMax<qreal>(1.0, qApp ? qApp->devicePixelRatio() : 1.0);
  // Pad strokes so AA edges are not clipped.
  constexpr qreal kPad = 1.5;
  for (qreal scale : {dpr, dpr * 2.0}) {
    const int logical = kLogicalIcon;
    const int px =
        qMax(1, static_cast<int>(std::lround((logical + kPad * 2.0) * scale)));
    QImage image(px, px, QImage::Format_ARGB32_Premultiplied);
    image.fill(Qt::transparent);
    {
      QPainter painter(&image);
      painter.setRenderHint(QPainter::Antialiasing, true);
      painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
      const qreal inset = kPad * scale;
      renderer.render(&painter,
                      QRectF(inset, inset, px - inset * 2.0, px - inset * 2.0));
    }

    auto make_pm = [&](const QColor& color) {
      QPixmap pm(QPixmap::fromImage(TintStraightAlpha(image, color)));
      pm.setDevicePixelRatio(scale);
      return pm;
    };

    const glass::OverlayTokens& g = O();
    const QPixmap normal = make_pm(g.icon);
    const QPixmap hover = make_pm(g.icon_hover);
    const QPixmap on = make_pm(g.icon_on);

    icon.addPixmap(normal, QIcon::Normal, QIcon::Off);
    icon.addPixmap(hover, QIcon::Active, QIcon::Off);
    icon.addPixmap(on, QIcon::Normal, QIcon::On);
    icon.addPixmap(on, QIcon::Active, QIcon::On);
    icon.addPixmap(on, QIcon::Selected, QIcon::On);
  }
  return icon;
}

class GlassToolColumn : public QFrame {
 public:
  explicit GlassToolColumn(QWidget* parent = nullptr) : QFrame(parent) {
    setObjectName(QStringLiteral("AutovizViewportToolGroup"));
    setAttribute(Qt::WA_TranslucentBackground, true);
    const glass::OverlayTokens& g = O();
    QHash<QString, QString> tokens;
    tokens.insert(QStringLiteral("{{icon-color}}"),
                  glass::Rgba(QColor(g.icon.red(), g.icon.green(), g.icon.blue(), 210)));
    tokens.insert(QStringLiteral("{{hover-bg}}"), glass::Rgba(g.button_hover_bg));
    tokens.insert(QStringLiteral("{{hover-border}}"), glass::Rgba(g.button_hover_border));
    tokens.insert(QStringLiteral("{{on-bg}}"), glass::Rgba(g.button_on_bg));
    tokens.insert(QStringLiteral("{{on-border}}"), glass::Rgba(g.button_on_border));
    tokens.insert(QStringLiteral("{{on-color}}"),
                  glass::Rgba(QColor(g.accent_soft.red(), g.accent_soft.green(),
                                     g.accent_soft.blue(), 245)));
    tokens.insert(QStringLiteral("{{pressed-bg}}"), glass::Rgba(QColor(56, 189, 248, 40)));
    setStyleSheet(style::sheet(QStringLiteral("chrome/viewport"), tokens));
  }

 protected:
  void paintEvent(QPaintEvent* /*event*/) override {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    const glass::OverlayTokens& g = O();
    const qreal radius = static_cast<qreal>(g.toolbar_radius);
    const QRectF card = QRectF(rect()).adjusted(1.0, 1.0, -1.0, -1.0);
    QPainterPath path;
    path.addRoundedRect(card, radius, radius);

    for (int i = 3; i >= 1; --i) {
      QPainterPath glow;
      glow.addRoundedRect(card.adjusted(-i, -i, i, i), radius + i, radius + i);
      QColor glow_c = g.accent;
      glow_c.setAlpha(3 * (4 - i));
      painter.fillPath(glow, glow_c);
    }

    painter.fillPath(path, g.fill_deep);
    QColor mid_fill = g.fill;
    mid_fill.setAlpha(10);
    QColor end_fill = g.accent;
    end_fill.setAlpha(10);
    QColor start_cool = g.cool_a;
    start_cool.setAlpha(18);
    QLinearGradient body(card.topLeft(), card.bottomRight());
    body.setColorAt(0.0, start_cool);
    body.setColorAt(0.55, mid_fill);
    body.setColorAt(1.0, end_fill);
    painter.fillPath(path, body);

    QColor sheen_end = g.sheen;
    sheen_end.setAlpha(0);
    QColor sheen_top = g.sheen;
    sheen_top.setAlpha(20);
    QLinearGradient sheen(card.topLeft(),
                          QPointF(card.left(), card.top() + 36.0));
    sheen.setColorAt(0.0, sheen_top);
    sheen.setColorAt(1.0, sheen_end);
    painter.fillPath(path, sheen);

    QColor rim = g.rim;
    rim.setAlpha(55);
    painter.setPen(QPen(rim, 1.0));
    painter.drawPath(path);
  }
};

QWidget* MakeDivider(QWidget* parent) {
  auto* wrap = new QWidget(parent);
  wrap->setFixedHeight(12);
  auto* layout = new QVBoxLayout(wrap);
  layout->setContentsMargins(12, 5, 12, 5);
  layout->setSpacing(0);
  auto* line = new QWidget(wrap);
  line->setFixedHeight(1);
  const QColor div = O().divider;
  QHash<QString, QString> tokens;
  tokens.insert(QStringLiteral("{{div-r}}"), QString::number(div.red()));
  tokens.insert(QStringLiteral("{{div-g}}"), QString::number(div.green()));
  tokens.insert(QStringLiteral("{{div-b}}"), QString::number(div.blue()));
  tokens.insert(QStringLiteral("{{div-mid}}"), glass::Rgba(div));
  line->setStyleSheet(style::sheet(QStringLiteral("chrome/viewport"), tokens));
  layout->addWidget(line);
  return wrap;
}

}  // namespace

ViewportFloatingToolbar::ViewportFloatingToolbar(QWidget* parent)
    : QWidget(parent) {
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  setAttribute(Qt::WA_TransparentForMouseEvents, false);
  setAttribute(Qt::WA_AlwaysStackOnTop, true);
  setAttribute(Qt::WA_TranslucentBackground, true);

  auto* outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  outer->setSpacing(0);

  auto* group = new GlassToolColumn(this);
  auto* layout = new QVBoxLayout(group);
  layout->setContentsMargins(6, 8, 6, 8);
  layout->setSpacing(3);

  inspect_button_ = MakeToolButton(
      LoadViewportStrokeIcon(QStringLiteral(":/autoviz/icons/viewport/inspect.svg")),
      tr("Inspect object"), true);
  layout->addWidget(inspect_button_, 0, Qt::AlignHCenter);
  connect(inspect_button_, &QToolButton::clicked, this, [this]() {
    if (callbacks_.on_inspect) {
      callbacks_.on_inspect();
    }
  });

  camera_2d_button_ =
      MakeTextToolButton(QStringLiteral("3D"), tr("Switch to 2D camera"), true);
  layout->addWidget(camera_2d_button_, 0, Qt::AlignHCenter);
  connect(camera_2d_button_, &QToolButton::clicked, this, [this]() {
    if (callbacks_.on_toggle_2d_camera) {
      callbacks_.on_toggle_2d_camera();
    }
  });

  measure_button_ = MakeToolButton(
      LoadViewportStrokeIcon(QStringLiteral(":/autoviz/icons/viewport/measure.svg")),
      tr("Measure distance"), true);
  layout->addWidget(measure_button_, 0, Qt::AlignHCenter);
  connect(measure_button_, &QToolButton::clicked, this, [this]() {
    if (callbacks_.on_measure) {
      callbacks_.on_measure();
    }
  });

  layout->addWidget(MakeDivider(group));

  recenter_button_ = MakeToolButton(
      LoadViewportStrokeIcon(
          QStringLiteral(":/autoviz/icons/viewport/recenter.svg")),
      tr("Re-center on frame"));
  layout->addWidget(recenter_button_, 0, Qt::AlignHCenter);
  connect(recenter_button_, &QToolButton::clicked, this, [this]() {
    if (callbacks_.on_recenter_frame) {
      callbacks_.on_recenter_frame();
    }
  });

  hud_button_ = MakeToolButton(
      LoadViewportStrokeIcon(QStringLiteral(":/autoviz/icons/viewport/gauge.svg")),
      tr("Show telemetry HUD"), true);
  hud_button_->setChecked(true);
  layout->addWidget(hud_button_, 0, Qt::AlignHCenter);
  connect(hud_button_, &QToolButton::toggled, this, [this](bool checked) {
    if (callbacks_.on_toggle_hud) {
      callbacks_.on_toggle_hud(checked);
    }
  });

  outer->addWidget(group);
  adjustSize();
  updateClickThroughMask();
}

void ViewportFloatingToolbar::showEvent(QShowEvent* event) {
  QWidget::showEvent(event);
  adjustSize();
  updateClickThroughMask();
}

void ViewportFloatingToolbar::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);
  updateClickThroughMask();
}

void ViewportFloatingToolbar::updateClickThroughMask() {
  QRegion region;
  const QList<QFrame*> groups = findChildren<QFrame*>(
      QStringLiteral("AutovizViewportToolGroup"));
  for (QFrame* group : groups) {
    if (group != nullptr && group->isVisible()) {
      // Inflate slightly so soft glow is not clipped by the click mask.
      region += group->geometry().adjusted(-2, -2, 2, 2);
    }
  }
  if (region.isEmpty()) {
    clearMask();
  } else {
    setMask(region);
  }
}

void ViewportFloatingToolbar::setCallbacks(
    ViewportFloatingToolbarCallbacks callbacks) {
  callbacks_ = std::move(callbacks);
}

void ViewportFloatingToolbar::setInspectChecked(bool checked) {
  if (inspect_button_ != nullptr) {
    inspect_button_->blockSignals(true);
    inspect_button_->setChecked(checked);
    inspect_button_->blockSignals(false);
  }
}

void ViewportFloatingToolbar::setMeasureChecked(bool checked) {
  if (measure_button_ != nullptr) {
    measure_button_->blockSignals(true);
    measure_button_->setChecked(checked);
    measure_button_->blockSignals(false);
  }
}

void ViewportFloatingToolbar::set2dCameraChecked(bool checked) {
  if (camera_2d_button_ == nullptr) {
    return;
  }
  camera_2d_button_->blockSignals(true);
  camera_2d_button_->setChecked(checked);
  camera_2d_button_->setText(checked ? QStringLiteral("2D")
                                     : QStringLiteral("3D"));
  camera_2d_button_->setToolTip(checked ? tr("Switch to 3D camera")
                                        : tr("Switch to 2D camera"));
  camera_2d_button_->blockSignals(false);
}

void ViewportFloatingToolbar::setRecenterToolTip(const QString& tip) {
  if (recenter_button_ != nullptr) {
    recenter_button_->setToolTip(tip);
  }
}

void ViewportFloatingToolbar::setHudChecked(bool checked) {
  if (hud_button_ != nullptr) {
    hud_button_->blockSignals(true);
    hud_button_->setChecked(checked);
    hud_button_->setToolTip(checked ? tr("Hide telemetry HUD")
                                    : tr("Show telemetry HUD"));
    hud_button_->blockSignals(false);
  }
}

QToolButton* ViewportFloatingToolbar::MakeToolButton(const QIcon& icon,
                                                     const QString& tip,
                                                     bool checkable) {
  auto* button = new QToolButton(this);
  button->setIcon(icon);
  button->setIconSize(QSize(kLogicalIcon, kLogicalIcon));
  button->setAutoRaise(true);
  button->setCursor(Qt::PointingHandCursor);
  button->setToolTip(tip);
  button->setCheckable(checkable);
  button->setFixedSize(QSize(kButton, kButton));
  button->setFocusPolicy(Qt::NoFocus);
  return button;
}

QToolButton* ViewportFloatingToolbar::MakeTextToolButton(const QString& text,
                                                         const QString& tip,
                                                         bool checkable) {
  auto* button = MakeToolButton({}, tip, checkable);
  button->setText(text);
  button->setToolButtonStyle(Qt::ToolButtonTextOnly);
  QFont font = button->font();
  font.setBold(true);
  font.setPixelSize(11);
  font.setLetterSpacing(QFont::AbsoluteSpacing, 0.6);
  button->setFont(font);
  return button;
}

}  // namespace autoviz
