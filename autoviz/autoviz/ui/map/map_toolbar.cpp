/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/map/map_toolbar.hpp"

#include <QApplication>
#include <QFile>
#include <QFrame>
#include <QHash>
#include <QImage>
#include <QPainter>
#include <QPainterPath>
#include <QRegion>
#include <QResizeEvent>
#include <QShowEvent>
#include <QSizePolicy>
#include <QSvgRenderer>
#include <QToolButton>
#include <QVBoxLayout>

#include <cmath>

#include "autoviz/ui/theme/glass.hpp"
#include "autoviz/ui/theme/style.hpp"

namespace autoviz {
namespace map {
namespace {

constexpr int kLogicalIcon = 18;
constexpr int kButton = 32;

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

QIcon LoadStrokeIcon(const QString& resource_path) {
  const qreal dpr = qMax<qreal>(1.0, qApp ? qApp->devicePixelRatio() : 1.0);
  constexpr qreal kPad = 1.5;
  QIcon icon;

  auto add_tinted = [&](const QImage& base) {
    if (base.isNull()) {
      return;
    }
    for (qreal scale : {dpr, dpr * 2.0}) {
      const int px =
          qMax(1, static_cast<int>(std::lround((kLogicalIcon + kPad * 2.0) * scale)));
      QImage image = base.scaled(px, px, Qt::KeepAspectRatio, Qt::SmoothTransformation)
                         .convertToFormat(QImage::Format_ARGB32_Premultiplied);
      auto make_pm = [&](const QColor& color) {
        QPixmap pm(QPixmap::fromImage(TintStraightAlpha(image, color)));
        pm.setDevicePixelRatio(scale);
        return pm;
      };
      const glass::OverlayTokens& g = O();
      icon.addPixmap(make_pm(g.icon), QIcon::Normal, QIcon::Off);
      icon.addPixmap(make_pm(g.icon_hover), QIcon::Active, QIcon::Off);
      icon.addPixmap(make_pm(g.icon_on), QIcon::Normal, QIcon::On);
      icon.addPixmap(make_pm(g.icon_on), QIcon::Active, QIcon::On);
    }
  };

  if (resource_path.endsWith(QStringLiteral(".svg"), Qt::CaseInsensitive)) {
    QFile file(resource_path);
    if (file.open(QIODevice::ReadOnly)) {
      QSvgRenderer renderer(file.readAll());
      if (renderer.isValid()) {
        const int px = qMax(1, static_cast<int>(std::lround((kLogicalIcon + kPad * 2.0) *
                                                              dpr * 2.0)));
        QImage image(px, px, QImage::Format_ARGB32_Premultiplied);
        image.fill(Qt::transparent);
        QPainter painter(&image);
        painter.setRenderHint(QPainter::Antialiasing, true);
        renderer.render(&painter, QRectF(image.rect()).adjusted(2, 2, -2, -2));
        add_tinted(image);
      }
    }
  } else {
    QImage image(resource_path);
    add_tinted(image);
  }
  return icon;
}

class GlassToolColumn : public QFrame {
 public:
  explicit GlassToolColumn(QWidget* parent = nullptr) : QFrame(parent) {
    setObjectName(QStringLiteral("AutovizMapToolGroup"));
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
    painter.fillPath(path, g.fill_deep);
    QColor mid_fill = g.fill;
    mid_fill.setAlpha(10);
    QColor end_fill = g.accent;
    end_fill.setAlpha(10);
    QLinearGradient body(card.topLeft(), card.bottomRight());
    body.setColorAt(0.0, g.cool_a);
    body.setColorAt(0.55, mid_fill);
    body.setColorAt(1.0, end_fill);
    painter.fillPath(path, body);
    QColor rim = g.rim;
    rim.setAlpha(55);
    painter.setPen(QPen(rim, 1.0));
    painter.drawPath(path);
  }
};

QWidget* MakeDivider(QWidget* parent) {
  auto* wrap = new QWidget(parent);
  wrap->setFixedHeight(10);
  auto* layout = new QVBoxLayout(wrap);
  layout->setContentsMargins(10, 4, 10, 4);
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

MapToolbar::MapToolbar(QWidget* parent) : QWidget(parent) {
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  setAttribute(Qt::WA_TransparentForMouseEvents, false);
  setAttribute(Qt::WA_AlwaysStackOnTop, true);
  setAttribute(Qt::WA_TranslucentBackground, true);

  auto* outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  outer->setSpacing(0);

  auto* group = new GlassToolColumn(this);
  auto* layout = new QVBoxLayout(group);
  layout->setContentsMargins(5, 7, 5, 7);
  layout->setSpacing(2);

  pan_button_ = MakeToolButton(
      LoadStrokeIcon(QStringLiteral(":/autoviz/icons/plot/pan.svg")),
      tr("Pan map"), true);
  waypoint_button_ = MakeToolButton(
      LoadStrokeIcon(QStringLiteral(":/autoviz/icons/classes/PublishPoint.svg")),
      tr("Add waypoint"), true);
  geofence_button_ = MakeToolButton(
      LoadStrokeIcon(QStringLiteral(":/autoviz/icons/plot/brush.svg")),
      tr("Add geofence vertex"), true);
  rally_button_ = MakeToolButton(
      LoadStrokeIcon(QStringLiteral(":/autoviz/icons/tool/crosshair.svg")),
      tr("Add rally point"), true);
  measure_button_ = MakeToolButton(
      LoadStrokeIcon(QStringLiteral(":/autoviz/icons/viewport/measure.svg")),
      tr("Measure distance"), true);
  recenter_button_ = MakeToolButton(
      LoadStrokeIcon(QStringLiteral(":/autoviz/icons/viewport/recenter.svg")),
      tr("Fit / center"), false);

  for (QToolButton* button :
       {pan_button_, waypoint_button_, geofence_button_, rally_button_}) {
    layout->addWidget(button, 0, Qt::AlignHCenter);
  }
  layout->addWidget(MakeDivider(group));
  layout->addWidget(measure_button_, 0, Qt::AlignHCenter);
  layout->addWidget(recenter_button_, 0, Qt::AlignHCenter);

  connect(pan_button_, &QToolButton::clicked, this, [this]() {
    if (callbacks_.on_edit_tool) {
      callbacks_.on_edit_tool(MapEditTool::kPan);
    }
  });
  connect(waypoint_button_, &QToolButton::clicked, this, [this]() {
    if (callbacks_.on_edit_tool) {
      callbacks_.on_edit_tool(MapEditTool::kWaypoint);
    }
  });
  connect(geofence_button_, &QToolButton::clicked, this, [this]() {
    if (callbacks_.on_edit_tool) {
      callbacks_.on_edit_tool(MapEditTool::kGeofence);
    }
  });
  connect(rally_button_, &QToolButton::clicked, this, [this]() {
    if (callbacks_.on_edit_tool) {
      callbacks_.on_edit_tool(MapEditTool::kRally);
    }
  });
  connect(measure_button_, &QToolButton::clicked, this, [this]() {
    if (callbacks_.on_measure) {
      callbacks_.on_measure();
    }
  });
  connect(recenter_button_, &QToolButton::clicked, this, [this]() {
    if (callbacks_.on_recenter) {
      callbacks_.on_recenter();
    }
  });

  outer->addWidget(group);
  syncEditButtons();
  adjustSize();
  updateClickThroughMask();
}

void MapToolbar::setCallbacks(MapToolbarCallbacks callbacks) {
  callbacks_ = std::move(callbacks);
}

void MapToolbar::setEditTool(MapEditTool tool) {
  edit_tool_ = tool;
  syncEditButtons();
}

void MapToolbar::setMeasureChecked(bool checked) {
  if (measure_button_ != nullptr) {
    measure_button_->setChecked(checked);
  }
}

void MapToolbar::setRecenterToolTip(const QString& tip) {
  if (recenter_button_ != nullptr) {
    recenter_button_->setToolTip(tip);
  }
}

void MapToolbar::showEvent(QShowEvent* event) {
  QWidget::showEvent(event);
  updateClickThroughMask();
}

void MapToolbar::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);
  updateClickThroughMask();
}

QToolButton* MapToolbar::MakeToolButton(const QIcon& icon, const QString& tip,
                                        bool checkable) {
  auto* button = new QToolButton(this);
  button->setIcon(icon);
  button->setIconSize(QSize(kLogicalIcon, kLogicalIcon));
  button->setFixedSize(kButton, kButton);
  button->setAutoRaise(true);
  button->setCheckable(checkable);
  button->setCursor(Qt::PointingHandCursor);
  button->setToolTip(tip);
  button->setFocusPolicy(Qt::NoFocus);
  return button;
}

void MapToolbar::updateClickThroughMask() {
  QRegion mask;
  for (QObject* child : children()) {
    auto* widget = qobject_cast<QWidget*>(child);
    if (widget != nullptr && widget->isVisible()) {
      mask = mask.united(widget->geometry());
    }
  }
  setMask(mask);
}

void MapToolbar::syncEditButtons() {
  if (pan_button_ != nullptr) {
    pan_button_->setChecked(edit_tool_ == MapEditTool::kPan);
  }
  if (waypoint_button_ != nullptr) {
    waypoint_button_->setChecked(edit_tool_ == MapEditTool::kWaypoint);
  }
  if (geofence_button_ != nullptr) {
    geofence_button_->setChecked(edit_tool_ == MapEditTool::kGeofence);
  }
  if (rally_button_ != nullptr) {
    rally_button_->setChecked(edit_tool_ == MapEditTool::kRally);
  }
}

}  // namespace map
}  // namespace autoviz
