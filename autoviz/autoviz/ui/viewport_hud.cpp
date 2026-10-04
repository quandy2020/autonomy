/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/viewport_hud.hpp"

#include <QFont>
#include <QHideEvent>
#include <QPainter>
#include <QPainterPath>
#include <QResizeEvent>
#include <QShowEvent>
#include <QTimer>
#include <QtMath>

#include <algorithm>
#include <cmath>

#include <automsgs/msgs/nav_msgs/odometry.pb.h>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/display/display.hpp"
#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/ui/theme/glass.hpp"

namespace autoviz {
namespace {

const glass::OverlayTokens& O() {
  static const glass::OverlayTokens tokens = glass::Overlay();
  return tokens;
}

constexpr int kHudWidth = 78;
constexpr int kHudHeight = 232;
constexpr double kMinSpeedScale = 3.0;
constexpr double kMaxSpeedCap = 40.0;
constexpr double kMinAngularScale = 1.0;
constexpr double kMaxAngularCap = 6.0;

QString FormatSpeed(double mps) {
  if (mps < 10.0) {
    return QString::number(mps, 'f', 2);
  }
  return QString::number(mps, 'f', 1);
}

QString FormatAngular(double rad_s) {
  const double abs_v = std::abs(rad_s);
  if (abs_v < 10.0) {
    return QString::number(rad_s, 'f', 2);
  }
  return QString::number(rad_s, 'f', 1);
}

QString FormatDistance(double meters) {
  if (meters < 1000.0) {
    if (meters < 10.0) {
      return QString::number(meters, 'f', 2);
    }
    if (meters < 100.0) {
      return QString::number(meters, 'f', 1);
    }
    return QString::number(meters, 'f', 0);
  }
  return QString::number(meters / 1000.0, 'f', 2);
}

QString DistanceUnit(double meters) {
  return meters < 1000.0 ? QStringLiteral("m") : QStringLiteral("km");
}

QString PreferOdomChannel(common::VisualizationManager* manager) {
  if (manager != nullptr) {
    for (display::Display* display : manager->displays()) {
      if (display == nullptr || !display->enabled()) {
        continue;
      }
      if (display->typeId() != "Odometry") {
        continue;
      }
      const std::string channel = display->channel();
      if (!channel.empty()) {
        return QString::fromStdString(channel);
      }
    }
  }
  return QStringLiteral("/odom");
}

void DrawDial(QPainter& painter, const QRectF& bounds, double normalized,
              const QColor& accent, const QString& title, const QString& value,
              const QString& unit, bool has_data) {
  const glass::OverlayTokens& g = O();
  const QPointF center(bounds.center().x(), bounds.center().y() + 5.0);
  const double radius =
      std::min({bounds.width() * 0.40, bounds.height() * 0.38, 26.0});
  const QRectF arc(center.x() - radius, center.y() - radius, radius * 2.0,
                   radius * 2.0);

  const int start = 210 * 16;
  const int span = -240 * 16;

  QPen track(g.track, 2.4, Qt::SolidLine, Qt::RoundCap);
  painter.setPen(track);
  painter.setBrush(Qt::NoBrush);
  painter.drawArc(arc, start, span);

  if (has_data) {
    QColor glow = accent;
    glow.setAlpha(50);
    QPen glow_pen(glow, 4.0, Qt::SolidLine, Qt::RoundCap);
    painter.setPen(glow_pen);
    painter.drawArc(arc, start,
                    static_cast<int>(span * std::clamp(normalized, 0.0, 1.0)));

    QColor fill = accent;
    fill.setAlpha(200);
    QPen value_pen(fill, 2.4, Qt::SolidLine, Qt::RoundCap);
    painter.setPen(value_pen);
    painter.drawArc(arc, start,
                    static_cast<int>(span * std::clamp(normalized, 0.0, 1.0)));
  }

  QFont title_font = painter.font();
  title_font.setPixelSize(7);
  title_font.setBold(true);
  title_font.setLetterSpacing(QFont::AbsoluteSpacing, 0.8);
  painter.setFont(title_font);
  painter.setPen(g.title);
  painter.drawText(QRectF(bounds.left(), bounds.top(), bounds.width(), 11.0),
                   Qt::AlignHCenter | Qt::AlignVCenter, title);

  QFont value_font = painter.font();
  value_font.setPixelSize(12);
  value_font.setBold(true);
  painter.setFont(value_font);
  painter.setPen(has_data ? g.value : g.value_idle);
  const QString shown = has_data ? value : QStringLiteral("—");
  painter.drawText(
      QRectF(center.x() - radius, center.y() - radius * 0.52, radius * 2.0,
             radius * 0.48),
      Qt::AlignHCenter | Qt::AlignBottom, shown);

  QFont unit_font = painter.font();
  unit_font.setPixelSize(7);
  unit_font.setBold(false);
  painter.setFont(unit_font);
  painter.setPen(g.unit);
  painter.drawText(
      QRectF(center.x() - radius, center.y() - 1.0, radius * 2.0, 11.0),
      Qt::AlignHCenter | Qt::AlignTop, unit);
}

}  // namespace

ViewportHudOverlay::ViewportHudOverlay(common::VisualizationManager* manager,
                                       QWidget* parent)
    : QWidget(parent), manager_(manager) {
  setFixedSize(kHudWidth, kHudHeight);
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  setAttribute(Qt::WA_TransparentForMouseEvents, true);
  setAttribute(Qt::WA_AlwaysStackOnTop, true);
  setAttribute(Qt::WA_TranslucentBackground, true);

  refresh_timer_ = new QTimer(this);
  refresh_timer_->setInterval(50);
  connect(refresh_timer_, &QTimer::timeout, this,
          &ViewportHudOverlay::applyLatestToUi);

  resolve_timer_ = new QTimer(this);
  resolve_timer_->setInterval(1500);
  connect(resolve_timer_, &QTimer::timeout, this,
          &ViewportHudOverlay::resolveChannelIfNeeded);

  syncActiveState();
}

ViewportHudOverlay::~ViewportHudOverlay() { unsubscribe(); }

void ViewportHudOverlay::setHudVisible(bool visible) {
  if (hud_visible_ == visible) {
    setVisible(visible);
    return;
  }
  hud_visible_ = visible;
  syncActiveState();
}

void ViewportHudOverlay::syncActiveState() {
  if (!hud_visible_) {
    if (refresh_timer_ != nullptr) {
      refresh_timer_->stop();
    }
    if (resolve_timer_ != nullptr) {
      resolve_timer_->stop();
    }
    unsubscribe();
    hide();
    return;
  }

  show();
  raise();
  if (refresh_timer_ != nullptr && !refresh_timer_->isActive()) {
    refresh_timer_->start();
  }
  if (resolve_timer_ != nullptr && !resolve_timer_->isActive()) {
    resolve_timer_->start();
  }
  resolveChannelIfNeeded();
  if (subscription_id_ == 0 && !channel_.isEmpty()) {
    resubscribe();
  }
  update();
}

void ViewportHudOverlay::setChannel(const QString& channel) {
  if (channel_ == channel) {
    return;
  }
  channel_ = channel;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    has_last_pose_ = false;
    distance_m_ = 0.0;
    snapshot_ = Snapshot{};
    snapshot_.max_speed_mps = max_speed_mps_;
    snapshot_.max_angular_rad_s = max_angular_rad_s_;
  }
  if (hud_visible_) {
    resubscribe();
    update();
  }
}

QString ViewportHudOverlay::channel() const { return channel_; }

void ViewportHudOverlay::resolveChannelIfNeeded() {
  if (!hud_visible_) {
    return;
  }
  Snapshot snap = readSnapshot();
  if (snap.has_data && !channel_.isEmpty()) {
    return;
  }
  const QString preferred = PreferOdomChannel(manager_);
  if (preferred.isEmpty()) {
    return;
  }
  // Prefer an Odometry display channel; otherwise keep /odom until data arrives,
  // then also try /fake/odom once.
  if (channel_.isEmpty()) {
    setChannel(preferred);
    return;
  }
  if (!snap.has_data && channel_ == QLatin1String("/odom") &&
      preferred == QLatin1String("/odom")) {
    setChannel(QStringLiteral("/fake/odom"));
  } else if (!snap.has_data && channel_ != preferred &&
             preferred != QLatin1String("/odom")) {
    setChannel(preferred);
  }
}

void ViewportHudOverlay::resubscribe() {
  unsubscribe();
  if (!hud_visible_ || channel_.isEmpty()) {
    return;
  }
  subscription_id_ = integration::ChannelReaderRegistry::instance().subscribe(
      channel_.toStdString(),
      [this](const std::string& payload) { onPayload(payload); });
}

void ViewportHudOverlay::unsubscribe() {
  if (subscription_id_ == 0) {
    return;
  }
  integration::ChannelReaderRegistry::instance().unsubscribe(subscription_id_);
  subscription_id_ = 0;
}

void ViewportHudOverlay::onPayload(const std::string& payload) {
  if (!hud_visible_) {
    return;
  }
  automsgs::msgs::nav_msgs::Odometry msg;
  if (!msg.ParseFromString(payload)) {
    return;
  }

  const auto& linear = msg.twist().twist().linear();
  const double speed =
      std::hypot(linear.x(), std::hypot(linear.y(), linear.z()));

  const auto& angular = msg.twist().twist().angular();
  // Prefer yaw rate; fall back to full angular magnitude if z is idle.
  const double angular_z = angular.z();
  const double angular_mag =
      std::hypot(angular.x(), std::hypot(angular.y(), angular.z()));
  const double angular_rate =
      std::abs(angular_z) >= 1e-6 ? angular_z : angular_mag;

  const auto& position = msg.pose().pose().pose().position();
  const double x = position.x();
  const double y = position.y();

  std::lock_guard<std::mutex> lock(mutex_);
  if (has_last_pose_) {
    const double dx = x - last_x_;
    const double dy = y - last_y_;
    const double step = std::hypot(dx, dy);
    // Ignore teleport / first jump after channel switch.
    if (step > 0.0 && step < 20.0) {
      distance_m_ += step;
    }
  }
  last_x_ = x;
  last_y_ = y;
  has_last_pose_ = true;

  max_speed_mps_ =
      std::clamp(std::max(max_speed_mps_, speed * 1.15), kMinSpeedScale,
                 kMaxSpeedCap);
  max_angular_rad_s_ = std::clamp(
      std::max(max_angular_rad_s_, std::abs(angular_rate) * 1.15),
      kMinAngularScale, kMaxAngularCap);

  snapshot_.has_data = true;
  snapshot_.speed_mps = speed;
  snapshot_.angular_rad_s = angular_rate;
  snapshot_.distance_m = distance_m_;
  snapshot_.max_speed_mps = max_speed_mps_;
  snapshot_.max_angular_rad_s = max_angular_rad_s_;
}

void ViewportHudOverlay::applyLatestToUi() {
  if (hud_visible_) {
    update();
  }
}

ViewportHudOverlay::Snapshot ViewportHudOverlay::readSnapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

void ViewportHudOverlay::paintEvent(QPaintEvent* /*event*/) {
  if (!hud_visible_) {
    return;
  }
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);

  const glass::OverlayTokens& g = O();
  const QRectF card = QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5);
  QPainterPath path;
  path.addRoundedRect(card, static_cast<qreal>(g.radius),
                      static_cast<qreal>(g.radius));

  painter.fillPath(path, g.fill);
  QLinearGradient cool(card.topLeft(), card.bottomRight());
  cool.setColorAt(0.0, g.cool_a);
  cool.setColorAt(1.0, g.cool_b);
  painter.fillPath(path, cool);

  QColor sheen_end = g.sheen;
  sheen_end.setAlpha(0);
  QLinearGradient sheen(card.topLeft(), QPointF(card.left(), card.top() + 28.0));
  sheen.setColorAt(0.0, g.sheen);
  sheen.setColorAt(1.0, sheen_end);
  painter.fillPath(path, sheen);

  painter.setPen(QPen(g.rim, 1.0));
  painter.drawPath(path);

  const Snapshot snap = readSnapshot();
  const double speed_norm =
      snap.max_speed_mps > 1e-6
          ? std::clamp(snap.speed_mps / snap.max_speed_mps, 0.0, 1.0)
          : 0.0;
  const double ang_norm =
      snap.max_angular_rad_s > 1e-6
          ? std::clamp(std::abs(snap.angular_rad_s) / snap.max_angular_rad_s,
                       0.0, 1.0)
          : 0.0;
  const double odo_norm =
      std::fmod(std::max(0.0, snap.distance_m), 1000.0) / 1000.0;

  const QRectF speed_bounds(4.0, 2.0, 70.0, 72.0);
  const QRectF ang_bounds(4.0, 78.0, 70.0, 72.0);
  const QRectF odo_bounds(4.0, 154.0, 70.0, 72.0);

  DrawDial(painter, speed_bounds, speed_norm, g.accent, QStringLiteral("SPEED"),
           FormatSpeed(snap.speed_mps), QStringLiteral("m/s"), snap.has_data);
  DrawDial(painter, ang_bounds, ang_norm, g.accent_mid, QStringLiteral("ANG"),
           FormatAngular(snap.angular_rad_s), QStringLiteral("rad/s"),
           snap.has_data);
  DrawDial(painter, odo_bounds, odo_norm, g.accent_soft, QStringLiteral("ODOM"),
           FormatDistance(snap.distance_m), DistanceUnit(snap.distance_m),
           snap.has_data);
}

void ViewportHudOverlay::showEvent(QShowEvent* event) {
  QWidget::showEvent(event);
  if (hud_visible_) {
    raise();
  }
}

void ViewportHudOverlay::hideEvent(QHideEvent* event) {
  QWidget::hideEvent(event);
}

void ViewportHudOverlay::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);
}

}  // namespace autoviz
