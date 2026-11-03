/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from gps_rviz_plugin OverlayGpsDisplay (BSD).
 *****************************************************************************/

#include "autoviz/plugins/displays/gps/overlay_gps_display.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

#include <QEventLoop>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QPainter>
#include <QTimer>
#include <QUrl>

namespace autoviz {
namespace display {
namespace {

constexpr int kMaxRequestUrlLength = 8192;

std::string MapTypeQuery(const std::string& map_type) {
  if (map_type == "terrain") {
    return "terrain";
  }
  if (map_type == "satellite") {
    return "satellite";
  }
  if (map_type == "hybrid") {
    return "hybrid";
  }
  return "roadmap";
}

}  // namespace

OverlayGpsDisplay::OverlayGpsDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::sensor_msgs::NavSatFix>(
          "OverlayGps", std::move(channel),
          "automsgs.msgs.sensor_msgs.NavSatFix") {
  setProperties({});
  resizeHistory(15);
}

std::vector<common::DisplayPropertySpec>
OverlayGpsDisplay::propertySpecs() const {
  return {
      {"api_key", "API Key", ""},
      {"zoom", "Zoom", "19"},
      {"width", "Width", "320"},
      {"height", "Height", "320"},
      {"scale", "Scale", "1"},
      {"map_type", "Map Type", "roadmap",
       {"roadmap", "terrain", "satellite", "hybrid"}},
      {"history_length", "History Length", "15"},
      {"messages_per_plot", "Messages Per Plot", "5"},
      {"left", "Left", "0"},
      {"top", "Top", "0"},
      {"alpha", "Alpha", "0.8"},
  };
}

void OverlayGpsDisplay::ensureOverlay() {
  if (!overlay_) {
    overlay_ = std::make_shared<viz::OverlayObject>(
        "OverlayGps/" + name() + "/" + channel());
  }
}

void OverlayGpsDisplay::onEnable() {
  ChannelDisplay::onEnable();
  ensureOverlay();
  if (overlay_) {
    overlay_->show();
  }
}

void OverlayGpsDisplay::onDisable() {
  if (overlay_) {
    overlay_->hide();
  }
  ChannelDisplay::onDisable();
}

void OverlayGpsDisplay::clearReceivedData() {
  map_image_ = QImage();
  fix_history_.clear();
  message_count_ = 0;
  require_update_texture_ = true;
}

void OverlayGpsDisplay::resizeHistory(std::size_t length) {
  const std::size_t capped = std::max<std::size_t>(1, length);
  while (fix_history_.size() > capped) {
    fix_history_.pop_front();
  }
}

bool OverlayGpsDisplay::buildRequestUrl(const FixSample& center,
                                        std::string* request_url) {
  if (request_url == nullptr) {
    return false;
  }

  const std::string api_key = propertyValue("api_key", "");
  if (api_key.empty()) {
    setStatusError("Google Static Map API key is empty");
    return false;
  }

  const int zoom =
      std::clamp(common::ParseIntProperty(propertyValue("zoom", "19"), 19), 0,
                 22);
  const int width = std::clamp(
      common::ParseIntProperty(propertyValue("width", "320"), 320), 1, 640);
  const int height = std::clamp(
      common::ParseIntProperty(propertyValue("height", "320"), 320), 1, 640);
  const int scale = std::clamp(
      common::ParseIntProperty(propertyValue("scale", "1"), 1), 1, 2);
  const std::string map_type =
      MapTypeQuery(propertyValue("map_type", "roadmap"));

  std::ostringstream url;
  url.setf(std::ios::fixed);
  url.precision(8);
  url << "https://maps.googleapis.com/maps/api/staticmap?"
      << "center=" << center.latitude << ',' << center.longitude
      << "&markers=color:red%7C" << center.latitude << ',' << center.longitude
      << "&zoom=" << zoom << "&size=" << width << 'x' << height
      << "&scale=" << scale << "&maptype=" << map_type
      << "&path=color:blue|weight:5";
  for (auto it = fix_history_.rbegin(); it != fix_history_.rend(); ++it) {
    url << '|' << it->latitude << ',' << it->longitude;
  }
  url << "&key=" << api_key;

  *request_url = url.str();
  if (request_url->size() > static_cast<std::size_t>(kMaxRequestUrlLength)) {
    setStatusError("Static Map request URL is too long (" +
                   std::to_string(request_url->size()) + " chars)");
    return false;
  }
  return true;
}

bool OverlayGpsDisplay::downloadMap(const std::string& request_url,
                                    QImage* out_image) {
  if (out_image == nullptr) {
    return false;
  }

  QNetworkAccessManager manager;
  QEventLoop loop;
  QTimer timeout;
  timeout.setSingleShot(true);
  QObject::connect(&timeout, &QTimer::timeout, &loop, &QEventLoop::quit);

  QNetworkRequest request{QUrl(QString::fromStdString(request_url))};
  request.setHeader(QNetworkRequest::UserAgentHeader,
                    QStringLiteral("autoviz-OverlayGps/1.0"));
  QNetworkReply* reply = manager.get(request);
  QObject::connect(reply, &QNetworkReply::finished, &loop, &QEventLoop::quit);
  timeout.start(10000);
  loop.exec();

  if (!reply->isFinished()) {
    reply->abort();
    reply->deleteLater();
    setStatusError("Static Map download timed out");
    return false;
  }
  if (reply->error() != QNetworkReply::NoError) {
    const std::string err = reply->errorString().toStdString();
    reply->deleteLater();
    setStatusError("Static Map download failed: " + err);
    return false;
  }

  const QByteArray bytes = reply->readAll();
  reply->deleteLater();
  QImage image;
  if (!image.loadFromData(bytes)) {
    setStatusError("Static Map response is not a valid image (check API key)");
    return false;
  }
  if (image.width() <= 0 || image.height() <= 0) {
    setStatusError("Static Map image has invalid size");
    return false;
  }
  *out_image = image.convertToFormat(QImage::Format_ARGB32);
  return true;
}

void OverlayGpsDisplay::processMessage(
    const automsgs::msgs::sensor_msgs::NavSatFix& message) {
  ++message_count_;
  const int messages_per_plot = std::max(
      1, common::ParseIntProperty(propertyValue("messages_per_plot", "5"), 5));
  if ((message_count_ % static_cast<std::size_t>(messages_per_plot)) != 0) {
    return;
  }

  const int history_length = std::max(
      1, common::ParseIntProperty(propertyValue("history_length", "15"), 15));
  resizeHistory(static_cast<std::size_t>(history_length));

  FixSample sample{message.latitude(), message.longitude()};
  if (!std::isfinite(sample.latitude) || !std::isfinite(sample.longitude)) {
    setStatusWarn("NavSatFix latitude/longitude is not finite");
    return;
  }
  fix_history_.push_back(sample);
  resizeHistory(static_cast<std::size_t>(history_length));

  std::string request_url;
  if (!buildRequestUrl(sample, &request_url)) {
    return;
  }

  QImage image;
  if (!downloadMap(request_url, &image)) {
    return;
  }

  map_image_ = std::move(image);
  require_update_texture_ = true;
  setStatusOk("Static Map updated");
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void OverlayGpsDisplay::refreshTexture() {
  ensureOverlay();
  if (!overlay_ || !require_update_texture_ || map_image_.isNull()) {
    return;
  }
  require_update_texture_ = false;

  const int left = common::ParseIntProperty(propertyValue("left", "0"), 0);
  const int top = common::ParseIntProperty(propertyValue("top", "0"), 0);
  const int width = std::max(
      1, common::ParseIntProperty(propertyValue("width", "320"), 320));
  const int height = std::max(
      1, common::ParseIntProperty(propertyValue("height", "320"), 320));
  const float alpha =
      std::clamp(common::ParseFloatProperty(propertyValue("alpha", "0.8"), 0.8f),
                 0.f, 1.f);

  QImage scaled = map_image_.scaled(width, height, Qt::IgnoreAspectRatio,
                                    Qt::SmoothTransformation);
  if (scaled.format() != QImage::Format_ARGB32) {
    scaled = scaled.convertToFormat(QImage::Format_ARGB32);
  }

  // Apply alpha uniformly onto the map pixels.
  for (int y = 0; y < scaled.height(); ++y) {
    QRgb* line = reinterpret_cast<QRgb*>(scaled.scanLine(y));
    for (int x = 0; x < scaled.width(); ++x) {
      QColor color = QColor::fromRgba(line[x]);
      color.setAlpha(static_cast<int>(alpha * 255.0f));
      line[x] = color.rgba();
    }
  }

  overlay_->updateTextureSize(static_cast<unsigned>(width),
                              static_cast<unsigned>(height));
  {
    viz::ScopedPixelBuffer buffer = overlay_->getBuffer();
    QImage hud = buffer.getQImage(static_cast<unsigned>(width),
                                  static_cast<unsigned>(height));
    QPainter painter(&hud);
    painter.setCompositionMode(QPainter::CompositionMode_Source);
    painter.fillRect(hud.rect(), Qt::transparent);
    painter.drawImage(0, 0, scaled);
    painter.end();
  }
  overlay_->setDimensions(width, height);
  overlay_->setPosition(left, top);
  if (enabled()) {
    overlay_->show();
  }
}

void OverlayGpsDisplay::onDraw(rendering::SceneOverlay& /*scene*/) {
  refreshTexture();
}

}  // namespace display
}  // namespace autoviz
