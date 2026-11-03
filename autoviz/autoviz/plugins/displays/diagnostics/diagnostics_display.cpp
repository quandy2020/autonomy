/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#include "autoviz/plugins/displays/diagnostics/diagnostics_display.hpp"

#include <algorithm>
#include <utility>

#include <QFont>
#include <QPainter>

namespace autoviz {
namespace display {
namespace {

QColor LevelColor(int level) {
  using Status = automsgs::msgs::diagnostic_msgs::DiagnosticStatus;
  switch (level) {
    case Status::OK:
      return QColor(80, 220, 120);
    case Status::WARN:
      return QColor(240, 200, 40);
    case Status::ERROR:
      return QColor(240, 70, 70);
    case Status::STALE:
      return QColor(160, 160, 160);
    default:
      return QColor(200, 200, 200);
  }
}

const char* LevelLabel(int level) {
  using Status = automsgs::msgs::diagnostic_msgs::DiagnosticStatus;
  switch (level) {
    case Status::OK:
      return "OK";
    case Status::WARN:
      return "WARN";
    case Status::ERROR:
      return "ERROR";
    case Status::STALE:
      return "STALE";
    default:
      return "?";
  }
}

}  // namespace

OverlayDiagnosticsDisplay::OverlayDiagnosticsDisplay(std::string channel)
    : ChannelDisplay<automsgs::msgs::diagnostic_msgs::DiagnosticArray>(
          "OverlayDiagnostics", std::move(channel),
          "automsgs.msgs.diagnostic_msgs.DiagnosticArray") {
  setProperties({});
}

std::vector<common::DisplayPropertySpec>
OverlayDiagnosticsDisplay::propertySpecs() const {
  return {{"left", "Left", "20"},
          {"top", "Top", "20"},
          {"width", "Width", "480"},
          {"height", "Height", "260"},
          {"text_size", "Text Size", "12"},
          {"max_lines", "Max Lines", "16"},
          {"bg_alpha", "Background Alpha", "0.75"},
          {"show_hardware_id", "Show Hardware Id", "false"}};
}

void OverlayDiagnosticsDisplay::ensureOverlay() {
  if (!overlay_) {
    overlay_ = std::make_shared<viz::OverlayObject>("OverlayDiagnostics/" +
                                                    name() + "/" + channel());
  }
}

void OverlayDiagnosticsDisplay::onEnable() {
  ChannelDisplay::onEnable();
  ensureOverlay();
  if (overlay_) {
    overlay_->show();
  }
  require_update_texture_ = true;
}

void OverlayDiagnosticsDisplay::onDisable() {
  if (overlay_) {
    overlay_->hide();
  }
  ChannelDisplay::onDisable();
}

void OverlayDiagnosticsDisplay::clearReceivedData() {
  lines_.clear();
  require_update_texture_ = true;
}

void OverlayDiagnosticsDisplay::processMessage(
    const automsgs::msgs::diagnostic_msgs::DiagnosticArray& message) {
  const int max_lines =
      common::ParseIntProperty(propertyValue("max_lines", "16"), 16);
  const bool show_hardware_id =
      propertyValue("show_hardware_id", "false") == "true";
  lines_.clear();
  for (int i = 0; i < message.status_size() &&
                  static_cast<int>(lines_.size()) < max_lines;
       ++i) {
    const auto& status = message.status(i);
    Line line;
    line.color = LevelColor(status.level());
    if (show_hardware_id && !status.hardware_id().empty()) {
      line.text = QString("[%1] %2 (%3): %4")
                      .arg(LevelLabel(status.level()))
                      .arg(QString::fromStdString(status.name()))
                      .arg(QString::fromStdString(status.hardware_id()))
                      .arg(QString::fromStdString(status.message()));
    } else {
      line.text = QString("[%1] %2: %3")
                      .arg(LevelLabel(status.level()))
                      .arg(QString::fromStdString(status.name()))
                      .arg(QString::fromStdString(status.message()));
    }
    lines_.push_back(std::move(line));
  }
  require_update_texture_ = true;
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void OverlayDiagnosticsDisplay::refreshTexture() {
  ensureOverlay();
  if (!overlay_ || !require_update_texture_) {
    return;
  }
  require_update_texture_ = false;

  const int left = common::ParseIntProperty(propertyValue("left", "20"), 20);
  const int top = common::ParseIntProperty(propertyValue("top", "20"), 20);
  const int width =
      common::ParseIntProperty(propertyValue("width", "480"), 480);
  const int height =
      common::ParseIntProperty(propertyValue("height", "260"), 260);
  const int text_size =
      common::ParseIntProperty(propertyValue("text_size", "12"), 12);
  QColor bg(0, 0, 0);
  bg.setAlphaF(
      common::ParseFloatProperty(propertyValue("bg_alpha", "0.75"), 0.75f));

  overlay_->updateTextureSize(static_cast<unsigned>(std::max(1, width)),
                              static_cast<unsigned>(std::max(1, height)));
  {
    viz::ScopedPixelBuffer buffer = overlay_->getBuffer();
    QImage hud = buffer.getQImage(overlay_->getTextureWidth(),
                                  overlay_->getTextureHeight(), bg);
    if (!hud.isNull()) {
      QPainter painter(&hud);
      painter.setRenderHint(QPainter::TextAntialiasing, true);
      QFont font(QStringLiteral("DejaVu Sans Mono"));
      font.setPointSize(std::max(1, text_size));
      painter.setFont(font);
      int y = 4 + text_size;
      for (const Line& line : lines_) {
        painter.setPen(line.color);
        painter.drawText(8, y, line.text);
        y += text_size + 4;
        if (y > hud.height() - 4) {
          break;
        }
      }
    }
  }
  overlay_->setDimensions(overlay_->getTextureWidth(),
                          overlay_->getTextureHeight());
  overlay_->setPosition(left, top);
  overlay_->show();
}

void OverlayDiagnosticsDisplay::onDraw(rendering::SceneOverlay& /*scene*/) {
  refreshTexture();
}

}  // namespace display
}  // namespace autoviz
