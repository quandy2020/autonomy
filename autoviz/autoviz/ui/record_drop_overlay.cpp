/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/record_drop_overlay.hpp"

#include <QFont>
#include <QPainter>
#include <QPaintEvent>
#include <QPen>

namespace autoviz {

RecordDropOverlay::RecordDropOverlay(QWidget* parent) : QWidget(parent) {
  setAttribute(Qt::WA_TransparentForMouseEvents, true);
  setAttribute(Qt::WA_NoSystemBackground, false);
  setAttribute(Qt::WA_TranslucentBackground, false);
  setAutoFillBackground(true);
  hide();
}

void RecordDropOverlay::paintEvent(QPaintEvent*) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.fillRect(rect(), QColor(0x1C, 0x1C, 0x1E));
  QPen pen(QColor(0xA8, 0xC9, 0x4A), 3, Qt::DashLine);
  painter.setPen(pen);
  painter.setBrush(Qt::NoBrush);
  painter.drawRoundedRect(rect().adjusted(18, 18, -18, -18), 8, 8);
  QFont title = font();
  title.setPointSize(20);
  title.setBold(true);
  painter.setFont(title);
  painter.setPen(QColor(0xF2, 0xF2, 0xF7));
  painter.drawText(rect().adjusted(0, -24, 0, 0), Qt::AlignCenter,
                   tr("Drop record to play"));
  QFont hint = font();
  hint.setPointSize(12);
  painter.setFont(hint);
  painter.setPen(QColor(0xAE, 0xAE, 0xB2));
  painter.drawText(rect().adjusted(0, 28, 0, 0), Qt::AlignCenter,
                   tr("Autolink .record  ·  .bag  ·  .mcap"));
}

}  // namespace autoviz
