/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/log/log_entry_delegate.hpp"

#include <algorithm>

#include <QPainter>
#include <QStyle>

#include "autoviz/ui/log/log_types.hpp"

namespace autoviz {
namespace log_panel {
namespace {

constexpr int kRoleTimestamp = Qt::UserRole + 1;
constexpr int kRoleLevel = Qt::UserRole + 2;
constexpr int kRoleName = Qt::UserRole + 3;
constexpr int kRoleMessage = Qt::UserRole + 4;

constexpr int kPaddingH = 10;
constexpr int kPaddingV = 3;
constexpr int kColumnGap = 8;
constexpr int kTimeWidth = 88;
constexpr int kLevelWidth = 42;
constexpr int kMinRowHeight = 22;
constexpr int kAccentWidth = 3;

QString ShortLevelLabel(LogLevel level) {
  switch (level) {
    case LogLevel::kDebug:
      return QStringLiteral("DBG");
    case LogLevel::kInfo:
      return QStringLiteral("INF");
    case LogLevel::kWarn:
      return QStringLiteral("WRN");
    case LogLevel::kError:
      return QStringLiteral("ERR");
    case LogLevel::kFatal:
      return QStringLiteral("FTL");
    default:
      return QStringLiteral("???");
  }
}

QColor RowBackground(bool selected, bool hovered, int row) {
  if (selected) {
    return QColor(8, 145, 178, 36);
  }
  if (hovered) {
    return QColor(15, 23, 42, 12);
  }
  return (row % 2 == 0) ? QColor(255, 255, 255) : QColor(248, 250, 252);
}

QColor AccentColor(LogLevel level) {
  switch (level) {
    case LogLevel::kDebug:
      return QColor(100, 116, 139);
    case LogLevel::kInfo:
      return QColor(8, 145, 178);
    case LogLevel::kWarn:
      return QColor(217, 119, 6);
    case LogLevel::kError:
      return QColor(220, 38, 38);
    case LogLevel::kFatal:
      return QColor(190, 24, 93);
    default:
      return QColor(148, 163, 184);
  }
}

QColor LevelColor(LogLevel level) {
  return AccentColor(level);
}

QColor MessageColor(LogLevel level) {
  switch (level) {
    case LogLevel::kDebug:
      return QColor(100, 116, 139);
    case LogLevel::kWarn:
      return QColor(180, 83, 9);
    case LogLevel::kError:
      return QColor(185, 28, 28);
    case LogLevel::kFatal:
      return QColor(157, 23, 77);
    default:
      return QColor(30, 41, 59);
  }
}

QString BuildBodyText(const QString& name, const QString& message) {
  QString normalized = message;
  normalized.replace(QLatin1Char('\r'), QLatin1Char(' '));
  normalized.replace(QLatin1Char('\n'), QLatin1Char(' '));
  normalized = normalized.simplified();
  if (name.isEmpty()) {
    return normalized;
  }
  return QStringLiteral("[%1] %2").arg(name, normalized);
}

}  // namespace

LogEntryDelegate::LogEntryDelegate(QObject* parent)
    : QStyledItemDelegate(parent) {}

void LogEntryDelegate::paint(QPainter* painter, const QStyleOptionViewItem& option,
                             const QModelIndex& index) const {
  if (!index.isValid()) {
    return;
  }

  const LogLevel level =
      static_cast<LogLevel>(index.data(kRoleLevel).toInt());
  const bool selected = option.state & QStyle::State_Selected;
  const bool hovered = option.state & QStyle::State_MouseOver;

  painter->save();
  painter->setRenderHint(QPainter::TextAntialiasing, true);
  painter->fillRect(option.rect, RowBackground(selected, hovered, index.row()));

  const QColor accent = AccentColor(level);
  painter->fillRect(QRect(option.rect.left(), option.rect.top(), kAccentWidth,
                          option.rect.height()),
                    accent);

  const QString timestamp = index.data(kRoleTimestamp).toString();
  const QString name = index.data(kRoleName).toString();
  const QString message = index.data(kRoleMessage).toString();
  const QString body = BuildBodyText(name, message);

  QRect content =
      option.rect.adjusted(kPaddingH + kAccentWidth, kPaddingV, -kPaddingH,
                           -kPaddingV);

  QFont font = option.font;
  font.setFamily(QStringLiteral("Monospace"));
  painter->setFont(font);
  const QFontMetrics metrics(font);

  int x = content.left();

  painter->setPen(QColor(100, 116, 139));
  painter->drawText(QRect(x, content.top(), kTimeWidth, content.height()),
                    Qt::AlignLeft | Qt::AlignVCenter, timestamp);
  x += kTimeWidth + kColumnGap;

  QFont level_font = font;
  level_font.setBold(true);
  painter->setFont(level_font);
  painter->setPen(LevelColor(level));
  painter->drawText(QRect(x, content.top(), kLevelWidth, content.height()),
                    Qt::AlignLeft | Qt::AlignVCenter, ShortLevelLabel(level));
  x += kLevelWidth + kColumnGap;

  painter->setFont(font);
  painter->setPen(MessageColor(level));
  const int body_width = std::max(20, content.right() - x + 1);
  QRect body_rect(x, content.top(), body_width, content.height());
  painter->drawText(body_rect, Qt::AlignLeft | Qt::AlignVCenter,
                    metrics.elidedText(body, Qt::ElideRight, body_width));

  painter->restore();
}

QSize LogEntryDelegate::sizeHint(const QStyleOptionViewItem& option,
                                 const QModelIndex& index) const {
  if (!index.isValid()) {
    return QStyledItemDelegate::sizeHint(option, index);
  }

  const int total_width = option.rect.width() > 0 ? option.rect.width() : 480;

  QFont font = option.font;
  font.setFamily(QStringLiteral("Monospace"));
  const int line_height = QFontMetrics(font).height();
  return QSize(total_width, std::max(kMinRowHeight, line_height + kPaddingV * 2));
}

}  // namespace log_panel
}  // namespace autoviz
