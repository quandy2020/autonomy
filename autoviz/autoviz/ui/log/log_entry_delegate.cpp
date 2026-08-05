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

constexpr int kPaddingH = 6;
constexpr int kPaddingV = 1;
constexpr int kColumnGap = 6;
constexpr int kTimeWidth = 76;
constexpr int kLevelWidth = 34;
constexpr int kMinRowHeight = 17;
constexpr int kAccentWidth = 2;

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

QColor RowBackground(const QPalette& palette, bool selected, bool hovered) {
  if (selected) {
    return palette.color(QPalette::Highlight).lighter(140);
  }
  if (hovered) {
    return palette.color(QPalette::AlternateBase);
  }
  return Qt::transparent;
}

QColor AccentColor(LogLevel level) {
  switch (level) {
    case LogLevel::kWarn:
      return QColor(180, 140, 60);
    case LogLevel::kError:
      return QColor(190, 90, 90);
    case LogLevel::kFatal:
      return QColor(160, 90, 130);
    default:
      return Qt::transparent;
  }
}

QColor LevelColor(LogLevel level, const QPalette& palette) {
  switch (level) {
    case LogLevel::kDebug:
      return palette.color(QPalette::PlaceholderText);
    case LogLevel::kInfo:
      return palette.color(QPalette::Mid);
    case LogLevel::kWarn:
      return QColor(158, 122, 48);
    case LogLevel::kError:
      return QColor(178, 88, 88);
    case LogLevel::kFatal:
      return QColor(150, 88, 120);
    default:
      return palette.color(QPalette::Mid);
  }
}

QColor MessageColor(LogLevel level, const QPalette& palette) {
  const QColor text = palette.color(QPalette::Text);
  switch (level) {
    case LogLevel::kDebug:
      return palette.color(QPalette::PlaceholderText);
    case LogLevel::kInfo:
      return text;
    case LogLevel::kWarn:
      return QColor(158, 122, 48);
    case LogLevel::kError:
      return QColor(178, 88, 88);
    case LogLevel::kFatal:
      return QColor(150, 88, 120);
    default:
      return text;
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
  const QPalette& palette = option.palette;

  painter->save();
  painter->fillRect(option.rect, RowBackground(palette, selected, hovered));

  const QColor accent = AccentColor(level);
  if (accent.alpha() > 0) {
    painter->fillRect(QRect(option.rect.left(), option.rect.top(), kAccentWidth,
                            option.rect.height()),
                      accent);
  }

  const QString timestamp = index.data(kRoleTimestamp).toString();
  const QString name = index.data(kRoleName).toString();
  const QString message = index.data(kRoleMessage).toString();
  const QString body = BuildBodyText(name, message);

  QRect content = option.rect.adjusted(kPaddingH, kPaddingV, -kPaddingH, -kPaddingV);
  if (accent.alpha() > 0) {
    content.setLeft(content.left() + kAccentWidth);
  }

  QFont font = option.font;
  font.setFamily(QStringLiteral("Monospace"));
  painter->setFont(font);
  const QFontMetrics metrics(font);

  int x = content.left();

  painter->setPen(palette.color(QPalette::PlaceholderText));
  painter->drawText(QRect(x, content.top(), kTimeWidth, content.height()),
                    Qt::AlignLeft | Qt::AlignVCenter, timestamp);
  x += kTimeWidth + kColumnGap;

  painter->setPen(LevelColor(level, palette));
  painter->drawText(QRect(x, content.top(), kLevelWidth, content.height()),
                    Qt::AlignLeft | Qt::AlignVCenter, ShortLevelLabel(level));
  x += kLevelWidth + kColumnGap;

  painter->setPen(MessageColor(level, palette));
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
