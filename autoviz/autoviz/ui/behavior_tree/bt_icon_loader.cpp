/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/behavior_tree/bt_icon_loader.hpp"

#include <QFile>
#include <QHash>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPainter>
#include <QSvgRenderer>

namespace autoviz {
namespace behavior_tree {
namespace {

constexpr char kGrootPrefix[] = ":/autoviz/icons/behavior_tree/groot/";
constexpr char kStyleResource[] = ":/autoviz/icons/behavior_tree/groot/NodesStyle.json";

QString KindFallbackResource(BtNodeKind kind) {
  switch (kind) {
    case BtNodeKind::kAction:
      return QStringLiteral("svg/letter_A.svg");
    case BtNodeKind::kCondition:
      return QStringLiteral("svg/letter_C.svg");
    case BtNodeKind::kControl:
      return QStringLiteral("svg/sequence.svg");
    case BtNodeKind::kDecorator:
      return QStringLiteral("svg/magic-wand.svg");
    case BtNodeKind::kSubTree:
      return QStringLiteral("svg/subtree.svg");
    case BtNodeKind::kRoot:
      return QStringLiteral("svg/tree.svg");
    case BtNodeKind::kUndefined:
      break;
  }
  return QStringLiteral("svg/nodes_tree.svg");
}

QByteArray TintSvgForLightBackground(QByteArray svg_bytes, const QColor& color) {
  const QString hex = color.name(QColor::HexRgb);
  const QByteArray fill = QStringLiteral("fill:%1").arg(hex).toUtf8();
  svg_bytes.replace("fill:#FFFFFF;", fill + ";");
  svg_bytes.replace("fill:#ffffff;", fill + ";");
  svg_bytes.replace("fill:#FFFFFF", fill);
  svg_bytes.replace("fill:#ffffff", fill);
  svg_bytes.replace("fill:white;", fill + ";");
  svg_bytes.replace("fill:white", fill);
  svg_bytes.replace("#FFFFFF", hex.toUtf8());
  svg_bytes.replace("#ffffff", hex.toUtf8());
  return svg_bytes;
}

QPixmap RenderResourcePixmap(const QString& resource_path, int size,
                             const std::optional<QColor>& svg_tint = std::nullopt,
                             bool light_toolbar = false) {
  if (resource_path.isEmpty() || size <= 0) {
    return {};
  }
  if (resource_path.endsWith(QLatin1String(".svg"), Qt::CaseInsensitive)) {
    QByteArray svg_bytes;
    if (QFile::exists(resource_path)) {
      QFile file(resource_path);
      if (file.open(QIODevice::ReadOnly)) {
        svg_bytes = file.readAll();
      }
    }
    if (svg_bytes.isEmpty()) {
      return {};
    }
    if (svg_tint.has_value()) {
      const QByteArray tinted_fill =
          QStringLiteral("fill:%1;").arg(svg_tint->name(QColor::HexRgb)).toUtf8();
      svg_bytes.replace("fill:#ffffff;", tinted_fill);
      svg_bytes.replace("fill:#FFFFFF;", tinted_fill);
    } else if (light_toolbar) {
      svg_bytes = TintSvgForLightBackground(svg_bytes, QColor(QStringLiteral("#334155")));
    }
    QSvgRenderer renderer(svg_bytes);
    if (!renderer.isValid()) {
      return {};
    }
    QPixmap pixmap(size, size);
    pixmap.fill(Qt::transparent);
    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
    renderer.render(&painter, QRectF(0, 0, size, size));
    return pixmap;
  }

  QPixmap source(resource_path);
  if (source.isNull()) {
    return {};
  }
  return source.scaled(size, size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
}

struct StyleEntry {
  QString icon;
  QString caption_alias;
  std::optional<QColor> caption_color;
};

QHash<QString, StyleEntry>& StyleEntries() {
  static QHash<QString, StyleEntry> entries;
  return entries;
}

bool& StyleLoaded() {
  static bool loaded = false;
  return loaded;
}

}  // namespace

QString BtIconLoader::grootResource(const QString& relative_path) {
  return QString::fromLatin1(kGrootPrefix) + relative_path;
}

void BtIconLoader::ensureStyleLoaded() {
  if (StyleLoaded()) {
    return;
  }
  StyleLoaded() = true;

  QFile file(QString::fromLatin1(kStyleResource));
  if (!file.open(QIODevice::ReadOnly)) {
    return;
  }

  const QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
  if (!doc.isObject()) {
    return;
  }

  const QJsonObject root = doc.object();
  for (auto it = root.begin(); it != root.end(); ++it) {
    if (!it.value().isObject()) {
      continue;
    }
    const QJsonObject obj = it.value().toObject();
    StyleEntry entry;
    if (obj.contains(QStringLiteral("icon"))) {
      entry.icon = obj.value(QStringLiteral("icon")).toString();
    }
    if (obj.contains(QStringLiteral("caption_color"))) {
      entry.caption_color = QColor(obj.value(QStringLiteral("caption_color")).toString());
    }
    if (obj.contains(QStringLiteral("caption_alias"))) {
      entry.caption_alias = obj.value(QStringLiteral("caption_alias")).toString();
    }
    StyleEntries().insert(it.key(), entry);
  }
}

QPixmap BtIconLoader::pixmap(const QString& resource_path, int size) {
  return RenderResourcePixmap(resource_path, size);
}

QIcon BtIconLoader::icon(const QString& resource_path, int size) {
  const QPixmap pix = pixmap(resource_path, size);
  if (pix.isNull()) {
    return {};
  }
  return QIcon(pix);
}

QIcon BtIconLoader::toolbarIconFromResource(const QString& resource_path, int size) {
  const QPixmap pix = RenderResourcePixmap(resource_path, size, std::nullopt, true);
  if (pix.isNull()) {
    return {};
  }
  return QIcon(pix);
}

QIcon BtIconLoader::toolbarIcon(const QString& relative_groot_path, int size) {
  return toolbarIconFromResource(grootResource(relative_groot_path), size);
}

QString BtIconLoader::nodeResource(const QString& registration_id, BtNodeKind kind) {
  ensureStyleLoaded();

  const auto lookup = [&](const QString& key) -> QString {
    const StyleEntry entry = StyleEntries().value(key);
    if (!entry.icon.isEmpty()) {
      return entry.icon;
    }
    return {};
  };

  if (!registration_id.isEmpty()) {
    const QString specific = lookup(registration_id);
    if (!specific.isEmpty()) {
      return specific;
    }
  }

  const QString kind_key = TagFromBtNodeKind(kind);
  if (!kind_key.isEmpty() && kind != BtNodeKind::kUndefined) {
    const QString kind_icon = lookup(kind_key);
    if (!kind_icon.isEmpty()) {
      return kind_icon;
    }
  }

  return grootResource(KindFallbackResource(kind));
}

QPixmap BtIconLoader::nodePixmap(const QString& registration_id, BtNodeKind kind, int size) {
  const std::optional<QColor> tint = nodeCaptionColor(registration_id, kind);
  return RenderResourcePixmap(nodeResource(registration_id, kind), size, tint);
}

QIcon BtIconLoader::nodeIcon(const QString& registration_id, BtNodeKind kind, int size) {
  const QPixmap pix = nodePixmap(registration_id, kind, size);
  if (pix.isNull()) {
    return {};
  }
  return QIcon(pix);
}

std::optional<QColor> BtIconLoader::nodeCaptionColor(const QString& registration_id,
                                                     BtNodeKind kind) {
  ensureStyleLoaded();
  if (StyleEntries().contains(registration_id)) {
    return StyleEntries().value(registration_id).caption_color;
  }
  const QString kind_key = TagFromBtNodeKind(kind);
  if (StyleEntries().contains(kind_key)) {
    return StyleEntries().value(kind_key).caption_color;
  }
  return std::nullopt;
}

QString BtIconLoader::nodeCaptionLabel(const QString& registration_id, BtNodeKind kind) {
  ensureStyleLoaded();
  if (StyleEntries().contains(registration_id)) {
    const QString alias = StyleEntries().value(registration_id).caption_alias;
    if (!alias.isEmpty()) {
      return alias;
    }
  }
  return registration_id;
}

QIcon BtIconLoader::panelIcon(int size) {
  return icon(QStringLiteral(":/autoviz/icons/panels/panel_behavior_tree.svg"), size);
}

QIcon BtIconLoader::modeIcon(const QString& mode_name, int size) {
  const QString lower = mode_name.trimmed().toLower();
  if (lower == QLatin1String("editor")) {
    return icon(grootResource(QStringLiteral("BT-edit.png")), size);
  }
  if (lower == QLatin1String("monitor")) {
    return icon(grootResource(QStringLiteral("BT-monitor.png")), size);
  }
  if (lower == QLatin1String("replay")) {
    return icon(grootResource(QStringLiteral("BT-log.png")), size);
  }
  return {};
}

}  // namespace behavior_tree
}  // namespace autoviz
