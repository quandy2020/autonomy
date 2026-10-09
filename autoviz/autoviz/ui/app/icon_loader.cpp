/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/app/icon_loader.hpp"

#include <QApplication>
#include <QCursor>
#include <QFile>
#include <QImage>
#include <QPainter>
#include <QPixmap>
#include <QPixmapCache>
#include <QSvgRenderer>

#include <cmath>

#include "autoviz/common/display_status.hpp"
#include "autoviz/ui/theme/application.hpp"
#include "autoviz/ui/panel/dock.hpp"

namespace autoviz {
namespace {

QString MapDisplayIcon(const QString& display_type) {
  return QStringLiteral(":/autoviz/icons/classes/") + display_type;
}

QString MapToolIconBase(const QString& tool_id) {
  static const struct {
    const char* id;
    const char* icon;
  } kMap[] = {
      {"Interact", "classes/Interact"},
      {"MoveCamera", "classes/MoveCamera"},
      {"Select", "classes/Select"},
      {"FocusCamera", "classes/FocusCamera"},
      {"Measure", "classes/Measure"},
      {"PoseEstimate", "classes/SetInitialPose"},
      {"SetInitialPose", "classes/SetInitialPose"},
      {"NavGoal", "classes/SetGoal"},
      {"SetGoal", "classes/SetGoal"},
      {"PublishPoint", "classes/PublishPoint"},
  };
  for (const auto& entry : kMap) {
    if (tool_id == QLatin1String(entry.id)) {
      return QStringLiteral(":/autoviz/icons/") + QLatin1String(entry.icon);
    }
  }
  return QStringLiteral(":/autoviz/icons/tool/cursor");
}

/** RViz2 panel / display icon resource base (no extension). */
QString MapPanelResourceBase(const QString& panel_id) {
  static const struct {
    const char* id;
    const char* icon;
  } kMap[] = {
      // rviz_common built-in panels (icons/classes/{Name}.svg)
      {"Displays", "classes/Displays"},
      {"Views", "classes/Views"},
      {"Selection", "classes/Selection"},
      {"Time", "classes/Time"},
      {"ToolProperties", "classes/ToolProperties"},
      {"Properties", "classes/ToolProperties"},
      // Autoviz panels — filename matches role under icons/panels/
      {"Panel3D", "panels/3d"},
      {"PanelImage", "panels/image"},
      {"PanelMap", "panels/map"},
      {"PanelPlot", "panels/plot"},
      {"PanelTable", "panels/table"},
      {"PanelRawMessages", "panels/raw_messages"},
      {"PanelTransformTree", "panels/transform_tree"},
      {"PanelPublish", "panels/publish"},
      {"PanelTeleop", "panels/teleop"},
      {"PanelService", "panels/service"},
      {"PanelChannelGraph", "panels/channel_graph"},
      {"PanelChannels", "panels/channels"},
      {"PanelRecord", "panels/data_source"},
      {"PanelStack", "panels/stack"},
      {"PanelTab", "panels/tab"},
  };
  for (const auto& entry : kMap) {
    if (panel_id == QLatin1String(entry.id)) {
      return QStringLiteral(":/autoviz/icons/") + QLatin1String(entry.icon);
    }
  }
  return QStringLiteral(":/autoviz/icons/default/class");
}

QString MapPanelIcon(const QString& panel_id) {
  return MapPanelResourceBase(panel_id);
}

QString MapDockTypeIcon(const QString& dock_type_id) {
  static const struct {
    const char* dock_id;
    const char* panel_id;
  } kMap[] = {
      {"ViewportDock", "Panel3D"},
      {"ImageDock", "PanelImage"},
      {"ImageDisplayDock", "PanelImage"},
      {"MapDock", "PanelMap"},
      {"PlotDock", "PanelPlot"},
      {"TableDock", "PanelTable"},
      {"PublishDock", "PanelPublish"},
      {"ChannelsDock", "PanelRawMessages"},
      {"ChannelBrowserDock", "PanelChannels"},
      {"RecordDock", "PanelRecord"},
      {"ServiceDock", "PanelService"},
      {"TeleopDock", "PanelTeleop"},
      {"ChannelGraphDock", "PanelChannelGraph"},
      {"TfTreeDock", "PanelTransformTree"},
      {"DisplaysDock", "Displays"},
      {"ViewsDock", "Views"},
      {"SelectionDock", "Selection"},
      {"ToolPropertiesDock", "ToolProperties"},
      {"PropertiesDock", "ToolProperties"},
      {"TimeDock", "Time"},
      {"Vehicle3DDock", "Panel3D"},
  };
  for (const auto& entry : kMap) {
    if (dock_type_id == QLatin1String(entry.dock_id)) {
      return MapPanelResourceBase(QLatin1String(entry.panel_id));
    }
  }
  return MapPanelResourceBase(dock_type_id);
}

QString MapPanelTitleIcon(const QString& role) {
  static const struct {
    const char* role;
    const char* icon;
  } kMap[] = {
      {"viewport.interact", "classes/Interact"},
      {"viewport.move_camera", "classes/MoveCamera"},
      {"viewport.reset_view", "tool/rotate"},
      {"viewport.view_settings", "classes/Views"},
      {"viewport.inspect", "viewport/inspect"},
      {"viewport.camera_2d", "viewport/camera"},
      {"viewport.measure", "viewport/measure"},
      {"viewport.gauge", "viewport/gauge"},
      {"viewport.recenter_frame", "viewport/recenter"},
      {"panel.close", "panel/close"},
      {"panel.expand", "panel/expand"},
      {"panel.more", "panel/more"},
      {"panel.settings", "panel/settings"},
      {"panel.split_right", "dock/right"},
      {"panel.split_down", "dock/down"},
      {"plot.reset_view", "plot/reset_view"},
      {"plot.select", "plot/select"},
      {"plot.inspect", "tool/crosshair"},
      {"plot.brush", "plot/brush"},
      {"plot.pan", "plot/pan"},
      {"plot.zoom", "plot/zoom"},
      {"plot.legend", "plot/legend"},
  };
  for (const auto& entry : kMap) {
    if (role == QLatin1String(entry.role)) {
      return QStringLiteral(":/autoviz/icons/") + QLatin1String(entry.icon);
    }
  }
  return {};
}

QString MapMenuIcon(const QString& menu_id) {
  static const struct {
    const char* id;
    const char* icon;
  } kMap[] = {
      {"menu.app", "menu/app"},
      {"menu.file", "menu/file"},
      {"menu.layout", "menu/layout"},
      {"menu.panels", "classes/Displays"},
      {"menu.view", "menu/view"},
      {"menu.help", "menu/help"},
      {"menu.camera", "menu/camera"},
      {"menu.backend", "menu/backend"},
      {"file.open", "menu/file_open"},
      {"file.open_record", "menu/file_open_record"},
      {"file.save", "menu/file_save"},
      {"file.save_as", "menu/file_save_as"},
      {"file.recent", "menu/file_recent"},
      {"file.image", "menu/file_image"},
      {"file.quit", "menu/file_quit"},
      {"file.config", "menu/file_open"},
      {"file.reset_layout", "menu/file_reset"},
      {"layout.open", "menu/file_open"},
      {"layout.save", "menu/file_save"},
      {"layout.save_as", "menu/file_save_as"},
      {"layout.reset", "menu/file_reset"},
      {"panels.add", "menu/add_panel"},
      {"panels.add", "tool/plus"},
      {"panels.delete", "tool/minus"},
      {"panels.fullscreen", "classes/Views"},
      {"panels.tools", "classes/Interact"},
      {"view.orbit", "menu/orbit"},
      {"view.xy_orbit", "menu/xy_orbit"},
      {"view.top_down", "menu/top_down"},
      {"view.top_down_ortho", "menu/top_down_ortho"},
      {"view.third_person", "menu/third_person"},
      {"view.fps", "menu/fps"},
      {"view.opengl", "menu/opengl"},
      {"view.ogre", "menu/ogre"},
      {"help.panel", "menu/help"},
      {"help.about", "menu/file_about"},
      {"app.settings", "menu/file_settings"},
  };
  for (const auto& entry : kMap) {
    if (menu_id == QLatin1String(entry.id)) {
      return QStringLiteral(":/autoviz/icons/") + entry.icon;
    }
  }
  return QStringLiteral(":/autoviz/icons/tool/menu");
}

QString StripExtension(const QString& path) {
  const int dot = path.lastIndexOf(QLatin1Char('.'));
  if (dot > 0) {
    return path.left(dot);
  }
  return path;
}

bool IsVisiblePixmap(const QPixmap& pixmap) {
  if (pixmap.isNull() || pixmap.width() <= 0 || pixmap.height() <= 0) {
    return false;
  }
  const QImage image = pixmap.toImage().convertToFormat(QImage::Format_ARGB32);
  for (int y = 0; y < image.height(); ++y) {
    for (int x = 0; x < image.width(); ++x) {
      if (qAlpha(image.pixel(x, y)) > 16) {
        return true;
      }
    }
  }
  return false;
}

void ConfigureIconPainter(QPainter& painter) {
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
}

/** SVG wrappers that embed the original PNG (QtSvg often skips <image>). */
QPixmap PixmapFromEmbeddedPngSvg(const QByteArray& svg_bytes) {
  static const QByteArray kMarker = QByteArrayLiteral("data:image/png;base64,");
  const int start = svg_bytes.indexOf(kMarker);
  if (start < 0) {
    return {};
  }
  int end = svg_bytes.indexOf('"', start);
  if (end < 0) {
    end = svg_bytes.indexOf('\'', start);
  }
  if (end <= start) {
    return {};
  }
  const QByteArray b64 =
      svg_bytes.mid(start + kMarker.size(), end - start - kMarker.size());
  QPixmap pixmap;
  if (!pixmap.loadFromData(QByteArray::fromBase64(b64), "PNG")) {
    return {};
  }
  return pixmap;
}

QByteArray ReadResourceBytes(const QString& resource_path) {
  if (!QFile::exists(resource_path)) {
    return {};
  }
  QFile file(resource_path);
  if (!file.open(QIODevice::ReadOnly)) {
    return {};
  }
  return file.readAll();
}

QPixmap RenderSvgPixmap(const QString& resource_path, int size) {
  if (size <= 0) {
    return {};
  }
  const QByteArray svg_bytes = ReadResourceBytes(resource_path);
  if (svg_bytes.isEmpty()) {
    return {};
  }
  const QPixmap embedded = PixmapFromEmbeddedPngSvg(svg_bytes);
  if (!embedded.isNull()) {
    return embedded.scaled(size, size, Qt::KeepAspectRatio,
                           Qt::SmoothTransformation);
  }
  QSvgRenderer renderer(svg_bytes);
  if (!renderer.isValid()) {
    return {};
  }
  // Premultiplied canvas: QSvgRenderer AA edges bleed on straight ARGB32.
  QImage image(size, size, QImage::Format_ARGB32_Premultiplied);
  image.fill(Qt::transparent);
  QPainter painter(&image);
  ConfigureIconPainter(painter);
  renderer.render(&painter, QRectF(0, 0, size, size));
  painter.end();
  return QPixmap::fromImage(image);
}

QImage TintMaskSourceIn(const QImage& mask, const QColor& color, int alpha) {
  QImage out(mask.size(), QImage::Format_ARGB32_Premultiplied);
  out.fill(0);
  QPainter painter(&out);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
  painter.drawImage(0, 0, mask);
  painter.setCompositionMode(QPainter::CompositionMode_SourceIn);
  QColor ink = color;
  ink.setAlpha(alpha);
  painter.fillRect(out.rect(), ink);
  painter.end();
  return out;
}

/** Fit opaque glyph into a square so thin plus/minus and full-bleed class
 *  icons share the same optical size in glass menus. */
QImage NormalizeMenuMask(const QImage& source, int size) {
  if (size <= 0) {
    return {};
  }
  QImage src = source.convertToFormat(QImage::Format_ARGB32_Premultiplied);
  if (src.isNull()) {
    return QImage(size, size, QImage::Format_ARGB32_Premultiplied);
  }

  int min_x = src.width();
  int min_y = src.height();
  int max_x = -1;
  int max_y = -1;
  for (int y = 0; y < src.height(); ++y) {
    const QRgb* line = reinterpret_cast<const QRgb*>(src.constScanLine(y));
    for (int x = 0; x < src.width(); ++x) {
      if (qAlpha(line[x]) > 24) {
        min_x = qMin(min_x, x);
        min_y = qMin(min_y, y);
        max_x = qMax(max_x, x);
        max_y = qMax(max_y, y);
      }
    }
  }

  QImage out(size, size, QImage::Format_ARGB32_Premultiplied);
  out.fill(0);
  if (max_x < min_x) {
    return out;
  }

  const QImage cropped =
      src.copy(QRect(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1));
  // Leave a thin margin so AA edges are not clipped by the 16px slot.
  const int target =
      qMax(1, static_cast<int>(std::lround(static_cast<double>(size) * 0.88)));
  const QImage fitted = cropped.scaled(target, target, Qt::KeepAspectRatio,
                                       Qt::SmoothTransformation);
  QPainter painter(&out);
  ConfigureIconPainter(painter);
  painter.drawImage((size - fitted.width()) / 2, (size - fitted.height()) / 2,
                    fitted);
  painter.end();
  return out;
}

void AddMonochromeMenuSlices(QIcon& icon, const QImage& raw_mask, const QColor& ink,
                             qreal scale, bool white_when_on = false) {
  const QImage mask = NormalizeMenuMask(raw_mask, raw_mask.width());
  const QColor normal_ink =
      ink.isValid() ? ink : QColor(0x47, 0x55, 0x69);  // #475569
  const QColor active_ink = QColor(0x0F, 0x76, 0x6E);  // #0D9488

  auto make_pm = [&](const QColor& color, int alpha) {
    QPixmap pm(QPixmap::fromImage(TintMaskSourceIn(mask, color, alpha)));
    pm.setDevicePixelRatio(scale);
    return pm;
  };

  const QPixmap normal_px = make_pm(normal_ink, 255);
  const QPixmap active_px = make_pm(active_ink, 255);
  const QPixmap on_px =
      white_when_on ? make_pm(QColor(255, 255, 255), 255) : active_px;
  const QPixmap disabled_px = make_pm(normal_ink, 110);
  for (QIcon::Mode mode : {QIcon::Normal, QIcon::Active, QIcon::Selected}) {
    const QPixmap& off_px = (mode == QIcon::Normal) ? normal_px : active_px;
    icon.addPixmap(off_px, mode, QIcon::Off);
    icon.addPixmap(white_when_on ? on_px : off_px, mode, QIcon::On);
  }
  icon.addPixmap(disabled_px, QIcon::Disabled, QIcon::Off);
  icon.addPixmap(disabled_px, QIcon::Disabled, QIcon::On);
}

QIcon MonochromeFromSvgResource(const QString& resource_path, const QColor& ink,
                                bool white_when_on = false) {
  const QByteArray svg_bytes = ReadResourceBytes(resource_path);
  if (svg_bytes.isEmpty()) {
    return {};
  }
  QSvgRenderer renderer(svg_bytes);
  if (!renderer.isValid()) {
    return {};
  }

  constexpr int kLogical = 16;
  const qreal dpr = qMax<qreal>(1.0, qApp ? qApp->devicePixelRatio() : 1.0);

  QIcon icon;
  for (qreal scale : {dpr, dpr * 2.0}) {
    const int px =
        qMax(1, static_cast<int>(std::lround(static_cast<double>(kLogical) * scale)));
    QImage mask(px, px, QImage::Format_ARGB32_Premultiplied);
    mask.fill(0);
    {
      QPainter painter(&mask);
      ConfigureIconPainter(painter);
      renderer.render(&painter, QRectF(0, 0, px, px));
    }
    AddMonochromeMenuSlices(icon, mask, ink, scale, white_when_on);
  }
  return icon;
}

QIcon MonochromeFromResourceBase(const QString& resource_base, const QColor& ink,
                                 bool white_when_on = false) {
  const QString svg_path = resource_base + QStringLiteral(".svg");
  if (QFile::exists(svg_path)) {
    const QIcon tinted =
        MonochromeFromSvgResource(svg_path, ink, white_when_on);
    if (!tinted.isNull()) {
      return tinted;
    }
  }
  const QString png_path = resource_base + QStringLiteral(".png");
  if (!QFile::exists(png_path)) {
    return {};
  }
  constexpr int kLogical = 16;
  const qreal dpr = qMax<qreal>(1.0, qApp ? qApp->devicePixelRatio() : 1.0);
  const QPixmap source(png_path);
  if (source.isNull()) {
    return {};
  }
  QIcon icon;
  for (qreal scale : {dpr, dpr * 2.0}) {
    const int px =
        qMax(1, static_cast<int>(std::lround(static_cast<double>(kLogical) * scale)));
    QImage mask(px, px, QImage::Format_ARGB32_Premultiplied);
    mask.fill(0);
    {
      QPainter painter(&mask);
      ConfigureIconPainter(painter);
      const QPixmap scaled =
          source.scaled(px, px, Qt::KeepAspectRatio, Qt::SmoothTransformation);
      painter.drawPixmap((px - scaled.width()) / 2, (px - scaled.height()) / 2,
                         scaled);
    }
    AddMonochromeMenuSlices(icon, mask, ink, scale, white_when_on);
  }
  return icon;
}

QIcon IconFromSvgResource(const QString& resource_path, const int* sizes, int count) {
  const QByteArray svg_bytes = ReadResourceBytes(resource_path);
  if (svg_bytes.isEmpty()) {
    return {};
  }
  const QPixmap embedded = PixmapFromEmbeddedPngSvg(svg_bytes);
  if (!embedded.isNull()) {
    QIcon icon;
    for (int i = 0; i < count; ++i) {
      const QPixmap pixmap = embedded.scaled(
          sizes[i], sizes[i], Qt::KeepAspectRatio, Qt::SmoothTransformation);
      if (IsVisiblePixmap(pixmap)) {
        icon.addPixmap(pixmap);
      }
    }
    return icon;
  }

  QSvgRenderer renderer(svg_bytes);
  if (!renderer.isValid()) {
    return {};
  }

  QIcon icon;
  for (int i = 0; i < count; ++i) {
    const QPixmap pixmap = RenderSvgPixmap(resource_path, sizes[i]);
    if (IsVisiblePixmap(pixmap)) {
      icon.addPixmap(pixmap);
    }
  }
  return icon;
}

QIcon IconFromPngResource(const QString& resource_path) {
  if (!QFile::exists(resource_path)) {
    return {};
  }
  const QPixmap source(resource_path);
  if (!IsVisiblePixmap(source)) {
    return {};
  }
  return QIcon(source);
}

QIcon IconFromResourcePath(const QString& resource_path, const int* sizes,
                           int count) {
  if (resource_path.endsWith(QLatin1String(".svg"), Qt::CaseInsensitive)) {
    return IconFromSvgResource(resource_path, sizes, count);
  }
  if (resource_path.endsWith(QLatin1String(".png"), Qt::CaseInsensitive)) {
    return IconFromPngResource(resource_path);
  }
  return {};
}

constexpr int kIconSizes[] = {16, 24, 32, 48};
constexpr int kMenuIconSizes[] = {16, 20, 24};
constexpr int kStatusIconSizes[] = {12, 16, 20, 24};

QIcon LoadIconByBase(const QString& resource_base) {
  // Prefer PNG (pixel-perfect) over SVG when both exist — many class icons are
  // 16×16 rasters; SVG wrappers were upscaled/blurry in HiDPI dialogs.
  static const char* kExtensions[] = {".png", ".svg"};
  for (const char* extension : kExtensions) {
    const QIcon icon = IconFromResourcePath(
        resource_base + QLatin1String(extension), kIconSizes,
        static_cast<int>(sizeof(kIconSizes) / sizeof(kIconSizes[0])));
    if (!icon.isNull()) {
      return icon;
    }
  }
  return {};
}

QIcon LoadDisplayIcon(const QString& display_type) {
  const QString base = MapDisplayIcon(display_type);
  QIcon icon = LoadIconByBase(base);
  if (!icon.isNull()) {
    return icon;
  }
  return LoadIconByBase(QStringLiteral(":/autoviz/icons/default/class"));
}

QIcon LoadToolIcon(const QString& tool_id) {
  const QString base = MapToolIconBase(tool_id);
  QIcon icon = LoadIconByBase(base);
  if (!icon.isNull()) {
    return icon;
  }
  return LoadIconByBase(QStringLiteral(":/autoviz/icons/default/class"));
}

QIcon LoadMenuIcon(const QString& resource_base) {
  // Prefer SVG line icons over legacy colorful PNGs.
  static const char* kExtensions[] = {".svg", ".png"};
  for (const char* extension : kExtensions) {
    const QIcon icon = IconFromResourcePath(
        resource_base + QLatin1String(extension), kMenuIconSizes,
        static_cast<int>(sizeof(kMenuIconSizes) / sizeof(kMenuIconSizes[0])));
    if (!icon.isNull()) {
      return icon;
    }
  }
  return {};
}

}  // namespace

QIcon IconLoader::applicationIcon() {
  QIcon icon;
#if defined(Q_OS_MACOS) || defined(Q_OS_MAC)
  // Prefer .icns so Dock / Mission Control pick up the squircle mask.
  const QIcon icns(QStringLiteral(":/autoviz/icons/aviz.icns"));
  if (!icns.isNull()) {
    return icns;
  }
#endif
  // Multi-resolution squirrel brand mark (macOS squircle, transparent corners).
  icon.addFile(QStringLiteral(":/autoviz/icons/aviz_32.png"), QSize(32, 32));
  icon.addFile(QStringLiteral(":/autoviz/icons/aviz_64.png"), QSize(64, 64));
  icon.addFile(QStringLiteral(":/autoviz/icons/aviz_128.png"), QSize(128, 128));
  icon.addFile(QStringLiteral(":/autoviz/icons/aviz_256.png"), QSize(256, 256));
  icon.addFile(QStringLiteral(":/autoviz/icons/aviz.png"), QSize(512, 512));
  icon.addFile(QStringLiteral(":/autoviz/icons/aviz_1024.png"), QSize(1024, 1024));
  if (!icon.isNull()) {
    return icon;
  }
  return LoadIconByBase(QStringLiteral(":/autoviz/icons/aviz"));
}

QIcon IconLoader::load(const QString& resource_path) {
  QIcon icon = LoadIconByBase(StripExtension(resource_path));
  if (!icon.isNull()) {
    return icon;
  }
  return LoadIconByBase(QStringLiteral(":/autoviz/icons/default/class"));
}

QIcon IconLoader::displayIcon(const QString& display_type) {
  return LoadDisplayIcon(display_type);
}

QIcon IconLoader::toolIcon(const QString& tool_id) {
  const QColor ink = PanelsMenuIconInk();
  const QIcon tinted = MonochromeFromResourceBase(MapToolIconBase(tool_id), ink,
                                                   /*white_when_on=*/true);
  if (!tinted.isNull()) {
    return tinted;
  }
  // Fallback: normalize whatever LoadToolIcon finds, with white checked state.
  const QIcon raw = LoadToolIcon(tool_id);
  if (raw.isNull()) {
    return {};
  }
  constexpr int kLogical = 20;
  const qreal dpr = qMax<qreal>(1.0, qApp ? qApp->devicePixelRatio() : 1.0);
  QIcon out;
  for (qreal scale : {dpr, dpr * 2.0}) {
    const int px =
        qMax(1, static_cast<int>(std::lround(static_cast<double>(kLogical) * scale)));
    QImage mask(px, px, QImage::Format_ARGB32_Premultiplied);
    mask.fill(0);
    {
      QPainter painter(&mask);
      ConfigureIconPainter(painter);
      raw.paint(&painter, QRect(0, 0, px, px), Qt::AlignCenter, QIcon::Normal,
                QIcon::Off);
    }
    AddMonochromeMenuSlices(out, mask, ink, scale, /*white_when_on=*/true);
  }
  return out;
}

QIcon IconLoader::toolbarGlyph(const QString& resource_base) {
  const QColor ink = PanelsMenuIconInk();
  const QString base = StripExtension(resource_base);
  // Keep ink on checked tools — toolbar uses a soft tint, not solid pills.
  const QIcon tinted =
      MonochromeFromResourceBase(base, ink, /*white_when_on=*/false);
  if (!tinted.isNull()) {
    return tinted;
  }
  return monochromeMenuIcon(load(resource_base), ink);
}

QCursor IconLoader::defaultCursor() {
  return QCursor(Qt::ArrowCursor);
}

QCursor IconLoader::makeIconCursor(const QPixmap& icon, const QString& cache_key) {
  QPixmap cursor_img;
  if (QPixmapCache::find(cache_key, &cursor_img)) {
    return QCursor(cursor_img, 1, 1);
  }

  constexpr int kCursorSize = 32;
  QPixmap base_cursor =
      RenderSvgPixmap(QStringLiteral(":/autoviz/icons/tool/cursor.svg"), kCursorSize);
  if (base_cursor.isNull()) {
    base_cursor = load(QStringLiteral(":/autoviz/icons/tool/cursor")).pixmap(kCursorSize, kCursorSize);
  }
  if (base_cursor.isNull() || icon.isNull()) {
    return defaultCursor();
  }

  cursor_img = QPixmap(kCursorSize, kCursorSize);
  cursor_img.fill(Qt::transparent);

  int draw_x = 12;
  int draw_y = 16;
  if (draw_x + icon.width() > kCursorSize) {
    draw_x = kCursorSize - icon.width();
  }
  if (draw_y + icon.height() > kCursorSize) {
    draw_y = kCursorSize - icon.height();
  }

  QPainter painter(&cursor_img);
  painter.drawPixmap(0, 0, base_cursor);
  painter.drawPixmap(draw_x, draw_y, icon);
  painter.end();

  QPixmapCache::insert(cache_key, cursor_img);
  return QCursor(cursor_img, 1, 1);
}

QCursor IconLoader::toolCursor(const QString& tool_id) {
  if (tool_id == QLatin1String("MoveCamera")) {
    return defaultCursor();
  }
  if (tool_id == QLatin1String("Interact")) {
    return QCursor(Qt::PointingHandCursor);
  }

  const QIcon icon = toolIcon(tool_id);
  QPixmap pixmap = icon.pixmap(22, 22);
  if (pixmap.isNull()) {
    pixmap = icon.pixmap(16, 16);
  }
  if (pixmap.isNull()) {
    return defaultCursor();
  }
  return makeIconCursor(pixmap, QStringLiteral("tool_cursor:") + tool_id);
}

QIcon IconLoader::panelIcon(const QString& panel_id) {
  return LoadIconByBase(MapPanelIcon(panel_id));
}

QIcon IconLoader::dockPanelIcon(const QString& dock_type_id) {
  return LoadIconByBase(MapDockTypeIcon(dock_type_id));
}

void IconLoader::applyDockPanelChrome(PanelDockWidget* dock,
                                      const QString& dock_type_id) {
  if (dock == nullptr) {
    return;
  }
  dock->setPanelIcon(dockPanelIcon(dock_type_id));
}

QIcon IconLoader::menuIcon(const QString& menu_id) {
  const QString base = MapMenuIcon(menu_id);
  const QIcon tinted = MonochromeFromResourceBase(base, PanelsMenuIconInk());
  if (!tinted.isNull()) {
    return tinted;
  }
  QIcon icon = LoadMenuIcon(base);
  if (icon.isNull()) {
    icon = load(QStringLiteral(":/autoviz/icons/tool/menu"));
  }
  return monochromeMenuIcon(icon, PanelsMenuIconInk());
}

QIcon IconLoader::panelsMenuDockIcon(const QString& dock_type_id) {
  const QIcon tinted =
      MonochromeFromResourceBase(MapDockTypeIcon(dock_type_id), PanelsMenuIconInk());
  if (!tinted.isNull()) {
    return tinted;
  }
  return monochromeMenuIcon(dockPanelIcon(dock_type_id), PanelsMenuIconInk());
}

QIcon IconLoader::monochromeMenuIcon(const QIcon& source, const QColor& ink) {
  if (source.isNull()) {
    return {};
  }
  constexpr int kLogical = 16;
  const qreal dpr = qMax<qreal>(1.0, qApp ? qApp->devicePixelRatio() : 1.0);

  QIcon out;
  for (qreal scale : {dpr, dpr * 2.0}) {
    const int px =
        qMax(1, static_cast<int>(std::lround(static_cast<double>(kLogical) * scale)));
    QImage mask(px, px, QImage::Format_ARGB32_Premultiplied);
    mask.fill(0);
    {
      QPainter painter(&mask);
      ConfigureIconPainter(painter);
      // Prefer an explicit DPR pixmap, then scale-to-fit the mask.
      QPixmap src = source.pixmap(QSize(kLogical, kLogical), scale);
      if (src.isNull()) {
        src = source.pixmap(QSize(px, px));
      }
      if (!src.isNull()) {
        const QPixmap scaled =
            src.scaled(px, px, Qt::KeepAspectRatio, Qt::SmoothTransformation);
        painter.drawPixmap((px - scaled.width()) / 2, (px - scaled.height()) / 2,
                           scaled);
      } else {
        source.paint(&painter, QRect(0, 0, px, px), Qt::AlignCenter, QIcon::Normal,
                     QIcon::Off);
      }
    }
    AddMonochromeMenuSlices(out, mask, ink, scale);
  }
  return out;
}

QIcon IconLoader::panelTitleIcon(const QString& role) {
  const QString base = MapPanelTitleIcon(role);
  if (!base.isEmpty()) {
    QIcon icon = LoadIconByBase(base);
    if (!icon.isNull()) {
      return icon;
    }
  }
  return load(QStringLiteral(":/autoviz/icons/default/class"));
}

QIcon IconLoader::panelExpandIcon() {
  QIcon icon;
  icon.addFile(QStringLiteral(":/autoviz/icons/panel/expand.svg"), QSize(),
               QIcon::Normal, QIcon::Off);
  icon.addFile(QStringLiteral(":/autoviz/icons/panel/collapse.svg"), QSize(),
               QIcon::Normal, QIcon::On);
  return icon;
}

QIcon StatusIconFromResource(const QString& resource_base) {
  static const char* kExtensions[] = {".png", ".svg"};
  for (const char* extension : kExtensions) {
    const QIcon icon = IconFromResourcePath(
        resource_base + QLatin1String(extension), kStatusIconSizes,
        static_cast<int>(sizeof(kStatusIconSizes) / sizeof(kStatusIconSizes[0])));
    if (!icon.isNull()) {
      return icon;
    }
  }
  return {};
}

QIcon IconLoader::statusIcon(display::DisplayStatusLevel level, bool enabled) {
  if (!enabled) {
    return StatusIconFromResource(QStringLiteral(":/autoviz/icons/status/disabled"));
  }
  switch (level) {
    case display::DisplayStatusLevel::kError:
      return StatusIconFromResource(QStringLiteral(":/autoviz/icons/status/error"));
    case display::DisplayStatusLevel::kWarn:
      return StatusIconFromResource(QStringLiteral(":/autoviz/icons/status/warn"));
    case display::DisplayStatusLevel::kOk:
    default:
      return StatusIconFromResource(QStringLiteral(":/autoviz/icons/status/ok"));
  }
}

}  // namespace autoviz
