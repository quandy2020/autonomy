/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/icon_loader.hpp"

#include <QCursor>
#include <QFile>
#include <QImage>
#include <QPainter>
#include <QPixmap>
#include <QPixmapCache>
#include <QSvgRenderer>

#include "autoviz/common/display_status.hpp"
#include "autoviz/ui/panel_dock_widget.hpp"

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
  return QStringLiteral(":/autoviz/icons/cursor");
}

/** RViz2 panel / display icon resource base (no extension). */
QString MapPanelResourceBase(const QString& panel_id) {
  static const struct {
    const char* id;
    const char* icon;
  } kMap[] = {
      // rviz_common built-in panels (icons/classes/{Name}.png|.svg)
      {"Displays", "classes/Displays"},
      {"Views", "classes/Views"},
      {"Help", "classes/Help"},
      {"Selection", "classes/Selection"},
      {"Time", "classes/Time"},
      {"Playback", "classes/Time"},
      {"ToolProperties", "classes/Tool Properties"},
      {"Transformation", "classes/Transformation"},
      // Autoviz panels — RViz display icons matched by role
      {"Panel3D", "panels/panel_3d"},
      {"PanelImage", "panels/panel_image"},
      {"PanelMap", "panels/panel_map"},
      {"PanelPlot", "panels/panel_plot"},
      {"PanelRawMessages", "panels/panel_raw_messages"},
      {"PanelTransformTree", "panels/panel_transform_tree"},
      {"PanelDataSource", "panels/panel_data_source"},
      {"PanelParameters", "panels/panel_parameters"},
      {"PanelPublish", "panels/panel_publish"},
      {"PanelTable", "panels/panel_table"},
      {"PanelTeleop", "panels/panel_teleop"},
      {"PanelGauge", "panels/panel_gauge"},
      {"PanelIndicator", "panels/panel_indicator"},
      {"PanelState", "panels/panel_state"},
      {"PanelService", "panels/panel_service"},
      {"PanelLog", "panels/panel_log"},
      {"PanelChannelGraph", "panels/panel_channel_graph"},
      {"PanelChannels", "panels/panel_channels"},
      {"PanelProblems", "panels/panel_problems"},
      {"PanelMarkdown", "panels/panel_markdown"},
      {"PanelStack", "panels/panel_stack"},
      {"PanelTab", "panels/panel_tab"},
      {"PanelAudio", "panels/panel_audio"},
      {"PanelVariables", "panels/panel_variables"},
      {"PanelVariableSlider", "panels/panel_variable_slider"},
      {"PanelStrataFloor", "classes/Grid"},
  };
  for (const auto& entry : kMap) {
    if (panel_id == QLatin1String(entry.id)) {
      return QStringLiteral(":/autoviz/icons/") + QLatin1String(entry.icon);
    }
  }
  return QStringLiteral(":/autoviz/icons/default_class_icon");
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
      {"AudioDock", "PanelAudio"},
      {"PlaybackDock", "PanelDataSource"},
      {"GaugeDock", "PanelGauge"},
      {"ImageDock", "PanelImage"},
      {"IndicatorDock", "PanelIndicator"},
      {"LogDock", "PanelLog"},
      {"MapDock", "PanelMap"},
      {"HelpDock", "PanelMarkdown"},
      {"ParametersDock", "PanelParameters"},
      {"PlotDock", "PanelPlot"},
      {"PublishDock", "PanelPublish"},
      {"ChannelsDock", "PanelRawMessages"},
      {"ChannelBrowserDock", "PanelChannels"},
      {"ProblemsDock", "PanelProblems"},
      {"ServiceDock", "PanelService"},
      {"StateTransitionDock", "PanelState"},
      {"TableDock", "PanelTable"},
      {"TeleopDock", "PanelTeleop"},
      {"ChannelGraphDock", "PanelChannelGraph"},
      {"VariablesDock", "PanelVariables"},
      {"VariableSliderDock", "PanelVariableSlider"},
      {"TfTreeDock", "PanelTransformTree"},
      {"StrataFloorDock", "PanelStrataFloor"},
      {"DisplaysDock", "Displays"},
      {"ViewsDock", "Views"},
      {"SelectionDock", "Selection"},
      {"ToolPropertiesDock", "ToolProperties"},
      {"TimeDock", "Time"},
      {"TransformationDock", "Transformation"},
      {"PropertyInspectorDock", "ToolProperties"},
      {"DroneDock", "Panel3D"},
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
      {"viewport.reset_view", "rotate"},
      {"viewport.view_settings", "classes/Views"},
      {"viewport.inspect", "panel/panel_inspect"},
      {"viewport.camera_2d", "move2d"},
      {"viewport.measure", "classes/Measure"},
      {"viewport.recenter_frame", "panel/panel_recenter"},
      {"panel.close", "panel/panel_close"},
      {"panel.expand", "panel/panel_expand"},
      {"panel.more", "panel/panel_more"},
      {"panel.settings", "panel/panel_settings"},
      {"panel.split_right", "right_dock"},
      {"panel.split_down", "left_dock"},
      {"plot.reset_view", "plot/plot_reset_view"},
      {"plot.select", "classes/Select"},
      {"plot.inspect", "crosshair"},
      {"plot.zoom", "zoom"},
      {"plot.legend", "plot/plot_legend"},
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
      {"file.open", "menu/open_config"},
      {"file.save", "menu/save_config"},
      {"file.save_as", "menu/save_as"},
      {"file.recent", "menu/recent"},
      {"file.image", "menu/screenshot"},
      {"file.quit", "menu/quit"},
      {"file.config", "menu/config_file"},
      {"layout.open", "menu/open_config"},
      {"layout.save", "menu/save_config"},
      {"layout.save_as", "menu/save_as"},
      {"layout.reset", "menu/reset_layout"},
      {"panels.add", "plus"},
      {"panels.delete", "minus"},
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
      {"help.about", "menu/about"},
      {"app.settings", "menu/settings"},
  };
  for (const auto& entry : kMap) {
    if (menu_id == QLatin1String(entry.id)) {
      return QStringLiteral(":/autoviz/icons/") + entry.icon;
    }
  }
  return QStringLiteral(":/autoviz/icons/menu");
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

QPixmap RenderSvgPixmap(const QString& resource_path, int size) {
  if (!QFile::exists(resource_path)) {
    return {};
  }
  QSvgRenderer renderer(resource_path);
  if (!renderer.isValid() || size <= 0) {
    return {};
  }
  QPixmap pixmap(size, size);
  pixmap.fill(Qt::transparent);
  QPainter painter(&pixmap);
  ConfigureIconPainter(painter);
  renderer.render(&painter, QRectF(0, 0, size, size));
  painter.end();
  return pixmap;
}

QIcon IconFromSvgResource(const QString& resource_path, const int* sizes, int count) {
  if (!QFile::exists(resource_path)) {
    return {};
  }
  QSvgRenderer renderer(resource_path);
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
  static const char* kExtensions[] = {".svg", ".png"};
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
  return LoadIconByBase(QStringLiteral(":/autoviz/icons/default_class_icon"));
}

QIcon LoadToolIcon(const QString& tool_id) {
  const QString base = MapToolIconBase(tool_id);
  QIcon icon = LoadIconByBase(base);
  if (!icon.isNull()) {
    return icon;
  }
  return LoadIconByBase(QStringLiteral(":/autoviz/icons/default_class_icon"));
}

QIcon LoadMenuIcon(const QString& resource_base) {
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
  return LoadIconByBase(QStringLiteral(":/autoviz/icons/aviz"));
}

QIcon IconLoader::load(const QString& resource_path) {
  QIcon icon = LoadIconByBase(StripExtension(resource_path));
  if (!icon.isNull()) {
    return icon;
  }
  return LoadIconByBase(QStringLiteral(":/autoviz/icons/default_class_icon"));
}

QIcon IconLoader::displayIcon(const QString& display_type) {
  return LoadDisplayIcon(display_type);
}

QIcon IconLoader::toolIcon(const QString& tool_id) {
  return LoadToolIcon(tool_id);
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
      RenderSvgPixmap(QStringLiteral(":/autoviz/icons/cursor.svg"), kCursorSize);
  if (base_cursor.isNull()) {
    base_cursor = load(QStringLiteral(":/autoviz/icons/cursor")).pixmap(kCursorSize, kCursorSize);
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
  QIcon icon = LoadMenuIcon(MapMenuIcon(menu_id));
  if (!icon.isNull()) {
    return icon;
  }
  return load(QStringLiteral(":/autoviz/icons/menu"));
}

QIcon IconLoader::panelTitleIcon(const QString& role) {
  const QString base = MapPanelTitleIcon(role);
  if (!base.isEmpty()) {
    QIcon icon = LoadIconByBase(base);
    if (!icon.isNull()) {
      return icon;
    }
  }
  return load(QStringLiteral(":/autoviz/icons/default_class_icon"));
}

QIcon IconLoader::panelExpandIcon() {
  QIcon icon;
  icon.addFile(QStringLiteral(":/autoviz/icons/panel/panel_expand.svg"), QSize(),
               QIcon::Normal, QIcon::Off);
  icon.addFile(QStringLiteral(":/autoviz/icons/panel/panel_collapse.svg"), QSize(),
               QIcon::Normal, QIcon::On);
  return icon;
}

QIcon StatusIconFromResource(const QString& resource_base) {
  static const char* kExtensions[] = {".svg", ".png"};
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
    return StatusIconFromResource(QStringLiteral(":/autoviz/icons/status_disabled"));
  }
  switch (level) {
    case display::DisplayStatusLevel::kError:
      return StatusIconFromResource(QStringLiteral(":/autoviz/icons/status_error"));
    case display::DisplayStatusLevel::kWarn:
      return StatusIconFromResource(QStringLiteral(":/autoviz/icons/status_warn"));
    case display::DisplayStatusLevel::kOk:
    default:
      return StatusIconFromResource(QStringLiteral(":/autoviz/icons/status_ok"));
  }
}

}  // namespace autoviz
