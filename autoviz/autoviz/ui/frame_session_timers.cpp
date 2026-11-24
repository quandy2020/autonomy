/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/frame_session.hpp"
#include "autoviz/ui/frame_session_internal.hpp"
#include "autoviz/ui/frame.hpp"
#include <QObject>
#include "autoviz/common/selection.hpp"
#include <algorithm>
#include <cstdio>
#include <unordered_map>
#include <unordered_set>
#include <QApplication>
#include <QFrame>
#include <QGuiApplication>
#include <QScrollArea>
#include <QSet>
#include <QStackedWidget>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QKeySequence>
#include <QLabel>
#include <QDateTime>
#include <QDialog>
#include <QDir>
#include <QDockWidget>
#include <QDragEnterEvent>
#include <QDragLeaveEvent>
#include <QDragMoveEvent>
#include <QDropEvent>
#include <QElapsedTimer>
#include <QEvent>
#include <QEventLoop>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QIODevice>
#include <QTextStream>
#include <QListWidget>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QMimeData>
#include <QUrl>
#include <QPainter>
#include <QPen>
#include <QPointer>
#include <QResizeEvent>
#include <QScreen>
#include <QSettings>
#include <QWindow>
#include <QShortcut>
#include <QSignalBlocker>
#include <QStatusBar>
#include <QSize>
#include <QSizePolicy>
#include <QTimer>
#include <QCursor>
#include <QVariant>
#include <QTabWidget>
#include <QToolBar>
#include <QToolButton>
#include <QGridLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QQuaternion>
#include <QVBoxLayout>
#include "autoviz/common/display_property.hpp"
#include "autoviz/common/tool_manager.hpp"
#include "autoviz/common/interaction/message_queue.hpp"
#include "autoviz/rendering/ogre_render_window.hpp"
#include "autoviz/rendering/ogre_scene_host.hpp"
#include "autoviz/rendering/gpu_capabilities.hpp"
#include "autoviz/rendering/render_system.hpp"
#include "autoviz/rendering/view_controller.hpp"
#include "autoviz/common/view_state_io.hpp"
#include "autoviz/ui/panel/add_dialog.hpp"
#include "autoviz/ui/app/settings_dialog.hpp"
#include "autoviz/ui/app/preferences.hpp"
#include "autoviz/ui/theme/application.hpp"
#include "autoviz/ui/app/translation.hpp"
#include "autoviz/ui/panel/catalog.hpp"
#include "autoviz/ui/panel/context_menu.hpp"
#include "autoviz/ui/panel/dock.hpp"
#include "autoviz/ui/panel/title_tools.hpp"
#include "autoviz/ui/theme/panel.hpp"
#include "autoviz/ui/displays/panel.hpp"
#include "autoviz/ui/app/icon_loader.hpp"
#include "autoviz/ui/image/image_panel.hpp"
#include "autoviz/ui/panel_host.hpp"
#include "autoviz/ui/panel/role.hpp"
#include "autoviz/ui/inspector/property_panel.hpp"
#include "autoviz/ui/dialog/import_record.hpp"
#include "autoviz/ui/dialog/record_open.hpp"
#include "autoviz/ui/plot/io/plot_config_io.hpp"
#include "autoviz/ui/publish/publish_config_io.hpp"
#include "autoviz/ui/image/image_config_io.hpp"
#include "autoviz/ui/plot/panel/plot_panel.hpp"
#include "autoviz/ui/teleop/teleop_panel.hpp"
#include "autoviz/ui/navigation/navigation_panel.hpp"
#include "autoviz/ui/raw/panel.hpp"
#include "autoviz/ui/channels/channels_panel.hpp"
#include "autoviz/ui/record/record_panel.hpp"
#include "autoviz/ui/evidence/evidence_panel.hpp"
#include "autoviz/ui/inspector/selection_panel.hpp"
#include "autoviz/ui/inspector/tool_properties_panel.hpp"
#include "autoviz/ui/tf_tree/panel.hpp"
#include "autoviz/ui/publish/publish_panel.hpp"
#include "autoviz/ui/map/map_panel.hpp"
#include "autoviz/ui/channel_graph/channel_graph_panel.hpp"
#include "autoviz/ui/service/service_panel.hpp"
#include "autoviz/ui/time/panel.hpp"
#include "autoviz/ui/panel/name_map.hpp"
#include "autoviz/ui/viewport_hud.hpp"
#include "autoviz/ui/views/panel.hpp"
#include "autoviz/commsgs/message_type_utils.hpp"
#include "autoviz/ui/record_drop_overlay.hpp"
#include "autoviz/ui/theme/tokens.hpp"

#include <QAbstractAnimation>
#include <QEasingCurve>
#include <QGraphicsOpacityEffect>
#include <QPropertyAnimation>


namespace autoviz {

using frame_session_detail::kForcePaintAfterOccludedTicks;
using frame_session_detail::SanitizedIdleDivisor;
void FrameSession::markViewportDirty() { viewport_dirty_ = true; }

void FrameSession::setIdleRenderDivisor(int divisor) {
  idle_render_divisor_ = SanitizedIdleDivisor(divisor);
  viewport_dirty_ = true;
}

void FrameSession::onRenderTick() {
  if (app_inactive_) {
    // Repaint once on resume even if nothing else changed.
    viewport_dirty_ = true;
    return;
  }
  if (frame_->isMinimized()) {
    viewport_dirty_ = true;
    return;
  }
  const QWindow* window = frame_->windowHandle();
  if (window != nullptr && !window->isExposed()) {
    // Covered / on another virtual desktop. Keep the dirty flag so the first
    // exposed tick repaints, but stop drawing until then.
    viewport_dirty_ = true;
    if (++occluded_ticks_ < kForcePaintAfterOccludedTicks) {
      return;
    }
    occluded_ticks_ = 0;
  } else {
    occluded_ticks_ = 0;
  }
  const qint64 elapsed_ms = render_elapsed_.restart();
  // After backgrounding, elapsed can be huge; clamp so FPS/view ticks don't
  // jump and we don't try to "catch up" expensive work in one frame.
  const float delta_seconds =
      static_cast<float>(std::min<qint64>(elapsed_ms, 100)) / 1000.f;
  frame_->viewport_->syncToolContext();
  frame_->manager_->update();

  // Idle throttling: distinguish "needs a repaint" from "just tick".
  const bool dirty = viewport_dirty_;
  viewport_dirty_ = false;
  if (dirty) {
    idle_render_skip_ = 0;
  } else if (++idle_render_skip_ % idle_render_divisor_ != 0) {
    frame_->panels_->tickVisibleImagePanels();
    frame_->chrome_->updateFps();
    return;
  }

  // Per-3D-View overlay draw — split panes must not share parent geometry.
  const ViewportPanelEntry* active_entry =
      frame_->viewport_->activeViewportEntry();
  frame_->viewport_->forEachViewportPanel([this, active_entry](
                                              ViewportPanelEntry& entry) {
    if (entry.visibility_bit == 0 || entry.scene_overlay == nullptr) {
      return;
    }
    const bool is_active =
        active_entry != nullptr && entry.dock == active_entry->dock;
    rendering::OgreSceneHost* host =
        entry.ogre_viewport != nullptr ? entry.ogre_viewport->ogreSceneHost()
                                       : nullptr;
    frame_->manager_->drawViewport(entry.visibility_bit, *entry.scene_overlay,
                                   host, is_active);
    if (is_active) {
      if (rendering::ViewController* controller =
              frame_->viewport_->activeViewController()) {
        controller->appendFocalShape(entry.scene_overlay.get());
      }
    }
  });
  frame_->viewport_->viewportTick(delta_seconds);
  frame_->panels_->tickVisibleImagePanels();
  frame_->chrome_->updateFps();
}

void FrameSession::onRefreshTick() {
  if (app_inactive_) {
    return;
  }
  // Defer ALL channel/UI churn until well after Ogre first paint. At ~8s the
  // previous path called ensureDisplaysForRecordChannels on a live Argus
  // session (displays already from .autoviz) → duplicate readers + tree/icon
  // rebuild under paint → heap corruption (QIcon/QCss/Ogre Material::clone).
  static QElapsedTimer startup_grace;
  if (!startup_grace.isValid()) {
    startup_grace.start();
  }
  if (startup_grace.elapsed() < 20000) {
    return;
  }
  frame_->manager_->refreshChannelList();
  // Auto-ensure only while a record is open. Live /tf publishers must not
  // re-create readers for channels already bound by the session config.
  if (frame_->manager_->playback().isPlaying() ||
      frame_->manager_->playback().isPaused()) {
    static std::unordered_set<std::string> live_displays_seen;
    std::unordered_map<std::string, std::string> fresh_channels;
    for (const auto& channel : frame_->manager_->channels()) {
      if (channel.message_type.empty() ||
          live_displays_seen.count(channel.channel_name) != 0) {
        continue;
      }
      fresh_channels.emplace(channel.channel_name, channel.message_type);
    }
    if (!fresh_channels.empty()) {
      frame_->manager_->ensureDisplaysForRecordChannels(fresh_channels);
      for (const auto& entry : fresh_channels) {
        live_displays_seen.insert(entry.first);
      }
    }
  }
  if (frame_->panels_->displays_panel_ != nullptr) {
    frame_->panels_->displays_panel_->refreshStatus();
  }
  if (frame_->panels_->views_panel_ != nullptr) {
    frame_->panels_->views_panel_->refreshFrameList();
  }
  frame_->chrome_->updateChannelList();
  if (frame_->panels_->tf_tree_panel_ != nullptr) {
    frame_->panels_->tf_tree_panel_->refresh();
  }
  frame_->chrome_->updateStatusBar();
}

void FrameSession::syncOffscreenPause() {
  const bool offscreen = frame_->isMinimized();
  if (offscreen == app_inactive_) {
    return;
  }
  app_inactive_ = offscreen;
  // Keep accepting live messages while Autoviz is open. Mouse leaving the
  // Image/3D native windows must not freeze Image, 3D, or TF.
  integration::MessageQueue::setAcceptIncoming(true);
  if (offscreen) {
    setRenderingPaused(true);
    return;
  }
  render_elapsed_.restart();
  if (frame_->panels_->displays_panel_ != nullptr) {
    frame_->panels_->displays_panel_->setLiveUpdatesPaused(false);
  }
  if (frame_->panels_->tf_tree_panel_ != nullptr) {
    frame_->panels_->tf_tree_panel_->setPaused(false);
  }
  setRenderingPaused(false);
}

void FrameSession::setRenderingPaused(bool paused) {
  if (paused) {
    render_timer_.stop();
    refresh_timer_.stop();
  } else {
    frame_->viewport_->applyTargetFrameRate(frame_->manager_->targetFrameRate());
    if (!refresh_timer_.isActive()) {
      refresh_timer_.setTimerType(Qt::PreciseTimer);
      refresh_timer_.start(1000);
    }
  }
}


}  // namespace autoviz
