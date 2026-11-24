/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/frame_layout.hpp"
#include "autoviz/ui/frame.hpp"
#include <QObject>
#include "autoviz/common/selection.hpp"
#include <algorithm>
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
#include <QMainWindow>
#include <QDragEnterEvent>
#include <QDragLeaveEvent>
#include <QDragMoveEvent>
#include <QDropEvent>
#include <QEvent>
#include <QFileDialog>
#include <QFileInfo>
#include <QListWidget>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QMimeData>
#include <QPainter>
#include <QPalette>
#include <QPen>
#include <QPointer>
#include <QResizeEvent>
#include <QScreen>
#include <QSettings>
#include <QShortcut>
#include <QSignalBlocker>
#include <QSizeGrip>
#include <QStatusBar>
#include <QSizePolicy>
#include <QTimer>
#include <QCursor>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QVariant>
#include <QTabWidget>
#include <QToolBar>
#include <QToolButton>
#include <QGridLayout>
#include <QQuaternion>
#include <QWindow>
#include <QVBoxLayout>
#include "autoviz/common/display_property.hpp"
#include "autoviz/common/tool_manager.hpp"
#include "autoviz/common/interaction/message_queue.hpp"
#include "autoviz/rendering/ogre_render_window.hpp"
#include "autoviz/rendering/gpu_capabilities.hpp"
#include "autoviz/rendering/view_controller.hpp"
#include "autoviz/common/view_state_io.hpp"
#include "autoviz/ui/viewport_panel.hpp"

#include <functional>
#include <memory>
#include <optional>
#include "autoviz/ui/panel/add_dialog.hpp"
#include "autoviz/ui/app/settings_dialog.hpp"
#include "autoviz/ui/app/preferences.hpp"
#include "autoviz/ui/theme/application.hpp"
#include "autoviz/ui/app/translation.hpp"
#include "autoviz/ui/panel/catalog.hpp"
#include "autoviz/ui/panel/context_menu.hpp"
#include "autoviz/ui/panel/content_only_float.hpp"
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


namespace autoviz {

void FrameLayout::updateMosaicDragGhost(PanelDockWidget* /*dock*/,
                                        const QPoint& /*global_pos*/) {
  // No top-level drag ghost. Drop feedback is MainPanelHost::setDropPreview.
  clearMosaicDragGhost();
}

void FrameLayout::clearMosaicDragGhost() {
  if (mosaic_drag_ghost_.isNull()) {
    return;
  }
  QWidget* ghost = mosaic_drag_ghost_.data();
  mosaic_drag_ghost_.clear();
  if (ghost != nullptr) {
    ghost->hide();
    ghost->deleteLater();
  }
}

namespace {

/**
 * qApp filter: after park/hide the title bar no longer sees Move/Release.
 * Must not depend on a translucent overlay hit-test (X11 drops those events).
 */
class MosaicDragMouseFilter final : public QObject {
 public:
  explicit MosaicDragMouseFilter(FrameLayout* layout, QObject* parent)
      : QObject(parent), layout_(layout) {}

  bool eventFilter(QObject* watched, QEvent* event) override {
    if (layout_ == nullptr || !layout_->isCenterDockDragging()) {
      return QObject::eventFilter(watched, event);
    }
    PanelDockWidget* dock = layout_->mosaicDragDock();
    if (dock == nullptr || event == nullptr) {
      return QObject::eventFilter(watched, event);
    }
    switch (event->type()) {
      case QEvent::MouseMove: {
        auto* mouse = static_cast<QMouseEvent*>(event);
        layout_->updateMainPanelDropPreview(
            dock, mouse->globalPosition().toPoint());
        break;
      }
      case QEvent::MouseButtonRelease: {
        auto* mouse = static_cast<QMouseEvent*>(event);
        if (mouse->button() == Qt::LeftButton) {
          const QPoint pos = mouse->globalPosition().toPoint();
          layout_->removeMosaicDragMouseFilter();
          dock->endTitleDrag(pos);
          return true;
        }
        break;
      }
      case QEvent::KeyPress: {
        auto* key = static_cast<QKeyEvent*>(event);
        if (key->key() == Qt::Key_Escape) {
          const QPoint pos = QCursor::pos();
          layout_->removeMosaicDragMouseFilter();
          dock->endTitleDrag(pos);
          return true;
        }
        break;
      }
      default:
        break;
    }
    return QObject::eventFilter(watched, event);
  }

 private:
  FrameLayout* layout_ = nullptr;
};

}  // namespace

PanelDockWidget* FrameLayout::mosaicDragDock() const {
  return mosaic_drag_dock_.data();
}

void FrameLayout::installMosaicDragMouseFilter(PanelDockWidget* dock) {
  removeMosaicDragMouseFilter();
  mosaic_drag_dock_ = dock;
  if (dock == nullptr || frame_ == nullptr) {
    return;
  }
  auto* filter = new MosaicDragMouseFilter(this, frame_);
  mosaic_drag_mouse_filter_ = filter;
  qApp->installEventFilter(filter);
  // Seed preview under the current cursor immediately.
  updateMainPanelDropPreview(dock, QCursor::pos());
}

void FrameLayout::removeMosaicDragMouseFilter() {
  if (mosaic_drag_mouse_filter_ != nullptr) {
    qApp->removeEventFilter(mosaic_drag_mouse_filter_);
    mosaic_drag_mouse_filter_->deleteLater();
    mosaic_drag_mouse_filter_ = nullptr;
  }
  mosaic_drag_dock_.clear();
}

void FrameLayout::beginMainPanelDrag(PanelDockWidget* dock) {
  if (dock == nullptr || main_panel_host_ == nullptr || !isMainPanel(dock)) {
    return;
  }
  center_manual_layout_ = true;
  ++center_tile_epoch_;
  center_tile_pending_ = false;
  // Foxglove: pull the dragged leaf out; every other pane stays live/unchanged.
  // Only the drop-half preview chrome changes visually.
  frame_->viewport_->scheduleSettleReinstallAll(/*delay_ms=*/-1);
  if (frame_->panels_ != nullptr &&
      frame_->panels_->views_panel_ != nullptr) {
    frame_->panels_->views_panel_->setViewController(nullptr);
  }
  // Park ONLY the dragged viewport — never charcoal-blank the rest of the mosaic.
  frame_->viewport_->forEachViewportPanel([this, dock](ViewportPanelEntry& entry) {
    if (entry.dock != dock) {
      return;
    }
    if (entry.hud_overlay != nullptr) {
      entry.hud_overlay->setHudVisible(false);
      entry.hud_overlay->hide();
    }
    frame_->viewport_->parkRenderWindowInEntry(entry);
    if (entry.ogre_viewport != nullptr) {
      entry.ogre_viewport->hideNativeSurface();
    }
  });
  if (main_panel_host_->hostsPanel(dock)) {
    main_panel_host_->removePanel(dock);
  }
  main_panel_host_->scrubEmptyPanes();
  main_panel_host_->syncHorizontalDockLayout();
  {
    const QSignalBlocker blocker(dock);
    if (dock->isFloating()) {
      dock->setFloating(false);
    }
    dock->hide();
  }
  // Remount sibling 3D Views after the leaf expands into the freed space.
  frame_->viewport_->forEachViewportPanel([dock](ViewportPanelEntry& entry) {
    if (entry.dock == dock) {
      return;
    }
    if (entry.host != nullptr) {
      entry.host->setStyleSheet(QString());
      entry.host->show();
    }
    if (entry.widget != nullptr && entry.host != nullptr &&
        entry.host->width() >= 2 && entry.host->height() >= 2) {
      entry.widget->setGeometry(entry.host->rect());
      entry.widget->show();
    }
    if (entry.ogre_viewport != nullptr) {
      entry.ogre_viewport->showNativeSurface();
    }
    if (entry.hud_overlay != nullptr) {
      entry.hud_overlay->show();
      entry.hud_overlay->setHudVisible(true);
    }
  });
  clearMosaicDragGhost();
  main_panel_host_->beginMosaicDropSession();
  installMosaicDragMouseFilter(dock);
  syncCenterLayout();
  updateMainPanelDropPreview(dock, QCursor::pos());
}

void FrameLayout::updateMainPanelDropPreview(PanelDockWidget* dock,
                                             const QPoint& global_pos) {
  if (dock == nullptr || main_panel_host_ == nullptr || !isMainPanel(dock)) {
    return;
  }
  updateMosaicDragGhost(dock, global_pos);
  main_panel_host_->setDropPreview(global_pos, dock, true);
}

bool FrameLayout::finishMainPanelDrag(PanelDockWidget* dock,
                                      const QPoint& global_pos) {
  if (dock == nullptr || main_panel_host_ == nullptr || !isMainPanel(dock)) {
    removeMosaicDragMouseFilter();
    if (main_panel_host_ != nullptr) {
      main_panel_host_->endMosaicDropSession();
    }
    return false;
  }
  removeMosaicDragMouseFilter();
  clearMosaicDragGhost();
  // Capture drop target while preview edge is still meaningful, then tear down
  // the charcoal/purple session chrome.
  QDockWidget* target = nullptr;
  Qt::Edge edge = Qt::LeftEdge;
  bool has_edge =
      main_panel_host_->hitTestDrop(global_pos, dock, &target, &edge);
  if (!has_edge && main_panel_host_->hostedPanels().isEmpty()) {
    // Empty mosaic — host-level edge for the returning sole leaf (no target).
    const QPoint local = main_panel_host_->mapFromGlobal(global_pos);
    const QRect host_rect = main_panel_host_->rect();
    if (host_rect.contains(local)) {
      const double w = static_cast<double>(qMax(1, host_rect.width()));
      const double h = static_cast<double>(qMax(1, host_rect.height()));
      const double x_rate =
          static_cast<double>(local.x() - host_rect.left()) / w;
      const double y_rate =
          static_cast<double>(local.y() - host_rect.top()) / h;
      const double dist_l = x_rate;
      const double dist_r = 1.0 - x_rate;
      const double dist_t = y_rate;
      const double dist_b = 1.0 - y_rate;
      const double best =
          std::min(std::min(dist_l, dist_r), std::min(dist_t, dist_b));
      if (best == dist_l) {
        edge = Qt::LeftEdge;
      } else if (best == dist_r) {
        edge = Qt::RightEdge;
      } else if (best == dist_t) {
        edge = Qt::TopEdge;
      } else {
        edge = Qt::BottomEdge;
      }
    }
  }
  main_panel_host_->endMosaicDropSession();

  const bool was_suppressing = suppress_center_tile_;
  suppress_center_tile_ = true;

  auto* target_panel = qobject_cast<PanelDockWidget*>(target);
  const bool is_viewport =
      frame_->panels_->panelTypeId(dock) == QLatin1String("ViewportDock") ||
      (target_panel != nullptr &&
       frame_->panels_->panelTypeId(target_panel) ==
           QLatin1String("ViewportDock"));

  auto park_all_viewports = [this]() {
    frame_->viewport_->forEachViewportPanel([this](ViewportPanelEntry& entry) {
      frame_->viewport_->parkRenderWindowInEntry(entry);
    });
  };

  bool accepted = false;
  if (has_edge && target != nullptr && target != dock) {
    if (is_viewport) {
      park_all_viewports();
    }
    configureMainPanelDock(dock);
    accepted = main_panel_host_->dockPanelAt(dock, target, edge);
  } else if (!main_panel_host_->hostsPanel(dock)) {
    // Prefer edge snap against any remaining leaf — never blind-append onto a
    // Vertical root (that only stacks and makes 左右 look broken).
    const QList<QDockWidget*> hosted = main_panel_host_->hostedPanels();
    QDockWidget* fallback_target = target;
    if (fallback_target == nullptr) {
      for (QDockWidget* candidate : hosted) {
        if (candidate != nullptr && candidate != dock) {
          fallback_target = candidate;
          break;
        }
      }
    }
    if (fallback_target != nullptr) {
      if (is_viewport) {
        park_all_viewports();
      }
      configureMainPanelDock(dock);
      if (!has_edge) {
        const QPoint local = main_panel_host_->mapFromGlobal(global_pos);
        const QRect host_rect = main_panel_host_->rect();
        if (host_rect.contains(local)) {
          const double w = static_cast<double>(qMax(1, host_rect.width()));
          const double h = static_cast<double>(qMax(1, host_rect.height()));
          const double x_rate =
              static_cast<double>(local.x() - host_rect.left()) / w;
          const double y_rate =
              static_cast<double>(local.y() - host_rect.top()) / h;
          const double dist_l = x_rate;
          const double dist_r = 1.0 - x_rate;
          const double dist_t = y_rate;
          const double dist_b = 1.0 - y_rate;
          const double best =
              std::min(std::min(dist_l, dist_r), std::min(dist_t, dist_b));
          if (best == dist_l) {
            edge = Qt::LeftEdge;
          } else if (best == dist_r) {
            edge = Qt::RightEdge;
          } else if (best == dist_t) {
            edge = Qt::TopEdge;
          } else {
            edge = Qt::BottomEdge;
          }
        } else {
          edge = Qt::RightEdge;
        }
      }
      accepted = main_panel_host_->dockPanelAt(dock, fallback_target, edge);
    } else {
      if (is_viewport) {
        park_all_viewports();
      }
      configureMainPanelDock(dock);
      if (dock->isFloating()) {
        dock->setFloating(false);
      }
      main_panel_host_->addPanel(dock);
      accepted = main_panel_host_->hostsPanel(dock);
    }
  }

  if (!accepted && !main_panel_host_->hostsPanel(dock)) {
    if (is_viewport) {
      park_all_viewports();
    }
    configureMainPanelDock(dock);
    if (dock->isFloating()) {
      dock->setFloating(false);
    }
    main_panel_host_->addPanel(dock);
    accepted = main_panel_host_->hostsPanel(dock);
  }

  // Always clear drop wash — leftover half-pane overlay reads as purple 黑边.
  main_panel_host_->setDropPreview(global_pos, dock, false);

  if (accepted) {
    center_manual_layout_ = true;
    if (dock->isFloating()) {
      dock->setFloating(false);
    }
    pre_float_mosaic_layout_.remove(dock->objectName());
    dock->show();
    dock->raise();
    wireMainPanelExpandTracking(dock);
    main_panel_host_->scrubEmptyPanes();
    main_panel_host_->syncHorizontalDockLayout();
    if (auto* image = qobject_cast<image::ImagePanel*>(dock->widget())) {
      image->setFloatButtonChecked(false);
    }
    if (is_viewport) {
      frame_->viewport_->setViewportFloatButtonChecked(dock, false);
      if (frame_->panels_->panelTypeId(dock) == QLatin1String("ViewportDock")) {
        frame_->viewport_->setActiveViewportDock(dock);
      }
      dock->setStyleSheet(QString());
      // Force GL recreate at final pane size — park left a charcoal plate and
      // stale GLX geometry paints black/grey blocks after mosaic snap.
      frame_->viewport_->forEachViewportPanel([](ViewportPanelEntry& entry) {
        entry.needs_gl_recreate = true;
      });
    }
    syncCenterLayout();
    // Wait for splitter sizes to commit before remounting GLX.
    frame_->viewport_->scheduleSettleReinstallAll(
        80, /*close_broken_duplicates=*/true);
    if (is_viewport) {
      QTimer::singleShot(0, frame_, [this]() {
        if (main_panel_host_ != nullptr) {
          main_panel_host_->syncHorizontalDockLayout();
        }
      });
      QTimer::singleShot(160, frame_, [this]() {
        if (frame_ == nullptr || frame_->viewport_ == nullptr) {
          return;
        }
        if (main_panel_host_ != nullptr) {
          main_panel_host_->syncHorizontalDockLayout();
          main_panel_host_->setDropPreview(QPoint(), nullptr, false);
        }
        frame_->viewport_->forEachViewportPanel([](ViewportPanelEntry& entry) {
          entry.needs_gl_recreate = true;
        });
        frame_->viewport_->scheduleSettleReinstallAll(
            0, /*close_broken_duplicates=*/false);
      });
    }
  } else {
    main_panel_host_->scrubEmptyPanes();
    main_panel_host_->syncHorizontalDockLayout();
    if (is_viewport) {
      frame_->viewport_->forEachViewportPanel([](ViewportPanelEntry& entry) {
        entry.needs_gl_recreate = true;
      });
    }
    frame_->viewport_->scheduleSettleReinstallAll(
        48, /*close_broken_duplicates=*/true);
    syncCenterLayout();
  }
  suppress_center_tile_ = was_suppressing;
  return accepted;
}


}  // namespace autoviz
