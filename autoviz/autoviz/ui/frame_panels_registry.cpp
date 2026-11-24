/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/frame_panels.hpp"
#include "autoviz/ui/frame.hpp"
#include <QObject>
#include "autoviz/ui/frame_detail.hpp"
#include "autoviz/common/selection.hpp"
#include "autoviz/plugins/displays/display_group.hpp"
#include "autoviz/ui/displays/panel.hpp"
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
#include <QPen>
#include <QPointer>
#include <QResizeEvent>
#include <QScreen>
#include <QSettings>
#include <QShortcut>
#include <QSignalBlocker>
#include <QStatusBar>
#include <QSizePolicy>
#include <QTimer>
#include <QCursor>
#include <QVariant>
#include <QTabWidget>
#include <QToolBar>
#include <QToolButton>
#include <QGridLayout>
#include <QQuaternion>
#include <QVBoxLayout>
#include "autoviz/common/display_property.hpp"
#include "autoviz/common/tool_manager.hpp"
#include "autoviz/common/interaction/message_queue.hpp"
#include "autoviz/rendering/ogre_render_window.hpp"
#include "autoviz/rendering/gpu_capabilities.hpp"
#include "autoviz/rendering/view_controller.hpp"
#include "autoviz/common/view_state_io.hpp"
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
#include "autoviz/ui/image/image_view_widget.hpp"
#include "autoviz/ui/panel_host.hpp"
#include "autoviz/ui/panel/role.hpp"
#include "autoviz/ui/inspector/property_panel.hpp"
#include "autoviz/ui/dialog/import_record.hpp"
#include "autoviz/ui/dialog/record_open.hpp"
#include "autoviz/ui/plot/io/plot_config_io.hpp"
#include "autoviz/ui/publish/publish_config_io.hpp"
#include "autoviz/ui/service/service_config_io.hpp"
#include "autoviz/ui/teleop/teleop_config_io.hpp"
#include "autoviz/ui/map/map_config_io.hpp"
#include "autoviz/ui/image/image_config_io.hpp"
#include "autoviz/ui/plot/panel/plot_panel.hpp"
#include "autoviz/ui/table/table_config_io.hpp"
#include "autoviz/ui/table/table_panel.hpp"
#include "autoviz/ui/teleop/teleop_panel.hpp"
#include "autoviz/ui/navigation/navigation_panel.hpp"
#include "autoviz/ui/raw/panel.hpp"
#include "autoviz/ui/channels/channels_panel.hpp"
#include "autoviz/ui/record/record_panel.hpp"
#include "autoviz/ui/evidence/evidence_panel.hpp"
#include "autoviz/ui/inspector/selection_panel.hpp"
#include "autoviz/ui/inspector/tool_properties_panel.hpp"
#include "autoviz/ui/tf_tree/panel.hpp"
#include "autoviz/ui/panel/operator_tools_panel.hpp"
#include "autoviz/ui/publish/publish_panel.hpp"
#include "autoviz/ui/map/map_panel.hpp"
#include "autoviz/ui/channel_graph/channel_graph_panel.hpp"
#include "autoviz/ui/channel_graph/channel_graph_config_io.hpp"
#include "autoviz/ui/service/service_panel.hpp"
#include "autoviz/ui/time/panel.hpp"
#include "autoviz/ui/panel/name_map.hpp"
#include "autoviz/ui/viewport_hud.hpp"
#include "autoviz/ui/views/panel.hpp"
#include "autoviz/commsgs/message_type_utils.hpp"


namespace autoviz {

void FramePanels::registerPanelDock(PanelDockWidget* dock) {
  if (dock == nullptr || dock->property("panelDockRegistered").toBool()) {
    return;
  }
  dock->setProperty("panelDockRegistered", true);
  IconLoader::applyDockPanelChrome(dock, panelTypeId(dock));
  QObject::connect(dock, &PanelDockWidget::activated, frame_, [this, dock]() {
    frame_->layout_->last_active_dock_ = dock;
    if (panelTypeId(dock) == QLatin1String("ViewportDock")) {
      frame_->viewport_->setActiveViewportDock(dock);
      return;
    }
    if (panelTypeId(dock) == QLatin1String("PlotDock")) {
      if (auto* panel = qobject_cast<plot::PlotPanel*>(dock->widget())) {
        setActivePlotPanel(panel);
      }
    } else if (panelTypeId(dock) == QLatin1String("ImageDock")) {
      if (auto* panel = qobject_cast<image::ImagePanel*>(dock->widget())) {
        setActiveImagePanel(panel);
      }
    } else if (panelTypeId(dock) == QLatin1String("TeleopDock")) {
      if (auto* panel = qobject_cast<teleop::TeleopPanel*>(dock->widget())) {
        setActiveTeleopPanel(panel);
      }
    } else if (panelTypeId(dock) == QLatin1String("PublishDock")) {
      if (auto* panel = qobject_cast<publish_panel::PublishPanel*>(dock->widget())) {
        setActivePublishPanel(panel);
      }
    } else if (panelTypeId(dock) == QLatin1String("ServiceDock")) {
      if (auto* panel = qobject_cast<service_panel::ServicePanel*>(dock->widget())) {
        setActiveServicePanel(panel);
      }
    } else if (panelTypeId(dock) == QLatin1String("MapDock")) {
      if (auto* panel = qobject_cast<map::MapPanel*>(dock->widget())) {
        setActiveMapPanel(panel);
      }
    }
  });
  QObject::connect(dock, &QDockWidget::visibilityChanged, frame_, &VisualizationFrame::onDockPanelVisibilityChange,

          Qt::UniqueConnection);
  // Panels-menu toggleViewAction calls show() while the dock may still be
  // parked under a hidden parent after close — isVisible() stays false and
  // visibilityChanged(true) never runs, so the center mosaic never re-attaches
  // (3D View / Plot / Map look like they refuse to open). Re-attach on toggle.
  if (frame_->layout_->isMainPanel(dock)) {
    if (QAction* toggle = dock->toggleViewAction()) {
      if (!toggle->property("mainPanelReopenWired").toBool()) {
        toggle->setProperty("mainPanelReopenWired", true);
        QObject::connect(toggle, &QAction::toggled, frame_, [this, dock](bool checked) {
          if (!checked || dock == nullptr ||
              dock->property("panelDisposed").toBool()) {
            return;
          }
          const QPointer<PanelDockWidget> guard(dock);
          QTimer::singleShot(0, frame_, [this, guard]() {
            if (!guard) {
              return;
            }
            PanelDockWidget* d = guard.data();
            if (QAction* action = d->toggleViewAction();
                action == nullptr || !action->isChecked()) {
              return;
            }
            // Already in the mosaic — do not retile (Ogre park/reinstall flicker).
            if (frame_->layout_->main_panel_host_ != nullptr &&
                frame_->layout_->main_panel_host_->hostsPanel(d) &&
                d->isVisible()) {
              frame_->layout_->last_active_dock_ = d;
              return;
            }
            frame_->layout_->suppress_center_tile_ = false;
            frame_->layout_->center_manual_layout_ = false;
            frame_->layout_->ensureMainPanelDockAttached(d);
            if (panelTypeId(d) == QLatin1String("ViewportDock")) {
              frame_->viewport_->ensureViewportPanelReady(d);
              frame_->viewport_->setActiveViewportDock(d);
            }
            if (!d->isVisible()) {
              d->show();
            }
            d->raise();
            frame_->layout_->last_active_dock_ = d;
            frame_->layout_->scheduleTileCenterPanels();
          });
        });
      }
    }
  }
  QObject::connect(frame_, &VisualizationFrame::fullScreenChange, dock,
          &PanelDockWidget::overrideVisibility, Qt::UniqueConnection);
  QObject::connect(dock, &QDockWidget::dockLocationChanged, frame_, &VisualizationFrame::markConfigModified);
  QObject::connect(dock, &PanelDockWidget::titleDragStarted, frame_, [this, dock]() {
    if (!frame_->layout_->isMainPanel(dock)) {
      return;
    }
    // Button-floated panels stay free windows — no mosaic undock / snap.
    if (dock->property("intentionalFloat").toBool()) {
      return;
    }
    ++frame_->layout_->center_dock_drag_count_;
    ++frame_->layout_->center_tile_epoch_;
    frame_->layout_->center_tile_pending_ = false;
    frame_->layout_->center_manual_layout_ = true;
    // Pull the leaf out of the mosaic so siblings expand; dock then floats.
    frame_->layout_->beginMainPanelDrag(dock);
  });
  QObject::connect(dock, &PanelDockWidget::titleDragMoved, frame_,
                   [this, dock](const QPoint& global_pos) {
                     if (!frame_->layout_->isMainPanel(dock)) {
                       return;
                     }
                     if (dock->property("intentionalFloat").toBool()) {
                       return;
                     }
                     frame_->layout_->updateMainPanelDropPreview(dock, global_pos);
                   });
  QObject::connect(
      dock, &PanelDockWidget::titleDragFinished, frame_,
      [this, dock](const QPoint& global_pos) {
        if (!frame_->layout_->isMainPanel(dock)) {
          return;
        }
        if (dock->property("intentionalFloat").toBool()) {
          // Free-float: stay above panels, never mosaic-吸附 on drop.
          frame_->session_->markConfigModified();
          return;
        }
        // Keep center_dock_drag_count_ until after finish so visibilityChanged
        // from show/reparent cannot scheduleTileCenterPanels mid-snap.
        frame_->layout_->finishMainPanelDrag(dock, global_pos);
        if (frame_->layout_->center_dock_drag_count_ > 0) {
          --frame_->layout_->center_dock_drag_count_;
        }
        frame_->layout_->syncCenterLayout();
        frame_->session_->markConfigModified();
      });
  QObject::connect(dock, &QDockWidget::topLevelChanged, frame_, [this](bool /*floating*/) {
    frame_->session_->markConfigModified();
  });
  QObject::connect(dock, &PanelDockWidget::closed, frame_, [this, dock]() {
    // Drag may have left retile suppressed; always clear on close.
    frame_->layout_->suppress_center_tile_ = false;
    frame_->layout_->center_dock_drag_count_ = 0;
    frame_->layout_->pre_float_mosaic_layout_.remove(dock->objectName());
    if (dock->property("intentionalFloat").toBool() ||
        IsContentOnlyFloat(dock)) {
      if (IsContentOnlyFloat(dock)) {
        SetContentOnlyFloatChrome(dock, false);
        if (auto* image = qobject_cast<image::ImagePanel*>(dock->widget())) {
          image->setContentOnlyMode(false);
        }
      }
      frame_->layout_->setIntentionalFloatChrome(dock, false);
      frame_->layout_->detachIntentionalFloatShell(dock);
    }
    if (QAction* toggle = dock->toggleViewAction()) {
      toggle->blockSignals(true);
      toggle->setChecked(false);
      toggle->blockSignals(false);
    }
    if (dock == displays_dock_) {
      frame_->layout_->displays_closed_by_user_ = true;
    }
    unregisterDeletePanelAction(dock);

    // Split duplicates (ViewportDock_2, PlotDock_2, …) are destroyed on close.
    // Canonical docks (objectName == typeId) stay alive so Panels menu keeps a
    // checkbox to reopen — including after every 3D View is closed.
    // Do NOT use viewport_dock_ pointer equality: removeViewportPanel used to
    // promote ViewportDock_2 to viewport_dock_, so closing the last Split pane
    // treated it as "primary" and the menu lost the sole 3D View toggle.
    const QString type_id = panelTypeId(dock);
    const bool drop_duplicate =
        frame_->layout_->isMainPanel(dock) &&
        panelTypeSupportsMultiInstance(type_id) &&
        dock->objectName() != type_id;
    // Viewport GLX / bookkeeping is torn down in the deferred lambda below
    // (park → removePanel → removeViewportPanel), never under closeEvent.
    if (drop_duplicate) {
      if (frame_->layout_->expanded_main_panel_dock_ == dock) {
        frame_->layout_->expanded_main_panel_dock_ = nullptr;
      }
      if (frame_->layout_->last_active_dock_ == dock) {
        frame_->layout_->last_active_dock_ = nullptr;
      }
      // Detach menu toggle before destroy — action is owned by the dock.
      if (QAction* toggle = dock->toggleViewAction()) {
        if (frame_->chrome_->panels_menu_ != nullptr) {
          frame_->chrome_->panels_menu_->removeAction(toggle);
        }
        frame_->chrome_->panels_menu_toggle_actions_.removeAll(QPointer<QAction>(toggle));
      }
      dock->setProperty("panelDisposed", true);
    }

    // Defer host removal / destroy / menu rebuild out of closeEvent.
    const QPointer<PanelDockWidget> dock_guard(dock);
    const QString closing_type = type_id;
    QTimer::singleShot(0, frame_, [this, dock_guard, drop_duplicate,
                                   closing_type]() {
      const bool closing_viewport =
          closing_type == QLatin1String("ViewportDock");
      const bool timer_was_active =
          frame_->session_ != nullptr &&
          frame_->session_->render_timer_.isActive();
      if (closing_viewport && timer_was_active) {
        frame_->session_->render_timer_.stop();
      }

      auto park_all_viewports = [this]() {
        if (frame_->viewport_ == nullptr) {
          return;
        }
        frame_->viewport_->forEachViewportPanel(
            [this](ViewportPanelEntry& entry) {
              frame_->viewport_->parkRenderWindowInEntry(entry);
            });
      };
      auto reinstall_surviving_viewports = [this]() {
        if (frame_->viewport_ == nullptr) {
          return;
        }
        // Park marked needs_gl_recreate — recreate on settle, not rebind-only
        // (rebind left transparent 3D Views after close / Panels retile).
        frame_->viewport_->scheduleSettleReinstallAll(48);
      };

      if (drop_duplicate && dock_guard) {
        if (closing_viewport) {
          park_all_viewports();
        }
        if (frame_->layout_->main_panel_host_ != nullptr &&
            frame_->layout_->main_panel_host_->hostsPanel(dock_guard.data())) {
          frame_->layout_->main_panel_host_->removePanel(dock_guard.data());
        }
        if (closing_viewport && frame_->viewport_ != nullptr) {
          // Destroy GLX after the dock left the mosaic tree.
          frame_->viewport_->removeViewportPanel(dock_guard.data());
        }
        if (QMainWindow* host = frame_->layout_->dockHostForPanel(dock_guard.data())) {
          if (host->dockWidgetArea(dock_guard.data()) != Qt::NoDockWidgetArea) {
            host->removeDockWidget(dock_guard.data());
          }
        }
        if (frame_->dockWidgetArea(dock_guard.data()) != Qt::NoDockWidgetArea) {
          frame_->removeDockWidget(dock_guard.data());
        }
        dock_guard->setProperty("panelClosing", false);
        // Rebuild after the dock (and its toggleViewAction) is gone.
        QObject::connect(dock_guard.data(), &QObject::destroyed, frame_, [this]() {
                  // Split-duplicate teardown must not leave Panels without a
                  // canonical 3D View checkbox.
                  if (frame_->findChild<PanelDockWidget*>(
                          QStringLiteral("ViewportDock")) == nullptr) {
                    PanelDockWidget* restored =
                        frame_->viewport_->createViewportPanelDock(
                            QStringLiteral("ViewportDock"),
                            /*create_render_window=*/false);
                    if (restored != nullptr) {
                      frame_->viewport_->viewport_dock_ = restored;
                      restored->hide();
                      frame_->layout_->configureMainPanelDock(restored);
                    }
                  }
                  frame_->chrome_->rebuildPanelsMenuToggles();
                  frame_->layout_->scheduleTileCenterPanels();
                  syncDeletePanelMenu();
                },
                static_cast<Qt::ConnectionType>(Qt::QueuedConnection |
                                                Qt::SingleShotConnection));
        dock_guard->deleteLater();
        reinstall_surviving_viewports();
        if (timer_was_active && frame_->session_ != nullptr &&
            !frame_->session_->app_inactive_) {
          frame_->session_->render_timer_.start();
        }
        return;
      }
      // Primary main panels (e.g. ViewportDock): detach from the mosaic so the
      // center can go empty / remaining panes can fill. Avoid a full retile —
      // that parks/reinstalls Ogre on every close and flickers surviving views.
      if (dock_guard && frame_->layout_->isMainPanel(dock_guard.data()) &&
          frame_->layout_->main_panel_host_ != nullptr &&
          frame_->layout_->main_panel_host_->hostsPanel(dock_guard.data())) {
        if (closing_viewport) {
          park_all_viewports();
        }
        frame_->layout_->main_panel_host_->removePanel(dock_guard.data());
        if (closing_viewport && frame_->viewport_ != nullptr) {
          frame_->viewport_->removeViewportPanel(dock_guard.data());
        }
        reinstall_surviving_viewports();
      } else if (dock_guard && closing_viewport && frame_->viewport_ != nullptr) {
        // Already detached (e.g. floating) — still drop GLX bookkeeping.
        frame_->viewport_->removeViewportPanel(dock_guard.data());
      }
      if (dock_guard) {
        dock_guard->setProperty("panelClosing", false);
      }
      if (frame_->layout_->main_panel_host_ != nullptr) {
        frame_->layout_->main_panel_host_->syncHorizontalDockLayout();
      }
      frame_->layout_->center_manual_layout_ = false;
      frame_->chrome_->rebuildPanelsMenuToggles();
      syncDeletePanelMenu();
      // Closing the last 3D View must leave a canonical ViewportDock so Panels
      // keeps one unchecked "3D View" toggle for reopen.
      if (frame_->findChild<PanelDockWidget*>(QStringLiteral("ViewportDock")) ==
          nullptr) {
        PanelDockWidget* restored =
            frame_->viewport_->createViewportPanelDock(
                QStringLiteral("ViewportDock"),
                /*create_render_window=*/false);
        if (restored != nullptr) {
          frame_->viewport_->viewport_dock_ = restored;
          restored->hide();
          frame_->layout_->configureMainPanelDock(restored);
          frame_->chrome_->rebuildPanelsMenuToggles();
        }
      }
      if (timer_was_active && frame_->session_ != nullptr &&
          !frame_->session_->app_inactive_) {
        frame_->session_->render_timer_.start();
      }
    });
    frame_->session_->markConfigModified();
  });
  frame_->layout_->wireMainPanelExpandTracking(dock);
  frame_->chrome_->registerPanelMenuToggle(dock);
}


}  // namespace autoviz
