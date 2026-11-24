/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/frame_session.hpp"
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

void FrameSession::setupRecordDropOverlay() {
  record_drop_overlay_ = new RecordDropOverlay(frame_);
  layoutRecordDropOverlay();
}

void FrameSession::layoutRecordDropOverlay() {
  if (record_drop_overlay_ == nullptr) {
    return;
  }
  record_drop_overlay_->setGeometry(frame_->rect());
  record_drop_overlay_->raise();
}

void FrameSession::showRecordDropOverlay(bool visible) {
  if (record_drop_overlay_ == nullptr) {
    return;
  }
  if (visible) {
    layoutRecordDropOverlay();
    record_drop_overlay_->show();
    record_drop_overlay_->raise();
  } else {
    record_drop_overlay_->hide();
  }
}

bool FrameSession::handleEvidenceMime(const QMimeData* mime, bool drop) {
  if (mime == nullptr || !mime->hasUrls()) {
    return false;
  }
  QString path;
  for (const QUrl& url : mime->urls()) {
    if (!url.isLocalFile()) {
      continue;
    }
    const QString candidate = url.toLocalFile();
    if (candidate.endsWith(QLatin1String(".sidecar.json"))) {
      path = candidate;
      break;
    }
  }
  if (path.isEmpty()) {
    return false;
  }
  if (!drop) {
    showRecordDropOverlay(true);
    return true;
  }
  showRecordDropOverlay(false);
  frame_->layout_->showPanelByObjectName(QStringLiteral("EvidenceDock"));
  for (PanelDockWidget* dock : frame_->findChildren<PanelDockWidget*>()) {
    if (dock == nullptr ||
        frame_->panels_->panelTypeId(dock) != QLatin1String("EvidenceDock")) {
      continue;
    }
    auto* panel = qobject_cast<EvidencePanel*>(dock->widget());
    if (panel == nullptr) {
      continue;
    }
    if (!panel->openSidecarFile(path)) {
      QMessageBox::warning(frame_, frame_->tr("Evidence"),
                           frame_->tr("Failed to parse sidecar:\n%1").arg(path));
    }
    break;
  }
  return true;
}

bool FrameSession::handleRecordMime(const QMimeData* mime, bool drop) {
  const QStringList paths = LocalRecordSourcePaths(mime);
  if (paths.isEmpty()) {
    return false;
  }
  if (drop) {
    showRecordDropOverlay(false);
    openRecordFile(paths.front());
  } else {
    showRecordDropOverlay(true);
  }
  return true;
}

void FrameSession::onOpenRecord() {
  const QString path = QFileDialog::getOpenFileName(
      frame_, frame_->tr("Open Record"), QString(),
      frame_->tr("Autolink Record (*.record);;Legacy Bag (*.bag);;MCAP (*.mcap);;All Files (*)"));
  if (!path.isEmpty()) {
    openRecordFile(path);
  }
}

bool FrameSession::openRecordFile(const QString& path) {
  if (path.isEmpty() || frame_->manager_ == nullptr) {
    return false;
  }
  integration::PlaybackController& playback = frame_->manager_->playback();
  playback.stop();

  const RecordSourceKind kind = ClassifyRecordSource(path);
  OpenRecordResult result = OpenRecordSource(&playback, path);
  if (!result.ok &&
      (kind == RecordSourceKind::kBag || kind == RecordSourceKind::kMcap)) {
    ImportRecordDialog dialog(&playback, frame_);
    dialog.setSourcePath(path);
    if (dialog.exec() != QDialog::Accepted || !dialog.recordOpened()) {
      return false;
    }
    result.ok = true;
  }
  if (!result.ok) {
    QMessageBox::warning(frame_, frame_->tr("Open Record"), result.error);
    return false;
  }

  // Bind Displays to record channels and let the UI thread create readers
  // before the player writers start (avoids write-failed storms / races).
  const int added = frame_->manager_->ensureDisplaysForRecordChannels(
      playback.channelTypes());
  if (frame_->panels_ != nullptr &&
      frame_->panels_->displays_panel_ != nullptr) {
    frame_->panels_->displays_panel_->refreshStatus();
  }
  frame_->manager_->update();
  QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);

  // Skip seekTo/preview before first play: previewAtLocked creates ephemeral
  // Writers then destroys them, racing topology callbacks into freed Writer*.
  if (!playback.play(1.0, false)) {
    QMessageBox::warning(
        frame_, frame_->tr("Open Record"),
        frame_->tr("Opened %1 but failed to start playback.")
            .arg(QFileInfo(result.record_path).fileName()));
    return false;
  }
  if (frame_->panels_ != nullptr) {
    if (frame_->panels_->record_dock_ == nullptr) {
      frame_->panels_->record_dock_ = frame_->panels_->createRecordPanelDock();
      frame_->layout_->addSidebarDock(frame_->panels_->record_dock_,
                                      Qt::RightDockWidgetArea);
      if (frame_->panels_->views_dock_ != nullptr) {
        frame_->tabifyDockWidget(frame_->panels_->views_dock_,
                                 frame_->panels_->record_dock_);
      }
    }
    frame_->layout_->ensureSidebarDockAttached(frame_->panels_->record_dock_);
    frame_->panels_->record_dock_->show();
    frame_->panels_->record_dock_->raise();
    if (frame_->panels_->record_panel_ != nullptr) {
      frame_->panels_->record_panel_->reloadFromPlayback();
    }
  }
  if (frame_->statusBar() != nullptr) {
    QString message =
        frame_->tr("Playing %1").arg(QFileInfo(result.record_path).fileName());
    if (added > 0) {
      message +=
          frame_->tr(" · added %1 display(s)").arg(added);
    }
    frame_->statusBar()->showMessage(message, 5000);
  }
  return true;
}

void FrameSession::dragEnterEvent(QDragEnterEvent* event) {
  if (handleEvidenceMime(event->mimeData(), false) ||
      handleRecordMime(event->mimeData(), false)) {
    event->acceptProposedAction();
    return;
  }
}

void FrameSession::dragMoveEvent(QDragMoveEvent* event) {
  if (handleEvidenceMime(event->mimeData(), false) ||
      handleRecordMime(event->mimeData(), false)) {
    event->acceptProposedAction();
    return;
  }
}

void FrameSession::dragLeaveEvent(QDragLeaveEvent* event) {
  showRecordDropOverlay(false);
}

void FrameSession::dropEvent(QDropEvent* event) {
  if (handleEvidenceMime(event->mimeData(), true) ||
      handleRecordMime(event->mimeData(), true)) {
    event->acceptProposedAction();
    return;
  }
}

bool FrameSession::eventFilter(QObject* watched, QEvent* event) {
  auto* widget = qobject_cast<QWidget*>(watched);
  if (widget == nullptr ||
      (widget != frame_ && widget != record_drop_overlay_ &&
       !frame_->isAncestorOf(widget))) {
    return false; // base handled by VisualizationFrame
  }

  switch (event->type()) {
    case QEvent::DragEnter:
    case QEvent::DragMove: {
      auto* drag = static_cast<QDragMoveEvent*>(event);
      if (handleEvidenceMime(drag->mimeData(), false) ||
          handleRecordMime(drag->mimeData(), false)) {
        drag->acceptProposedAction();
        return true;
      }
      break;
    }
    case QEvent::Drop: {
      auto* drop = static_cast<QDropEvent*>(event);
      if (handleEvidenceMime(drop->mimeData(), true) ||
          handleRecordMime(drop->mimeData(), true)) {
        drop->acceptProposedAction();
        return true;
      }
      break;
    }
    case QEvent::DragLeave:
      if (!frame_->rect().contains(frame_->mapFromGlobal(QCursor::pos()))) {
        showRecordDropOverlay(false);
      }
      break;
    default:
      break;
  }
  return false; // base handled by VisualizationFrame
}


}  // namespace autoviz
