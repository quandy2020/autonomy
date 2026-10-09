/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/frame_chrome.hpp"
#include <QActionGroup>
#include <QMenuBar>
#include <QStatusBar>
#include <QToolBar>
#include "autoviz/ui/frame.hpp"
#include <QObject>
#include "autoviz/ui/frame_detail.hpp"
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
#include "autoviz/integration/message_queue.hpp"
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
#include "autoviz/ui/plot/plot_config_io.hpp"
#include "autoviz/ui/publish/publish_config_io.hpp"
#include "autoviz/ui/image/image_config_io.hpp"
#include "autoviz/ui/plot/plot_panel.hpp"
#include "autoviz/ui/teleop/teleop_panel.hpp"
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
#include "autoviz/ui/viewport_toolbar.hpp"
#include "autoviz/ui/viewport_hud.hpp"
#include "autoviz/ui/views/panel.hpp"
#include "autoviz/commsgs/message_type_utils.hpp"

namespace autoviz {

FrameChrome::FrameChrome(VisualizationFrame* frame) : frame_(frame) {}

namespace {

QString PanelMenuTitleWithMnemonic(const QString& title, QSet<QChar>* used_mnemonics) {
  if (used_mnemonics == nullptr || title.isEmpty()) {
    return title;
  }
  for (int i = 0; i < title.size(); ++i) {
    const QChar ch = title.at(i);
    if (!ch.isLetter() || ch.unicode() >= 128) {
      continue;
    }
    const QChar upper = ch.toUpper();
    if (used_mnemonics->contains(upper)) {
      continue;
    }
    used_mnemonics->insert(upper);
    return title.left(i) + QLatin1Char('&') + title.mid(i);
  }
  return title;
}

/** Menu grouping for Panels toggles (layout commands stay above the separator). */
enum class PanelMenuGroup {
  kCoreViz = 0,
  kDataObserve = 1,
  kToolsFrames = 2,
  kOther = 3,
};

PanelMenuGroup PanelMenuGroupForType(const QString& type_id) {
  if (type_id == QLatin1String("ViewportDock") ||
      type_id == QLatin1String("DisplaysDock") ||
      type_id == QLatin1String("ImageDock") ||
      type_id == QLatin1String("ViewsDock")) {
    return PanelMenuGroup::kCoreViz;
  }
  if (type_id == QLatin1String("RecordDock") ||
      type_id == QLatin1String("ChannelBrowserDock") ||
      type_id == QLatin1String("PlotDock") ||
      type_id == QLatin1String("TableDock") ||
      type_id == QLatin1String("ChannelsDock") ||
      type_id == QLatin1String("ChannelGraphDock") ||
      type_id == QLatin1String("TimeDock")) {
    return PanelMenuGroup::kDataObserve;
  }
  if (type_id == QLatin1String("SelectionDock") ||
      type_id == QLatin1String("Tools") ||
      type_id == QLatin1String("ToolPropertiesDock") ||
      type_id == QLatin1String("TfTreeDock") ||
      type_id == QLatin1String("TeleopDock")) {
    return PanelMenuGroup::kToolsFrames;
  }
  return PanelMenuGroup::kOther;
}

int PanelMenuPreferredOrder(const QString& type_id) {
  static const char* kCore[] = {"ViewportDock", "DisplaysDock", "ImageDock",
                                "ViewsDock"};
  static const char* kData[] = {"RecordDock", "ChannelBrowserDock", "PlotDock",
                                "TableDock", "ChannelsDock", "ChannelGraphDock",
                                "TimeDock"};
  static const char* kTools[] = {"SelectionDock", "Tools", "ToolPropertiesDock",
                                 "TfTreeDock", "TeleopDock"};
  const auto find = [&](const char* const* list, int n) {
    for (int i = 0; i < n; ++i) {
      if (type_id == QLatin1String(list[i])) {
        return i;
      }
    }
    return 100;
  };
  switch (PanelMenuGroupForType(type_id)) {
    case PanelMenuGroup::kCoreViz:
      return find(kCore, 4);
    case PanelMenuGroup::kDataObserve:
      return find(kData, 7);
    case PanelMenuGroup::kToolsFrames:
      return find(kTools, 5);
    case PanelMenuGroup::kOther:
    default:
      return 100;
  }
}

}  // namespace

namespace {

QString PanelsMenuDescription(const QString& type_id, const QString& title) {
  for (const PanelCatalogEntry& entry : PanelCatalog()) {
    if (entry.object_name != nullptr &&
        type_id == QLatin1String(entry.object_name) &&
        entry.description != nullptr && entry.description[0] != '\0') {
      return QCoreApplication::translate("autoviz::PanelCatalog",
                                         entry.description);
    }
  }
  if (type_id == QLatin1String("Tools")) {
    return QCoreApplication::translate("autoviz::VisualizationFrame",
                                       "Show or hide the main toolbar");
  }
  if (type_id == QLatin1String("DisplaysDock")) {
    return QCoreApplication::translate("autoviz::VisualizationFrame",
                                       "Manage visualization displays");
  }
  if (type_id == QLatin1String("ViewsDock")) {
    return QCoreApplication::translate("autoviz::VisualizationFrame",
                                       "Camera views and view controllers");
  }
  if (type_id == QLatin1String("SelectionDock")) {
    return QCoreApplication::translate("autoviz::VisualizationFrame",
                                       "Inspect selected objects");
  }
  if (type_id == QLatin1String("ToolPropertiesDock")) {
    return QCoreApplication::translate("autoviz::VisualizationFrame",
                                       "Properties for the active tool");
  }
  if (type_id == QLatin1String("TimeDock")) {
    return QCoreApplication::translate("autoviz::VisualizationFrame",
                                       "Simulation and wall time");
  }
  return QCoreApplication::translate("autoviz::VisualizationFrame",
                                     "Show or hide the %1 panel")
      .arg(title);
}

}  // namespace

void FrameChrome::setupToolShortcuts() {
  qDeleteAll(tool_shortcuts_);
  tool_shortcuts_.clear();
  if (frame_->manager_ == nullptr) {
    return;
  }

  const auto shouldIgnoreShortcut = []() {
    QWidget* focus = QApplication::focusWidget();
    return focus != nullptr &&
           (focus->inherits("QLineEdit") || focus->inherits("QPlainTextEdit") ||
            focus->inherits("QTextEdit") || focus->inherits("QSpinBox") ||
            focus->inherits("QDoubleSpinBox"));
  };

  const common::ToolManager& tools = frame_->manager_->tools();
  for (const std::string& tool_id : tools.toolIds()) {
    const char letter = tools.shortcutKeyForTool(tool_id);
    if (letter == '\0') {
      continue;
    }
    auto* shortcut =
        new QShortcut(QKeySequence(QString(QChar::fromLatin1(letter))), frame_);
    shortcut->setContext(Qt::WindowShortcut);
    shortcut->setAutoRepeat(false);
    QObject::connect(shortcut, &QShortcut::activated, frame_, [this, tool_id, shouldIgnoreShortcut]() {
      if (shouldIgnoreShortcut()) {
        return;
      }
      if (frame_->manager_->tools().activeToolId() == tool_id) {
        applyActiveTool(frame_->manager_->tools().defaultToolId());
      } else {
        applyActiveTool(tool_id);
      }
    });
    tool_shortcuts_.push_back(shortcut);
  }

  auto* escape_shortcut = new QShortcut(
      ShortcutForId(LoadAppUiPreferences(), QStringLiteral("tools.reset")), frame_);
  escape_shortcut->setContext(Qt::WindowShortcut);
  escape_shortcut->setAutoRepeat(false);
  QObject::connect(escape_shortcut, &QShortcut::activated, frame_, [this, shouldIgnoreShortcut]() {
    if (shouldIgnoreShortcut()) {
      return;
    }
    if (frame_->manager_->tools().activeToolId() != frame_->manager_->tools().defaultToolId()) {
      applyActiveTool(frame_->manager_->tools().defaultToolId());
    }
  });
  tool_shortcuts_.push_back(escape_shortcut);
}

void FrameChrome::setupMenu() {
  auto* file_menu = frame_->menuBar()->addMenu(frame_->tr("&File"));
  PrepareAppMenu(file_menu);
  open_config_action_ = file_menu->addAction(
      IconLoader::menuIcon(QStringLiteral("file.open")), frame_->tr("&Open Config..."), frame_, &VisualizationFrame::onOpenConfig);
  open_config_action_->setShortcut(QKeySequence::Open);
  open_record_action_ = file_menu->addAction(
      IconLoader::menuIcon(QStringLiteral("file.open_record")),

      frame_->tr("Open &Record..."), frame_, &VisualizationFrame::onOpenRecord);
  open_record_action_->setShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_O));
  frame_->addAction(open_record_action_);
  save_config_action_ = file_menu->addAction(
      IconLoader::menuIcon(QStringLiteral("file.save")), frame_->tr("&Save Config"), frame_, &VisualizationFrame::onSaveConfig);
  save_config_action_->setShortcut(QKeySequence::Save);
  save_config_as_action_ = file_menu->addAction(
      IconLoader::menuIcon(QStringLiteral("file.save_as")),

      frame_->tr("Save Config &As..."), frame_, &VisualizationFrame::onSaveConfigAs);
  save_config_as_action_->setShortcut(QKeySequence::SaveAs);
  recent_configs_menu_ = file_menu->addMenu(
      IconLoader::menuIcon(QStringLiteral("file.recent")), frame_->tr("&Recent Configs"));
  PrepareAppMenu(recent_configs_menu_);
  file_menu->addAction(IconLoader::menuIcon(QStringLiteral("file.image")),
                       frame_->tr("Save &Image..."), frame_, &VisualizationFrame::onScreenshot);
  file_menu->addSeparator();
  file_menu->addAction(IconLoader::menuIcon(QStringLiteral("app.settings")),
                       frame_->tr("&Settings..."), frame_, &VisualizationFrame::onAppSettings);
  file_menu->addAction(IconLoader::menuIcon(QStringLiteral("file.reset_layout")),
                       frame_->tr("&Reset to default layout"), frame_, &VisualizationFrame::onResetDefaultLayout);
  file_menu->addSeparator();
  quit_action_ = file_menu->addAction(
      IconLoader::menuIcon(QStringLiteral("file.quit")), frame_->tr("&Quit"), qApp,
      &QApplication::quit);
  quit_action_->setShortcut(QKeySequence::Quit);
  frame_->addAction(quit_action_);

  panels_menu_ = frame_->menuBar()->addMenu(frame_->tr("&Panels"));
  PreparePanelsMenu(panels_menu_);
  QObject::connect(panels_menu_, &QMenu::aboutToShow, frame_, [this]() {
    // Refresh toggles so closed Split duplicates disappear and checkboxes
    // match the live center/sidebar docks.
    rebuildPanelsMenuToggles();
    for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
      if (dock == nullptr) {
        continue;
      }
      if (QAction* toggle = dock->toggleViewAction()) {
        toggle->blockSignals(true);
        toggle->setChecked(dock->isVisible());
        toggle->blockSignals(false);
      }
    }
  });

  add_panel_action_ = panels_menu_->addAction(
      IconLoader::menuIcon(QStringLiteral("panels.add")),

      frame_->tr("&Add Panel"), frame_, &VisualizationFrame::onAddPanel);
  add_panel_action_->setToolTip(frame_->tr("Open the catalog to add a panel"));
  frame_->addAction(add_panel_action_);

  delete_panel_menu_ = panels_menu_->addMenu(
      IconLoader::menuIcon(QStringLiteral("panels.delete")),
      frame_->tr("&Delete Panel"));
  PreparePanelsMenu(delete_panel_menu_);
  delete_panel_menu_->setEnabled(false);
  delete_panel_menu_->setToolTip(frame_->tr("Close a visible panel"));

  fullscreen_action_ = panels_menu_->addAction(
      IconLoader::menuIcon(QStringLiteral("panels.fullscreen")),

      frame_->tr("&Fullscreen"), frame_, &VisualizationFrame::onToggleFullscreen);
  fullscreen_action_->setCheckable(true);
  fullscreen_action_->setToolTip(frame_->tr("Enter or exit fullscreen view"));
  frame_->addAction(fullscreen_action_);

  QObject::connect(frame_, &VisualizationFrame::fullScreenChange, fullscreen_action_,
          &QAction::setChecked);
  panels_menu_->addSeparator();

  auto* backend_group = new QActionGroup(frame_);
  backend_group->setExclusive(true);
  backend_ogre_action_ = new QAction(frame_->tr("&Ogre"), frame_);
  backend_ogre_action_->setCheckable(true);
  backend_ogre_action_->setChecked(true);
  backend_ogre_action_->setToolTip(
      frame_->tr("Autoviz 3D viewport uses Ogre 1.x only."));
  backend_group->addAction(backend_ogre_action_);
  QObject::connect(backend_ogre_action_, &QAction::triggered, frame_,
                   &VisualizationFrame::onBackendOgre);

  frame_->session_->syncRenderBackendMenu(QStringLiteral("Ogre"));

  auto* help_menu = frame_->menuBar()->addMenu(frame_->tr("&Help"));
  PrepareAppMenu(help_menu);
  help_menu->addAction(IconLoader::menuIcon(QStringLiteral("help.about")),

                       frame_->tr("&About"), frame_, &VisualizationFrame::onHelpAbout);

  QSettings settings;
  frame_->session_->recent_configs_ =
      settings.value(QStringLiteral("recent_configs")).toStringList();
  frame_->session_->updateRecentConfigMenu();
  configureMenuBar();
}

void FrameChrome::configureMenuBar() {
  frame_->menuBar()->setObjectName(QString::fromLatin1(AppThemeIds::kMenuBar));
  frame_->menuBar()->setNativeMenuBar(false);
  frame_->menuBar()->setDefaultUp(false);
  frame_->menuBar()->setAttribute(Qt::WA_StyledBackground, false);
  frame_->menuBar()->setAutoFillBackground(false);
}

void FrameChrome::setupToolbar() {
  tool_bar_ = frame_->addToolBar(frame_->tr("Tools"));
  tool_bar_->setObjectName(QString::fromLatin1(AppThemeIds::kToolBar));
  tool_bar_->setMovable(false);
  tool_bar_->setFloatable(false);
  tool_bar_->setAttribute(Qt::WA_StyledBackground, false);
  tool_bar_->setAutoFillBackground(false);
  tool_bar_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  tool_bar_->setIconSize(QSize(18, 18));
  tool_action_group_ = new QActionGroup(frame_);
  tool_action_group_->setExclusive(true);

  add_tool_action_ = new QAction(
      IconLoader::toolbarGlyph(QStringLiteral(":/autoviz/icons/tool/plus")),
      QString(), frame_);
  add_tool_action_->setToolTip(frame_->tr("Add a tool to the toolbar"));
  QObject::connect(add_tool_action_, &QAction::triggered, frame_, &VisualizationFrame::onAddToolTriggered);

  remove_tool_menu_ = new QMenu(frame_);
  PrepareAppMenu(remove_tool_menu_);
  QObject::connect(remove_tool_menu_, &QMenu::triggered, frame_, &VisualizationFrame::onRemoveToolTriggered);
  auto* remove_tool_button = new QToolButton(frame_);
  remove_tool_button->setObjectName(QStringLiteral("AutovizToolbarChromeButton"));
  remove_tool_button->setMenu(remove_tool_menu_);
  remove_tool_button->setPopupMode(QToolButton::InstantPopup);
  remove_tool_button->setToolTip(frame_->tr("Remove a tool from the toolbar"));
  remove_tool_button->setIcon(
      IconLoader::toolbarGlyph(QStringLiteral(":/autoviz/icons/tool/minus")));
  remove_tool_button->setAutoRaise(true);
  remove_tool_button->setCursor(Qt::PointingHandCursor);
  remove_tool_button->setToolButtonStyle(Qt::ToolButtonIconOnly);
  remove_tool_button->setFixedSize(32, 32);

  rebuildToolbar();
  tool_bar_->addSeparator();
  tool_bar_->addAction(add_tool_action_);
  tool_bar_->addWidget(remove_tool_button);

  if (panels_menu_ != nullptr) {
    rebuildPanelsMenuToggles();
  } else {
    registerPanelToggleActions();
  }
  setupToolbarLayoutControls();
}

void FrameChrome::setupToolbarLayoutControls() {
  if (tool_bar_ == nullptr) {
    return;
  }

  toolbar_spacer_ = new QWidget(tool_bar_);
  toolbar_spacer_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  tool_bar_->addWidget(toolbar_spacer_);

  auto configureSidebarToggle = [this](QAction* action) {
    if (action == nullptr || tool_bar_ == nullptr) {
      return;
    }
    if (auto* button =
            qobject_cast<QToolButton*>(tool_bar_->widgetForAction(action))) {
      button->setObjectName(QStringLiteral("AutovizToolbarSidebarToggle"));
      button->setAutoRaise(true);
      button->setToolButtonStyle(Qt::ToolButtonIconOnly);
      button->setFixedSize(28, 28);
      button->setCursor(Qt::PointingHandCursor);
    }
  };

  toolbar_toggle_left_dock_action_ = tool_bar_->addAction(
      IconLoader::toolbarGlyph(
          QStringLiteral(":/autoviz/icons/dock/sidebar_left")),
      QString());
  toolbar_toggle_left_dock_action_->setCheckable(true);
  toolbar_toggle_left_dock_action_->setChecked(true);
  toolbar_toggle_left_dock_action_->setToolTip(frame_->tr("Hide Left"));
  configureSidebarToggle(toolbar_toggle_left_dock_action_);
  QObject::connect(toolbar_toggle_left_dock_action_, &QAction::toggled, frame_, [this](bool show_left) {
            frame_->layout_->hideLeftDock(!show_left);
            syncToolbarLayoutControls();
            frame_->session_->markConfigModified();
          });

  toolbar_toggle_right_dock_action_ = tool_bar_->addAction(
      IconLoader::toolbarGlyph(
          QStringLiteral(":/autoviz/icons/dock/sidebar_right")),
      QString());
  toolbar_toggle_right_dock_action_->setCheckable(true);
  toolbar_toggle_right_dock_action_->setChecked(true);
  toolbar_toggle_right_dock_action_->setToolTip(frame_->tr("Hide Right"));
  configureSidebarToggle(toolbar_toggle_right_dock_action_);
  QObject::connect(toolbar_toggle_right_dock_action_, &QAction::toggled, frame_, [this](bool show_right) {
            frame_->layout_->hideRightDock(!show_right);
            syncToolbarLayoutControls();
            frame_->session_->markConfigModified();
          });

  syncToolbarLayoutControls();
}

void FrameChrome::syncToolbarLayoutControls() {
  if (toolbar_toggle_left_dock_action_ != nullptr) {
    toolbar_toggle_left_dock_action_->blockSignals(true);
    const bool show_left = frame_->manager_ != nullptr &&
                           !frame_->manager_->hideLeftDock() &&
                           frame_->layout_->sidebarAreaHasVisibleDock(
                               Qt::LeftDockWidgetArea);
    toolbar_toggle_left_dock_action_->setChecked(show_left);
    toolbar_toggle_left_dock_action_->setToolTip(
        show_left ? frame_->tr("Hide Left") : frame_->tr("Show Left"));
    toolbar_toggle_left_dock_action_->blockSignals(false);
  }
  if (toolbar_toggle_right_dock_action_ != nullptr) {
    toolbar_toggle_right_dock_action_->blockSignals(true);
    const bool show_right = frame_->manager_ != nullptr &&
                            !frame_->manager_->hideRightDock() &&
                            frame_->layout_->sidebarAreaHasVisibleDock(
                                Qt::RightDockWidgetArea);
    toolbar_toggle_right_dock_action_->setChecked(show_right);
    toolbar_toggle_right_dock_action_->setToolTip(
        show_right ? frame_->tr("Hide Right") : frame_->tr("Show Right"));
    toolbar_toggle_right_dock_action_->blockSignals(false);
  }
}

void FrameChrome::rebuildToolbar() {
  if (tool_bar_ == nullptr || tool_action_group_ == nullptr) {
    return;
  }

  if (frame_->manager_->toolbarTools().empty()) {
    frame_->manager_->setToolbarTools({});
  }

  for (QAction* action : toolbar_tool_actions_) {
    tool_action_group_->removeAction(action);
    tool_bar_->removeAction(action);
    delete action;
  }
  toolbar_tool_actions_.clear();
  if (remove_tool_menu_ != nullptr) {
    remove_tool_menu_->clear();
  }

  int shortcut_index = 1;
  const common::ToolManager& tools = frame_->manager_->tools();
  for (const std::string& tool_id : frame_->manager_->toolbarTools()) {
    const QString tool_q = QString::fromStdString(tool_id);
    const QString label = tools.toolLabel(tool_id);
    auto* action = new QAction(IconLoader::toolIcon(tool_q), label, frame_);
    action->setObjectName(tool_q);
    action->setCheckable(true);
    action->setData(tool_q);
    if (shortcut_index <= 9) {
      action->setShortcut(QKeySequence(QString::number(shortcut_index)));
      ++shortcut_index;
    }
    const char letter_shortcut = tools.shortcutKeyForTool(tool_id);
    if (letter_shortcut != '\0') {
      action->setToolTip(
          frame_->tr("%1 (快捷键 %2)").arg(label).arg(QChar::fromLatin1(letter_shortcut)));
    } else {
      action->setToolTip(label);
    }
    if (tool_id == tools.activeToolId()) {
      action->setChecked(true);
    }
    QObject::connect(action, &QAction::triggered, frame_, [this, tool_id]() {
      applyActiveTool(tool_id);
    });
    tool_action_group_->addAction(action);
    if (add_tool_action_ != nullptr &&
        tool_bar_->actions().contains(add_tool_action_)) {
      tool_bar_->insertAction(add_tool_action_, action);
    } else {
      tool_bar_->addAction(action);
    }
    toolbar_tool_actions_.push_back(action);
    if (remove_tool_menu_ != nullptr) {
      QAction* remove_action = remove_tool_menu_->addAction(label);
      remove_action->setData(tool_q);
    }
  }

  if (add_tool_action_ != nullptr) {
    add_tool_action_->setEnabled(!tools.toolsNotInToolbar().empty());
  }
}

void FrameChrome::applyActiveTool(const std::string& tool_id) {
  if (ViewportPanelEntry* entry = frame_->viewport_->activeViewportEntry()) {
    // Prefer dock-scoped path so Measure session clears for frame_ Split only.
    frame_->viewport_->setViewportLocalTool(entry->dock, tool_id);
    syncActiveToolUi();
    return;
  }
  if (!frame_->manager_->tools().setActiveTool(tool_id)) {
    return;
  }
  syncActiveToolUi();
}

void FrameChrome::syncActiveToolUi() {
  syncToolbarToActiveTool();
  frame_->viewport_->syncViewportTitleBarTools();
  frame_->viewport_->syncToolContext();
  if (frame_->panels_->tool_properties_panel_ != nullptr) {
    frame_->panels_->tool_properties_panel_->refresh();
  }
  status_hint_.clear();
  updateStatusBar();
  frame_->viewport_->updateViewportCursor();
  if (ViewportPanelEntry* entry = frame_->viewport_->activeViewportEntry()) {
    if (entry->widget != nullptr) {
      entry->widget->setFocus(Qt::MouseFocusReason);
    }
  }
  frame_->viewport_->requestViewportUpdate();
}

void FrameChrome::syncToolbarToActiveTool() {
  const QString active =
      QString::fromStdString(frame_->manager_->tools().activeToolId());
  for (QAction* action : toolbar_tool_actions_) {
    if (action == nullptr) {
      continue;
    }
    action->setChecked(action->data().toString() == active);
  }
}

void FrameChrome::onAddToolTriggered() {
  QMenu menu(frame_);
  for (const std::string& id : frame_->manager_->tools().toolsNotInToolbar()) {
    const QString tool_q = QString::fromStdString(id);
    QAction* action = menu.addAction(frame_->manager_->tools().toolLabel(id));
    action->setData(tool_q);
    action->setIcon(IconLoader::toolIcon(tool_q));
  }
  if (menu.isEmpty()) {
    return;
  }
  QAction* picked = menu.exec(QCursor::pos());
  if (picked == nullptr) {
    return;
  }
  if (frame_->manager_->tools().addToolToToolbar(
          picked->data().toString().toStdString())) {
    rebuildToolbar();
    frame_->session_->markConfigModified();
  }
}

void FrameChrome::onRemoveToolTriggered(QAction* action) {
  if (action == nullptr) {
    return;
  }
  if (frame_->manager_->tools().removeToolFromToolbar(
          action->data().toString().toStdString())) {
    rebuildToolbar();
    if (frame_->panels_->tool_properties_panel_ != nullptr) {
      frame_->panels_->tool_properties_panel_->refresh();
    }
    updateStatusBar();
    frame_->session_->markConfigModified();
  }
}

void FrameChrome::registerPanelMenuToggle(PanelDockWidget* dock) {
  Q_UNUSED(dock);
  rebuildPanelsMenuToggles();
}

void FrameChrome::registerPanelToggleActions() {
  rebuildPanelsMenuToggles();
}

QString detail::PanelsMenuDisplayTitle(const QString& type_id, const QString& fallback) {
  // Short Title Case nouns — keep Panels menu labels consistent in length/style.
  if (type_id == QLatin1String("ViewportDock")) {
    return QCoreApplication::translate("autoviz::VisualizationFrame", "3D View");
  }
  if (type_id == QLatin1String("DisplaysDock")) {
    return QCoreApplication::translate("autoviz::VisualizationFrame", "Displays");
  }
  if (type_id == QLatin1String("ImageDock")) {
    return QCoreApplication::translate("autoviz::VisualizationFrame", "Image");
  }
  if (type_id == QLatin1String("ViewsDock")) {
    return QCoreApplication::translate("autoviz::VisualizationFrame", "Views");
  }
  if (type_id == QLatin1String("ChannelBrowserDock")) {
    return QCoreApplication::translate("autoviz::VisualizationFrame", "Channels");
  }
  if (type_id == QLatin1String("PlotDock")) {
    return QCoreApplication::translate("autoviz::VisualizationFrame", "Plot");
  }
  if (type_id == QLatin1String("TableDock")) {
    return QCoreApplication::translate("autoviz::VisualizationFrame", "Table");
  }
  if (type_id == QLatin1String("ChannelsDock")) {
    return QCoreApplication::translate("autoviz::VisualizationFrame", "Messages");
  }
  if (type_id == QLatin1String("RecordDock")) {
    return QCoreApplication::translate("autoviz::VisualizationFrame", "Record");
  }
  if (type_id == QLatin1String("TimeDock")) {
    return QCoreApplication::translate("autoviz::VisualizationFrame", "Time");
  }
  if (type_id == QLatin1String("SelectionDock")) {
    return QCoreApplication::translate("autoviz::VisualizationFrame", "Selection");
  }
  if (type_id == QLatin1String("Tools")) {
    return QCoreApplication::translate("autoviz::VisualizationFrame", "Tools");
  }
  if (type_id == QLatin1String("ToolPropertiesDock")) {
    return QCoreApplication::translate("autoviz::VisualizationFrame",
                                       "Properties");
  }
  if (type_id == QLatin1String("TfTreeDock")) {
    return QCoreApplication::translate("autoviz::VisualizationFrame",
                                       "Transforms");
  }
  if (type_id == QLatin1String("MapDock")) {
    return QCoreApplication::translate("autoviz::VisualizationFrame", "Map");
  }
  if (type_id == QLatin1String("PublishDock")) {
    return QCoreApplication::translate("autoviz::VisualizationFrame", "Publish");
  }
  if (type_id == QLatin1String("ServiceDock")) {
    return QCoreApplication::translate("autoviz::VisualizationFrame", "Service");
  }
  if (type_id == QLatin1String("TeleopDock")) {
    return QCoreApplication::translate("autoviz::VisualizationFrame", "Teleop");
  }
  if (type_id == QLatin1String("ChannelGraphDock")) {
    return QCoreApplication::translate("autoviz::VisualizationFrame",
                                       "Channel Graph");
  }
  return fallback;
}

void FrameChrome::rebuildPanelsMenuToggles() {
  if (panels_menu_ == nullptr) {
    return;
  }

  // QPointer: Split-duplicate docks may already be destroyed; never touch
  // dangling toggleViewAction pointers left in the tracked list.
  for (const QPointer<QAction>& action : panels_menu_toggle_actions_) {
    if (action.isNull()) {
      continue;
    }
    action->setShortcut(QKeySequence());
    panels_menu_->removeAction(action.data());
  }
  panels_menu_toggle_actions_.clear();

  struct ToggleEntry {
    QString title;
    QString type_id;
    QAction* action = nullptr;
    PanelMenuGroup group = PanelMenuGroup::kOther;
    int preferred = 100;
  };
  QVector<ToggleEntry> entries;

  if (tool_bar_ != nullptr) {
    QAction* tools_toggle = tool_bar_->toggleViewAction();
    tools_toggle->setCheckable(true);
    const QString title = frame_->tr("Tools");
    tools_toggle->setText(title);
    tools_toggle->setIcon(
        IconLoader::menuIcon(QStringLiteral("panels.tools")));
    entries.push_back({title, QStringLiteral("Tools"), tools_toggle,
                       PanelMenuGroupForType(QStringLiteral("Tools")),
                       PanelMenuPreferredOrder(QStringLiteral("Tools"))});
  }

  for (PanelDockWidget* dock : frame_->layout_->orderedDockWidgets()) {
    if (dock == nullptr || dock->property("panelDisposed").toBool()) {
      continue;
    }
    IconLoader::applyDockPanelChrome(dock, frame_->panels_->panelTypeId(dock));
    QAction* toggle = dock->toggleViewAction();
    toggle->setCheckable(true);
    toggle->blockSignals(true);
    toggle->setChecked(dock->isVisible());
    toggle->blockSignals(false);
    const QString type_id = frame_->panels_->panelTypeId(dock);
    const QString fallback = dock->windowTitle().trimmed().isEmpty()
                                 ? dock->objectName()
                                 : dock->windowTitle().trimmed();
    const QString title = detail::PanelsMenuDisplayTitle(type_id, fallback);
    const QIcon icon = IconLoader::panelsMenuDockIcon(type_id);
    if (!icon.isNull()) {
      toggle->setIcon(icon);
    }
    toggle->setText(title);
    entries.push_back({title, type_id, toggle, PanelMenuGroupForType(type_id),
                       PanelMenuPreferredOrder(type_id)});
  }

  std::sort(entries.begin(), entries.end(),
            [](const ToggleEntry& a, const ToggleEntry& b) {
              if (a.group != b.group) {
                return static_cast<int>(a.group) < static_cast<int>(b.group);
              }
              if (a.preferred != b.preferred) {
                return a.preferred < b.preferred;
              }
              return QString::localeAwareCompare(a.title, b.title) < 0;
            });

  QSet<QChar> used_mnemonics;
  used_mnemonics.insert(QLatin1Char('N'));
  used_mnemonics.insert(QLatin1Char('D'));
  used_mnemonics.insert(QLatin1Char('F'));
  used_mnemonics.insert(QLatin1Char('P'));

  PanelMenuGroup previous_group = PanelMenuGroup::kOther;
  bool first_item = true;
  for (ToggleEntry& entry : entries) {
    if (entry.action == nullptr) {
      continue;
    }
    if (!first_item && entry.group != previous_group) {
      QAction* separator = panels_menu_->addSeparator();
      panels_menu_toggle_actions_.push_back(separator);
    }
    first_item = false;
    previous_group = entry.group;

    entry.action->setShortcut(QKeySequence());
    entry.action->setText(
        PanelMenuTitleWithMnemonic(entry.title, &used_mnemonics));
    entry.action->setToolTip(PanelsMenuDescription(entry.type_id, entry.title));
    if (!entry.action->property("panelsMenuToggleWired").toBool()) {
      entry.action->setProperty("panelsMenuToggleWired", true);
      QObject::connect(entry.action, &QAction::triggered, frame_, &VisualizationFrame::markConfigModified, Qt::UniqueConnection);
    }
    panels_menu_->addAction(entry.action);
    panels_menu_toggle_actions_.push_back(entry.action);
  }
}

void FrameChrome::onToolTriggered(QAction* action) {
  if (action == nullptr) {
    return;
  }
  applyActiveTool(action->data().toString().toStdString());
  frame_->session_->markConfigModified();
}

void FrameChrome::updateChannelList() {
  if (frame_->panels_->raw_messages_panel_ != nullptr) {
    frame_->panels_->raw_messages_panel_->refreshChannels();
  }
  if (frame_->panels_->channels_panel_ != nullptr) {
    frame_->panels_->channels_panel_->refreshChannels();
  }
  frame_->panels_->refreshAllPlotSettingsChannels();
}

void FrameChrome::setupStatusBar() {
  // Match RViz2 VisualizationFrame: [status message …] [N fps]
  status_label_ = new QLabel(QString(), frame_);
  status_label_->setTextFormat(Qt::RichText);
  status_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  StylePanelStatusLabel(status_label_);
  frame_->statusBar()->addPermanentWidget(status_label_, 1);

  fps_label_ = new QLabel(QString(), frame_);
  fps_label_->setMinimumWidth(40);
  fps_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  StylePanelStatusLabel(fps_label_);
  frame_->statusBar()->addPermanentWidget(fps_label_, 0);

  last_fps_calc_ = std::chrono::steady_clock::now();
  updateStatusBar();
}

void FrameChrome::updateFps() {
  ++frame_count_;
  const auto now = std::chrono::steady_clock::now();
  if (now - last_fps_calc_ <= std::chrono::seconds(1)) {
    return;
  }
  const double seconds =
      std::chrono::duration<double>(now - last_fps_calc_).count();
  const int fps = seconds > 0.0 ? static_cast<int>(frame_count_ / seconds) : 0;
  frame_count_ = 0;
  last_fps_calc_ = now;
  const QString fps_text = QStringLiteral("%1 fps").arg(fps);
  if (fps_label_ != nullptr) {
    fps_label_->setText(fps_text);
  }
  if (frame_->panels_->time_panel_ != nullptr) {
    frame_->panels_->time_panel_->setFpsText(fps_text);
  }
}

void FrameChrome::updateStatusBar() {
  if (status_label_ == nullptr) {
    return;
  }
  // RViz2: status bar message is only the active tool status — not a chip strip.
  QString message = status_hint_;
  if (message.isEmpty() && frame_->manager_ != nullptr) {
    message = frame_->manager_->tools().activeStatusText();
  }
  status_label_->setText(message);
}

PanelContextMenuCallbacks FrameChrome::makePanelContextMenuCallbacks(
    PanelDockWidget* dock) {
  PanelContextMenuCallbacks callbacks;
  if (dock == nullptr) {
    return callbacks;
  }
  callbacks.current_object_name = frame_->panels_->panelTypeId(dock);
  callbacks.change_panel = [this, dock](const QString& object_name) {
    frame_->layout_->changePanelInDock(dock, object_name);
  };
  callbacks.split = [this, dock](Qt::Orientation orientation) {
    frame_->layout_->onSplitActiveDock(dock, orientation);
  };
  callbacks.expand = [this, dock]() { frame_->layout_->expandPanelDock(dock); };
  callbacks.remove = [dock]() { dock->close(); };
  return callbacks;
}

void FrameChrome::installStandardPanelTitleTools(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  const PanelContextMenuCallbacks callbacks = makePanelContextMenuCallbacks(dock);
  PanelTitleBarOptions options;
  options.show_expand = true;
  options.expand_checkable = false;
  options.on_expand = callbacks.expand;
  dock->setTitleBarTools(
      CreatePanelTitleBarTools(dock, callbacks, options).widget);
}

}  // namespace autoviz
