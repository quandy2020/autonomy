/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file frame_panels.hpp
 * @brief VisualizationFrame collaborator — create, wire, and track feature panels.
 *
 * Factory and wiring for Displays, Time, Views, Image, Plot, Teleop, Publish,
 * Service, Map, Channel Graph, TF Tree, and related docks. Tracks the “active”
 * multi-instance panel for Property Inspector binding and session capture.
 *
 * @see VisualizationFrame
 * @see FrameLayout
 * @see PanelCatalog()
 * @see PropertyInspectorPanel
 */

#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <QHash>
#include <QList>
#include <QObject>
#include <QPointer>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QElapsedTimer>

#include <QColor>
#include <QImage>
#include <QVector3D>

#include "autoviz/common/selection.hpp"
#include "autoviz/ui/panel/context_menu.hpp"
#include "autoviz/ui/panel/dock.hpp"
#include "autoviz/ui/viewport_panel.hpp"

class QLabel;
class QMenu;
class QToolBar;
class QToolButton;
class QAction;
class QActionGroup;
class QShortcut;
class QMimeData;
class QDragEnterEvent;
class QDragMoveEvent;
class QDragLeaveEvent;
class QDropEvent;
class QKeyEvent;
class QResizeEvent;
class QEvent;

namespace autoviz {

class VisualizationFrame;
struct AppSettingsResult;
struct AppUiPreferences;
class MainPanelHost;
class DisplaysPanel;
class TimePanel;
class ViewsPanel;
class ToolPropertiesPanel;
class SelectionPanel;
class RawMessagesPanel;
class ChannelsPanel;
class RecordPanel;
class PropertyInspectorPanel;
class TfTreePanel;
namespace plot { class PlotPanel; }
namespace image { class ImagePanel; }
namespace teleop { class TeleopPanel; }
namespace publish_panel { class PublishPanel; }
namespace map { class MapPanel; }
namespace service_panel { class ServicePanel; }
namespace channel_graph { class ChannelGraphPanel; }
namespace table_panel { class TablePanel; }
namespace rendering { class ViewController; class OgreRenderWindow; }

/**
 * @class FramePanels
 * @brief Collaborator for @ref VisualizationFrame — panels responsibilities.
 *
 * ## Responsibilities
 *
 * - Create dock + panel pairs for catalog types
 * - Wire signals (focus → Property Inspector, title updates, delete menu)
 * - Capture / restore per-panel session configs (Image / Plot / Publish)
 * - Track active multi-instance panels for inspector binding
 *
 * ## Singleton vs multi-instance
 *
 * Displays, Views, Time, etc. are single docks held as members. Image, Plot,
 * Publish, Teleop, Service, Map, Channel Graph, TF Tree support multiple
 * instances via @ref uniquePanelObjectName() when
 * @ref panelTypeSupportsMultiInstance() is true.
 *
 * @note Not a @c QObject — owned by @ref VisualizationFrame.
 *
 * @see FrameLayout::changePanelInDock()
 * @see PanelCatalog()
 */
class FramePanels {
  friend class VisualizationFrame;
  friend class FrameLayout;
  friend class FrameViewport;
  friend class FrameChrome;
  friend class FrameSession;

 public:
  /**
   * @brief Constructs panel helpers bound to @p frame.
   * @param frame Non-owning back-pointer (must outlive this object).
   */
  explicit FramePanels(VisualizationFrame* frame);

  /** @brief Default destructor; docks are parented to the frame / host. */
  ~FramePanels() = default;

  FramePanels(const FramePanels&) = delete;
  FramePanels& operator=(const FramePanels&) = delete;

  /**
   * @brief Marks @p dock as the last-active panel (split / delete / focus).
   * @param dock Activated panel dock.
   */
  void activatePanelDock(PanelDockWidget* dock);

  /**
   * @brief Applies Plot settings-pane visibility from the current session.
   */
  void applyPlotSettingsVisibilityFromSession();

  /**
   * @brief Binds @p panel into the Property Inspector (Image page).
   * @param panel Image panel to inspect.
   */
  void bindImageToPropertyInspector(image::ImagePanel* panel);

  /**
   * @brief Binds @p panel into the Property Inspector (Map page).
   * @param panel Map panel to inspect.
   */
  void bindMapToPropertyInspector(map::MapPanel* panel);

  /**
   * @brief Binds @p panel into the Property Inspector (Plot page).
   * @param panel Plot panel to inspect.
   */
  void bindPlotToPropertyInspector(plot::PlotPanel* panel);

  /**
   * @brief Binds @p panel into the Property Inspector (Publish page).
   * @param panel Publish panel to inspect.
   */
  void bindPublishToPropertyInspector(publish_panel::PublishPanel* panel);

  /**
   * @brief Binds @p panel into the Property Inspector (Service page).
   * @param panel Service panel to inspect.
   */
  void bindServiceToPropertyInspector(service_panel::ServicePanel* panel);

  /**
   * @brief Binds @p panel into the Property Inspector (Teleop page).
   * @param panel Teleop panel to inspect.
   */
  void bindTeleopToPropertyInspector(teleop::TeleopPanel* panel);

  /**
   * @brief Snapshots all Image panel configs into session storage.
   */
  void captureImagePanelConfigs();

  /**
   * @brief Opens / closes a docked Image viewer for each enabled Image
   *        display (RViz2 associated-widget behaviour).
   *
   * Viewers are flexible @ref PanelDockWidget instances on the visualization
   * frame (@ref FrameLayout::configureFlexibleDock) so they can float and
   * snap to any dock area. Closing unchecks the Displays row; re-checking
   * recreates / shows the dock.
   *
   * Call after Displays add / remove / enable / rename / session restore.
   */
  void syncImageDisplayWindows();

  /**
   * @brief Pushes a decoded frame into the Image-display dock named
   *        @p source, if one is open.
   *
   * @param source Display name (from @c image_updated).
   * @param image Decoded frame.
   */
  void updateImageDisplayWindowFrame(const QString& source,
                                     const QImage& image);

  /**
   * @brief Snapshots all Plot panel configs into session storage.
   */
  void capturePlotPanelConfigs();

  /**
   * @brief Snapshots all Table panel configs into session storage.
   */
  void captureTablePanelConfigs();

  /**
   * @brief Snapshots all Channel Graph panel configs into session storage.
   */
  void captureChannelGraphPanelConfigs();

  /**
   * @brief Snapshots all Transform Tree panel configs into session storage.
   */
  void captureTfTreePanelConfigs();

  /**
   * @brief Snapshots all Publish panel configs into session storage.
   */
  void capturePublishPanelConfigs();

  /**
   * @brief Snapshots all Service Call panel configs into session storage.
   */
  void captureServicePanelConfigs();

  /**
   * @brief Snapshots all Teleop panel configs into session storage.
   */
  void captureTeleopPanelConfigs();

  /**
   * @brief Snapshots all Map panel configs into session storage.
   */
  void captureMapPanelConfigs();

  /**
   * @brief Clears inspector binding if it currently shows @p panel.
   * @param panel Image panel being unbound / destroyed.
   */
  void clearPropertyInspectorForImage(image::ImagePanel* panel);

  /**
   * @brief Clears inspector binding if it currently shows @p panel.
   * @param panel Map panel being unbound / destroyed.
   */
  void clearPropertyInspectorForMap(map::MapPanel* panel);

  /**
   * @brief Clears inspector binding if it currently shows @p panel.
   * @param panel Plot panel being unbound / destroyed.
   */
  void clearPropertyInspectorForPlot(plot::PlotPanel* panel);

  /**
   * @brief Clears inspector binding if it currently shows @p panel.
   * @param panel Publish panel being unbound / destroyed.
   */
  void clearPropertyInspectorForPublish(publish_panel::PublishPanel* panel);

  /**
   * @brief Clears inspector binding if it currently shows @p panel.
   * @param panel Service panel being unbound / destroyed.
   */
  void clearPropertyInspectorForService(service_panel::ServicePanel* panel);

  /**
   * @brief Clears inspector binding if it currently shows @p panel.
   * @param panel Teleop panel being unbound / destroyed.
   */
  void clearPropertyInspectorForTeleop(teleop::TeleopPanel* panel);

  /**
   * @brief Creates a Record playback dock (+ panel) and registers it.
   * @param object_name Optional fixed @c objectName; empty → @c RecordDock.
   * @return Singleton right-sidebar Record dock.
   */
  PanelDockWidget* createRecordPanelDock(const QString& object_name = QString());

  /**
   * @brief Creates a Channel Graph dock (+ panel) and registers it.
   * @param object_name Optional fixed @c objectName; empty → unique name.
   * @return New dock, or existing one when reusing a singleton slot.
   */
  PanelDockWidget* createChannelGraphPanelDock(const QString& object_name = QString());

  /**
   * @brief Creates an Image panel dock.
   * @param object_name Optional fixed @c objectName; empty → unique name.
   * @return New @ref PanelDockWidget hosting @ref image::ImagePanel.
   */
  PanelDockWidget* createImagePanelDock(const QString& object_name = QString());

  /**
   * @brief Creates a Map panel dock.
   * @param object_name Optional fixed @c objectName; empty → unique name.
   * @return New dock hosting @ref map::MapPanel.
   */
  PanelDockWidget* createMapPanelDock(const QString& object_name = QString());

  /**
   * @brief Creates a Plot panel dock.
   * @param object_name Optional fixed @c objectName; empty → unique name.
   * @return New dock hosting @ref plot::PlotPanel.
   */
  PanelDockWidget* createPlotPanelDock(const QString& object_name = QString());

  /**
   * @brief Creates a Table panel dock.
   * @param object_name Optional fixed @c objectName; empty → unique name.
   * @return New dock hosting @ref table_panel::TablePanel.
   */
  PanelDockWidget* createTablePanelDock(const QString& object_name = QString());

  /**
   * @brief Creates a Publish panel dock.
   * @param object_name Optional fixed @c objectName; empty → unique name.
   * @return New dock hosting @ref publish_panel::PublishPanel.
   */
  PanelDockWidget* createPublishPanelDock(const QString& object_name = QString());

  /**
   * @brief Creates a Service panel dock.
   * @param object_name Optional fixed @c objectName; empty → unique name.
   * @return New dock hosting @ref service_panel::ServicePanel.
   */
  PanelDockWidget* createServicePanelDock(const QString& object_name = QString());

  /**
   * @brief Creates a Teleop panel dock.
   * @param object_name Optional fixed @c objectName; empty → unique name.
   * @return New dock hosting @ref teleop::TeleopPanel.
   */
  PanelDockWidget* createTeleopPanelDock(const QString& object_name = QString());

  /**
   * @brief Creates a TF Tree panel dock.
   * @param object_name Optional fixed @c objectName; empty → unique name.
   * @return New dock hosting @ref TfTreePanel.
   */
  PanelDockWidget* createTfTreePanelDock(const QString& object_name = QString());

  /**
   * @brief Clones @p source into a new dock of the same panel type.
   *
   * Used by Split right / Split down.
   *
   * @param source Dock to duplicate.
   * @return New dock, or @c nullptr if the type cannot be duplicated.
   */
  PanelDockWidget* duplicatePanelDock(PanelDockWidget* source);

  /**
   * @brief Ensures an Image dock with @p object_name exists (session restore).
   * @param object_name Dock @c objectName to find or create.
   */
  void ensureImageDockExists(const QString& object_name);

  /**
   * @brief Ensures a Plot dock with @p object_name exists (session restore).
   * @param object_name Dock @c objectName to find or create.
   */
  void ensurePlotDockExists(const QString& object_name);

  /**
   * @brief Ensures a Publish dock with @p object_name exists (session restore).
   * @param object_name Dock @c objectName to find or create.
   */
  void ensurePublishDockExists(const QString& object_name);

  /**
   * @brief Installs focus tracking so the active Image panel drives the
   *        Property Inspector.
   */
  void installImageFocusTracking();

  /**
   * @brief Installs focus tracking so the active Plot panel drives the
   *        Property Inspector.
   */
  void installPlotFocusTracking();

  /**
   * @brief Deletes the last-active / selected deletable panel dock.
   */
  void onDeletePanel();

  /**
   * @brief Returns the catalog type id for @p dock.
   * @param dock Panel dock.
   * @return Type id string (e.g. @c "Image"), or empty if unknown.
   */
  QString panelTypeId(const PanelDockWidget* dock) const;

  /**
   * @brief @c true if @p panel_type_id may have more than one dock instance.
   * @param panel_type_id Catalog type id.
   */
  bool panelTypeSupportsMultiInstance(const QString& panel_type_id) const;

  /**
   * @brief Refreshes channel lists in all Plot settings widgets.
   */
  void refreshAllPlotSettingsChannels();

  /**
   * @brief Adds @p dock to the Delete Panel submenu.
   * @param dock Deletable panel dock.
   */
  void registerDeletePanelAction(PanelDockWidget* dock);

  /**
   * @brief Registers @p dock with layout/chrome (menus, visibility hooks).
   * @param dock Newly created panel dock.
   */
  void registerPanelDock(PanelDockWidget* dock);

  /**
   * @brief Restores Image panel configs captured for the current session.
   */
  void restoreImagePanelConfigs();

  /**
   * @brief Restores Plot panel configs captured for the current session.
   */
  void restorePlotPanelConfigs();

  /**
   * @brief Restores Table panel configs captured for the current session.
   */
  void restoreTablePanelConfigs();

  /**
   * @brief Restores Channel Graph panel configs captured for the current session.
   */
  void restoreChannelGraphPanelConfigs();

  /**
   * @brief Restores Transform Tree panel configs captured for the current session.
   */
  void restoreTfTreePanelConfigs();

  /**
   * @brief Restores Publish panel configs captured for the current session.
   */
  void restorePublishPanelConfigs();

  /**
   * @brief Restores Service Call panel configs captured for the current session.
   */
  void restoreServicePanelConfigs();

  /**
   * @brief Restores Teleop panel configs captured for the current session.
   */
  void restoreTeleopPanelConfigs();

  /**
   * @brief Restores Map panel configs captured for the current session.
   */
  void restoreMapPanelConfigs();

  /**
   * @brief Sets the active Image panel (inspector + title focus).
   * @param panel Active image panel, or @c nullptr to clear.
   */
  void setActiveImagePanel(image::ImagePanel* panel);

  /**
   * @brief Sets the active Map panel for inspector binding.
   * @param panel Active map panel, or @c nullptr to clear.
   */
  void setActiveMapPanel(map::MapPanel* panel);

  /**
   * @brief Sets the active Plot panel for inspector binding.
   * @param panel Active plot panel, or @c nullptr to clear.
   */
  void setActivePlotPanel(plot::PlotPanel* panel);

  /**
   * @brief Sets the active Publish panel for inspector binding.
   * @param panel Active publish panel, or @c nullptr to clear.
   */
  void setActivePublishPanel(publish_panel::PublishPanel* panel);

  /**
   * @brief Sets the active Service panel for inspector binding.
   * @param panel Active service panel, or @c nullptr to clear.
   */
  void setActiveServicePanel(service_panel::ServicePanel* panel);

  /**
   * @brief Sets the active Teleop panel for inspector binding.
   * @param panel Active teleop panel, or @c nullptr to clear.
   */
  void setActiveTeleopPanel(teleop::TeleopPanel* panel);

  /**
   * @brief Rebuilds enabled state of Delete Panel menu entries.
   */
  void syncDeletePanelMenu();

  /**
   * @brief Allocates a unique dock @c objectName based on @p base.
   *
   * @param base Preferred name (e.g. @c ImageDock); suffix added on collision.
   * @return Unique object name not used by any existing dock.
   */
  QString uniquePanelObjectName(const QString& base) const;

  /**
   * @brief Removes @p dock from the Delete Panel submenu.
   * @param dock Dock being destroyed or unregistered.
   */
  void unregisterDeletePanelAction(PanelDockWidget* dock);

  /**
   * @brief Updates Image dock window title from panel channel / state.
   * @param dock Host dock.
   * @param panel Image panel content.
   */
  void updateImageDockTitle(PanelDockWidget* dock, image::ImagePanel* panel);

  /**
   * @brief Updates Map dock window title from panel state.
   * @param dock Host dock.
   * @param panel Map panel content.
   */
  void updateMapDockTitle(PanelDockWidget* dock, map::MapPanel* panel);

  /**
   * @brief Updates Plot dock window title from panel state.
   * @param dock Host dock.
   * @param panel Plot panel content.
   */
  void updatePlotDockTitle(PanelDockWidget* dock, plot::PlotPanel* panel);

  /** @brief Updates Table dock title from panel config. */
  void updateTableDockTitle(PanelDockWidget* dock, table_panel::TablePanel* panel);

  /**
   * @brief Updates Publish dock window title from panel state.
   * @param dock Host dock.
   * @param panel Publish panel content.
   */
  void updatePublishDockTitle(PanelDockWidget* dock,
                              publish_panel::PublishPanel* panel);

  /**
   * @brief Updates Service dock window title from panel state.
   * @param dock Host dock.
   * @param panel Service panel content.
   */
  void updateServiceDockTitle(PanelDockWidget* dock,
                              service_panel::ServicePanel* panel);

  /**
   * @brief Updates Teleop dock window title from panel state.
   * @param dock Host dock.
   * @param panel Teleop panel content.
   */
  void updateTeleopDockTitle(PanelDockWidget* dock, teleop::TeleopPanel* panel);

  /**
   * @brief Connects Channel Graph panel signals to frame / inspector.
   * @param dock Host dock.
   * @param panel Channel graph content widget.
   */
  void wireChannelGraphPanel(PanelDockWidget* dock,
                             channel_graph::ChannelGraphPanel* panel);

  /**
   * @brief Connects Image panel signals (focus, title, inspector).
   * @param dock Host dock.
   * @param panel Image content widget.
   */
  void wireImagePanel(PanelDockWidget* dock, image::ImagePanel* panel);

  /**
   * @brief Connects Map panel signals.
   * @param dock Host dock.
   * @param panel Map content widget.
   */
  void wireMapPanel(PanelDockWidget* dock, map::MapPanel* panel);

  /**
   * @brief Connects Plot panel signals (focus, title, CSV download, inspector).
   * @param dock Host dock.
   * @param panel Plot content widget.
   */
  void wirePlotPanel(PanelDockWidget* dock, plot::PlotPanel* panel);

  /**
   * @brief Connects Table panel signals.
   * @param dock Host dock.
   * @param panel Table content widget.
   */
  void wireTablePanel(PanelDockWidget* dock, table_panel::TablePanel* panel);

  /**
   * @brief Connects Publish panel signals.
   * @param dock Host dock.
   * @param panel Publish content widget.
   */
  void wirePublishPanel(PanelDockWidget* dock,
                        publish_panel::PublishPanel* panel);

  /**
   * @brief Connects Service panel signals.
   * @param dock Host dock.
   * @param panel Service content widget.
   */
  void wireServicePanel(PanelDockWidget* dock,
                        service_panel::ServicePanel* panel);

  /**
   * @brief Connects Teleop panel signals.
   * @param dock Host dock.
   * @param panel Teleop content widget.
   */
  void wireTeleopPanel(PanelDockWidget* dock, teleop::TeleopPanel* panel);

  /**
   * @brief Connects TF Tree panel signals.
   * @param dock Host dock.
   * @param panel TF tree content widget.
   */
  void wireTfTreePanel(PanelDockWidget* dock, TfTreePanel* panel);

  /**
   * @brief Connects Record panel signals (open file, title-bar actions).
   * @param dock Host dock.
   * @param panel Record content widget.
   */
  void wireRecordPanel(PanelDockWidget* dock, RecordPanel* panel);

  /**
   * @brief Add Panel entry point (dialog / catalog → create and place dock).
   */
  void onAddPanel();

 private:
  /** Non-owning back-pointer to the main window. */
  VisualizationFrame* frame_ = nullptr;

  /** Legacy / channel raw-messages dock (when present). */
  PanelDockWidget* channel_dock_ = nullptr;

  /** Channels list dock. */
  PanelDockWidget* channels_dock_ = nullptr;

  /** Displays (visualization tree) dock. */
  PanelDockWidget* displays_dock_ = nullptr;

  /** Record playback dock (right sidebar, rqt_bag-style). */
  PanelDockWidget* record_dock_ = nullptr;

  /** Property Inspector dock. */
  PanelDockWidget* properties_dock_ = nullptr;

  /** Time / playback dock (bottom). */
  PanelDockWidget* time_dock_ = nullptr;

  /** Views (camera) dock. */
  PanelDockWidget* views_dock_ = nullptr;

  /** Tool Properties dock. */
  PanelDockWidget* tool_props_dock_ = nullptr;

  /** Selection details dock. */
  PanelDockWidget* selection_dock_ = nullptr;

  /** Primary TF Tree dock (multi-instance may create more). */
  PanelDockWidget* tf_dock_ = nullptr;

  /** Primary Channel Graph dock. */
  PanelDockWidget* channel_graph_dock_ = nullptr;

  /** Primary Teleop dock. */
  PanelDockWidget* teleop_dock_ = nullptr;

  /** Primary Image dock. */
  PanelDockWidget* image_dock_ = nullptr;

  /** Primary Plot dock. */
  PanelDockWidget* plot_dock_ = nullptr;

  /** Displays panel content (non-owning; child of @c displays_dock_). */
  DisplaysPanel* displays_panel_ = nullptr;

  /** Time panel content. */
  TimePanel* time_panel_ = nullptr;

  /** Views panel content. */
  ViewsPanel* views_panel_ = nullptr;

  /** Tool properties panel content. */
  ToolPropertiesPanel* tool_properties_panel_ = nullptr;

  /** Selection panel content. */
  SelectionPanel* selection_panel_ = nullptr;

  /** Raw messages panel content. */
  RawMessagesPanel* raw_messages_panel_ = nullptr;

  /** Channels panel content. */
  ChannelsPanel* channels_panel_ = nullptr;

  /** Record playback panel content. */
  RecordPanel* record_panel_ = nullptr;

  /** Primary Image panel content. */
  image::ImagePanel* image_panel_ = nullptr;

  /**
   * Per Image-display viewer docks keyed by display name (RViz2 associated
   * widget). Flexible outer-frame docks (any-area snap); not persisted as
   * @c ImagePanelConfig.
   */
  QHash<QString, QPointer<PanelDockWidget>> image_display_docks_;

  /** Primary Plot panel content. */
  plot::PlotPanel* plot_panel_ = nullptr;

  /** Last-focused Plot panel for inspector. */
  plot::PlotPanel* active_plot_panel_ = nullptr;

  /** Last-focused Image panel for inspector. */
  image::ImagePanel* active_image_panel_ = nullptr;

  /** Last-focused Teleop panel for inspector. */
  teleop::TeleopPanel* active_teleop_panel_ = nullptr;

  /** Last-focused Publish panel for inspector. */
  publish_panel::PublishPanel* active_publish_panel_ = nullptr;

  /** Last-focused Service panel for inspector. */
  service_panel::ServicePanel* active_service_panel_ = nullptr;

  /** Last-focused Map panel for inspector. */
  map::MapPanel* active_map_panel_ = nullptr;

  /** Plot currently shown in the Property Inspector. */
  plot::PlotPanel* inspector_plot_panel_ = nullptr;

  /** Image currently shown in the Property Inspector. */
  image::ImagePanel* inspector_image_panel_ = nullptr;

  /** Teleop currently shown in the Property Inspector. */
  teleop::TeleopPanel* inspector_teleop_panel_ = nullptr;

  /** Publish currently shown in the Property Inspector. */
  publish_panel::PublishPanel* inspector_publish_panel_ = nullptr;

  /** Service currently shown in the Property Inspector. */
  service_panel::ServicePanel* inspector_service_panel_ = nullptr;

  /** Map currently shown in the Property Inspector. */
  map::MapPanel* inspector_map_panel_ = nullptr;

  /** Shared Property Inspector panel widget. */
  PropertyInspectorPanel* property_inspector_panel_ = nullptr;

  /** Primary TF Tree panel content. */
  TfTreePanel* tf_tree_panel_ = nullptr;

  /** Delete-menu actions keyed by dock. */
  QHash<PanelDockWidget*, QAction*> delete_panel_actions_;
};

}  // namespace autoviz
