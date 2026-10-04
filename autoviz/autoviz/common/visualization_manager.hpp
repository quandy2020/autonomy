/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file visualization_manager.hpp
 * @brief Central runtime hub for Autoviz displays, TF, tools, views, and session I/O.
 *
 * Owns Autolink context, the display tree, frame/transform/view/tool/selection
 * managers, and window/panel persist state. The main window
 * (@c VisualizationFrame) drives @ref update() and binds UI callbacks.
 *
 * @see DisplayContext
 * @see SessionConfig
 * @see FrameManager
 * @see ToolManager
 * @see ViewManager
 */

#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <QImage>
#include <QString>
#include <QVector3D>

#include "autolink/node/node.hpp"
#include "autoviz/transform/buffer.hpp"
#include "autoviz/common/session_config.hpp"
#include "autoviz/common/selection_manager.hpp"
#include "autoviz/common/selection_handler.hpp"
#include "autoviz/common/pick_registry.hpp"
#include "autoviz/common/frame_manager.hpp"
#include "autoviz/common/transformation_manager.hpp"
#include "autoviz/common/view_manager.hpp"
#include "autoviz/common/tool_manager.hpp"
#include "autoviz/display/display.hpp"
#include "autoviz/display/interactive_marker_registry.hpp"
#include "autoviz/integration/autolink_context.hpp"
#include "autoviz/integration/channel_manager.hpp"
#include "autoviz/integration/playback_controller.hpp"
#include "autoviz/rendering/scene_overlay.hpp"
#include "autoviz/transform/listener.hpp"

namespace autoviz {
namespace common {

/**
 * @enum TimeSyncMode
 * @brief How simulation time is synchronized with a source channel.
 *
 * Mirrors @ref FrameManager::SyncMode at the VisualizationManager level for
 * wall / sim clocks shown in the UI.
 */
enum class TimeSyncMode {
  kOff = 0,          /**< No sync; use wall / free-running sim time. */
  kExact = 1,        /**< Exact stamp sync with @ref timeSyncSource. */
  kApproximate = 2,  /**< Approximate time sync. */
};

/**
 * @class VisualizationManager
 * @brief Process-level coordinator for visualization runtime state.
 *
 * ## Lifecycle
 *
 * Call @ref initialize() once with the binary name, then @ref update() from
 * the UI timer. @ref shutdown() tears down Autolink and displays.
 *
 * ## Display tree
 *
 * Top-level displays (and nested Group children) are owned in @c displays_.
 * @c display_views_ is a flat view used by the Displays panel. Mutators such
 * as @ref addDisplay / @ref moveDisplay follow RViz-style Group semantics.
 *
 * @note Does not own the Qt main window; layout strings and callbacks let
 *       VisualizationFrame apply geometry and redraw.
 */
class VisualizationManager {
 public:
  /**
   * @brief Constructs an uninitialized manager (call @ref initialize).
   */
  VisualizationManager();

  /**
   * @brief Destroys displays and releases Autolink resources if still live.
   */
  ~VisualizationManager();

  /**
   * @brief Initializes Autolink, TF, registries, and default tools/displays.
   *
   * @param binary_name Process name for Autolink / logging.
   * @return @c true on success.
   */
  bool initialize(const char* binary_name);

  /**
   * @brief Shuts down Autolink, clears displays, and marks the manager idle.
   */
  void shutdown();

  /**
   * @brief Whether the manager is initialized and usable.
   * @return @c true after successful @ref initialize until @ref shutdown.
   */
  bool ok() const;

  /**
   * @brief Per-frame update: time, channels, displays, tools, overlays.
   *
   * Sets @c updating_ while drawing to guard reentrancy.
   */
  void update();

  /**
   * @brief True while a display/tool draw pass is in progress (reentrancy guard).
   * @return @c true during @ref update draw.
   */
  bool isUpdating() const { return updating_; }

  /**
   * @brief Shared immediate-mode scene overlay.
   * @return Mutable overlay reference.
   */
  rendering::SceneOverlay& sceneOverlay() { return scene_overlay_; }

  /**
   * @brief Context passed to Display / Tool plugins.
   * @return Mutable @ref DisplayContext.
   */
  DisplayContext& displayContext() { return display_context_; }

  /**
   * @brief Fixed-frame / TF lookup facade.
   * @return Mutable @ref FrameManager.
   */
  FrameManager& frameManager() { return frame_manager_; }

  /**
   * @brief Active FrameTransformer selection manager.
   * @return Mutable @ref TransformationManager.
   */
  TransformationManager& transformationManager() {
    return transformation_manager_;
  }

  /**
   * @brief Const access to the transformation manager.
   * @return Const @ref TransformationManager.
   */
  const TransformationManager& transformationManager() const {
    return transformation_manager_;
  }

  /**
   * @brief Saved-view / current-view config manager.
   * @return Mutable @ref ViewManager.
   */
  ViewManager& viewManager() { return view_manager_; }

  /**
   * @brief Global selection set.
   * @return Mutable @ref SelectionManager.
   */
  SelectionManager& selectionManager() { return selection_manager_; }

  /**
   * @brief Const selection manager.
   * @return Const @ref SelectionManager.
   */
  const SelectionManager& selectionManager() const {
    return selection_manager_;
  }

  /**
   * @brief Per-frame pick metadata registry.
   * @return Mutable @ref PickRegistry.
   */
  PickRegistry& pickRegistry() { return pick_registry_; }

  /**
   * @brief Pick-handle → SelectionHandler map.
   * @return Mutable @ref HandlerManager.
   */
  HandlerManager& handlerManager() { return handler_manager_; }

  /**
   * @brief Cached Autolink channel list from the last refresh.
   * @return Const channel info vector.
   */
  const std::vector<integration::ChannelInfo>& channels() const {
    return cached_channels_;
  }

  /**
   * @brief Refreshes @c cached_channels_ from @c ChannelManager.
   */
  void refreshChannelList();

  /**
   * @brief Flat list of top-level display pointers for the Displays panel.
   * @return Const vector of non-owning display pointers.
   */
  const std::vector<display::Display*>& displays() const {
    return display_views_;
  }

  /**
   * @brief Current fixed frame id.
   * @return Fixed frame string.
   */
  const std::string& fixedFrame() const { return fixed_frame_; }

  /**
   * @brief Sets the fixed frame and propagates to FrameManager / displays.
   * @param frame New fixed frame id.
   */
  void setFixedFrame(const std::string& frame);

  /**
   * @brief Background color as @c "R;G;B".
   * @return Color string.
   */
  const std::string& backgroundColor() const { return background_color_; }

  /**
   * @brief Sets the background color and invokes the UI callback if set.
   * @param color @c "R;G;B" string.
   */
  void setBackgroundColor(const std::string& color);

  /**
   * @brief Registers a callback when the background color changes.
   * @param callback Receives the new color string.
   */
  void setBackgroundColorCallback(
      std::function<void(const std::string&)> callback);

  /**
   * @brief Target render frame rate.
   * @return Frames per second.
   */
  int targetFrameRate() const { return target_frame_rate_; }

  /**
   * @brief Sets the target frame rate and invokes the UI callback if set.
   * @param rate Frames per second.
   */
  void setTargetFrameRate(int rate);

  /**
   * @brief Registers a callback when the target frame rate changes.
   * @param callback Receives the new rate.
   */
  void setFrameRateCallback(std::function<void(int)> callback);

  /**
   * @brief Convenience: channel names from the cached channel list.
   * @return Name vector.
   */
  std::vector<std::string> channelNames() const;

  /**
   * @brief Registers a redraw request callback (viewport update).
   * @param callback No-arg functor.
   */
  void setRedrawCallback(std::function<void()> callback);

  /**
   * @brief Loads a session file and applies it via @ref applySession.
   *
   * @param path Path to @c .autoviz / YAML.
   * @return @c true on success.
   */
  bool loadSession(const std::string& path);

  /**
   * @brief Saves @ref currentSession() to a file.
   *
   * @param path Destination path.
   * @return @c true on success.
   */
  bool saveSession(const std::string& path) const;

  /**
   * @brief Captures the current runtime state as a @ref SessionConfig.
   * @return Session snapshot.
   */
  SessionConfig currentSession() const;

  /**
   * @brief Saved viewpoint bookmarks (delegates to @ref ViewManager).
   * @return Const bookmark list.
   */
  const std::vector<SavedViewConfig>& savedViews() const {
    return view_manager_.savedViews();
  }

  /**
   * @brief Replaces saved viewpoint bookmarks.
   * @param views New bookmark list.
   */
  void setSavedViews(const std::vector<SavedViewConfig>& views) {
    view_manager_.setSavedViews(views);
  }

  /**
   * @brief Whether a current-view snapshot is stored.
   * @return @c true if present.
   */
  bool hasCurrentView() const { return view_manager_.hasCurrentView(); }

  /**
   * @brief Current-view snapshot.
   * @return Const @ref SavedViewConfig.
   */
  const SavedViewConfig& currentView() const {
    return view_manager_.currentView();
  }

  /**
   * @brief Stores the current-view snapshot and updates the controller name.
   * @param view Snapshot including type and camera fields.
   */
  void setCurrentView(const SavedViewConfig& view) {
    view_manager_.setCurrentView(view);
    view_controller_name_ = view.type;
  }

  /**
   * @brief Registers a selection-changed listener.
   * @param callback Receives the new selection list.
   */
  void setSelectionChangedCallback(
      std::function<void(const std::vector<SelectionEntry>&)> callback);

  /**
   * @brief Registers a “focus camera on selection” listener.
   * @param callback Receives the world-space focus target.
   */
  void setSelectionFocusCallback(std::function<void(const QVector3D&)> callback);

  /**
   * @brief Shared Autolink node (for tools / panels that publish).
   * @return Shared node pointer (may be empty before initialize).
   */
  std::shared_ptr<::autolink::Node> autolinkNode() const;

  /**
   * @brief Registers a callback when an Image display produces a new frame.
   * @param callback Receives source name and image.
   */
  void setImageUpdateCallback(
      std::function<void(const QString&, const QImage&)> callback);

  /**
   * @brief Most recent image from any Image display.
   * @return Qt image (may be null).
   */
  QImage latestImage() const { return latest_image_; }

  /**
   * @brief Source label for @ref latestImage().
   * @return Display / channel name string.
   */
  QString latestImageSource() const { return latest_image_source_; }

  /**
   * @brief Resolves a display by top-level index and optional child index.
   *
   * @param index Top-level display index.
   * @param child_index Child within a Group (−1 = the top-level itself).
   * @return Display pointer, or @c nullptr if out of range.
   */
  display::Display* displayAt(std::size_t index, int child_index = -1);

  /**
   * @brief Const overload of @ref displayAt.
   * @param index Top-level index.
   * @param child_index Child index (−1 = top-level).
   * @return Const display pointer, or @c nullptr.
   */
  const display::Display* displayAt(std::size_t index, int child_index = -1) const;

  /**
   * @brief Enables or disables a display (and updates UI status).
   *
   * @param index Top-level index.
   * @param enabled Desired enabled state.
   * @param child_index Child index (−1 = top-level).
   */
  void setDisplayEnabled(std::size_t index, bool enabled, int child_index = -1);

  /**
   * @brief Sets the subscribed channel for a display.
   *
   * @param index Top-level index.
   * @param channel Channel name.
   * @param child_index Child index (−1 = top-level).
   */
  void setDisplayChannel(std::size_t index, const std::string& channel,
                         int child_index = -1);

  /**
   * @brief Sets one display property key/value.
   *
   * @param index Top-level index.
   * @param key Property key.
   * @param value Property value string.
   * @param child_index Child index (−1 = top-level).
   */
  void setDisplayProperty(std::size_t index, const std::string& key,
                          const std::string& value, int child_index = -1);

  /**
   * @brief Creates and attaches a display from config.
   *
   * @param config Type, name, channel, properties, children.
   * @return @c true if the type was known and the display was added.
   */
  bool addDisplay(const DisplayConfig& config);

  /**
   * @brief Removes a display (or a child of a Group).
   *
   * @param index Top-level index.
   * @param child_index Child index (−1 = remove top-level).
   * @return @c true if removed.
   */
  bool removeDisplay(std::size_t index, int child_index = -1);

  /**
   * @brief Duplicates a top-level display (deep copy of config).
   *
   * @param index Top-level index to clone.
   * @return @c true on success.
   */
  bool duplicateDisplay(std::size_t index);

  /**
   * @brief Renames a top-level display.
   *
   * @param index Top-level index.
   * @param name New instance name.
   */
  void setDisplayName(std::size_t index, const std::string& name);

  /**
   * @brief Moves a display into a Group, to root, or reorders at root (RViz-style).
   *
   * @param from_index Source top-level index.
   * @param from_child_index Source child (−1 = top-level node).
   * @param to_group_index Destination Group top-level index.
   * @param to_child_index Insert position within the Group.
   * @return @c true on success.
   */
  bool moveDisplay(std::size_t from_index, int from_child_index,
                   std::size_t to_group_index, int to_child_index);

  /**
   * @brief Moves a display (or child) to a root insert position.
   *
   * @param from_index Source top-level index.
   * @param from_child_index Source child (−1 = top-level).
   * @param root_insert_index Destination index among top-level displays.
   * @return @c true on success.
   */
  bool moveDisplayToRoot(std::size_t from_index, int from_child_index,
                         std::size_t root_insert_index);

  /**
   * @brief Reorders two top-level displays.
   *
   * @param from_index Source index.
   * @param to_index Destination index.
   * @return @c true on success.
   */
  bool reorderRootDisplay(std::size_t from_index, std::size_t to_index);

  /**
   * @brief Returns whether @p target is @p root or nested under it.
   *
   * Used to prevent illegal Group moves (dropping a Group into itself).
   *
   * @param root Candidate ancestor.
   * @param target Candidate descendant.
   * @return @c true if @p target is in @p root's subtree.
   */
  static bool isDisplayInSubtree(const display::Display* root,
                                 const display::Display* target);

  /**
   * @brief Stores main-window Qt state and geometry (base64).
   *
   * @param state_b64 @c QMainWindow::saveState encoding.
   * @param geometry_b64 @c QWidget::saveGeometry encoding.
   */
  void setWindowLayout(const std::string& state_b64,
                       const std::string& geometry_b64);

  /**
   * @brief Stores central / main panel Qt state (base64).
   * @param state_b64 Panel state encoding.
   */
  void setMainPanelLayout(const std::string& state_b64);

  /**
   * @brief Sets left/right dock hide flags.
   *
   * @param hide_left Hide left dock strip.
   * @param hide_right Hide right dock strip.
   */
  void setDockHideState(bool hide_left, bool hide_right);

  /**
   * @brief Replaces per-panel collapse layout records.
   * @param layouts Panel layout configs.
   */
  void setPanelLayouts(const std::vector<PanelLayoutConfig>& layouts);

  /**
   * @brief Sets which named panels should be visible.
   * @param panels Panel name list.
   */
  void setVisiblePanels(const std::vector<std::string>& panels);

  /**
   * @brief Replaces Plot panel persist records.
   * @param panels Plot panel configs.
   */
  void setPlotPanels(const std::vector<PlotPanelPersistConfig>& panels);

  /**
   * @brief Replaces Image panel persist records.
   * @param panels Image panel configs.
   */
  void setImagePanels(const std::vector<ImagePanelPersistConfig>& panels);

  /**
   * @brief Replaces Publish panel persist records.
   * @param panels Publish panel configs.
   */
  void setPublishPanels(const std::vector<PublishPanelPersistConfig>& panels);

  /**
   * @brief Replaces Service Call panel persist records.
   * @param panels Service panel configs.
   */
  void setServicePanels(const std::vector<ServicePanelPersistConfig>& panels);

  /**
   * @brief Replaces Teleop panel persist records.
   * @param panels Teleop panel configs.
   */
  void setTeleopPanels(const std::vector<TeleopPanelPersistConfig>& panels);

  /**
   * @brief Replaces Map panel persist records.
   * @param panels Map panel configs.
   */
  void setMapPanels(const std::vector<MapPanelPersistConfig>& panels);

  /**
   * @brief Replaces Channels sidebar persist state.
   * @param config Channels browser config.
   */
  void setChannelsBrowser(const ChannelsBrowserPersistConfig& config);

  /**
   * @brief Replaces Raw Messages panel persist state.
   * @param config Raw messages config.
   */
  void setRawMessages(const RawMessagesPersistConfig& config);

  /**
   * @brief Replaces Table panel persist records.
   * @param panels Table panel configs.
   */
  void setTablePanels(const std::vector<TablePanelPersistConfig>& panels);

  /**
   * @brief Replaces Channel Graph panel persist records.
   * @param panels Channel Graph panel configs.
   */
  void setChannelGraphPanels(
      const std::vector<ChannelGraphPanelPersistConfig>& panels);

  /**
   * @brief Replaces Transform Tree panel persist records.
   * @param panels TF Tree panel configs.
   */
  void setTfTreePanels(const std::vector<TfTreePanelPersistConfig>& panels);

  /**
   * @brief Sets the global plot-settings visibility flag.
   * @param visible Whether plot settings are shown.
   */
  void setPlotSettingsVisible(bool visible);

  /**
   * @brief Stores absolute window frame geometry (−1 = unset).
   *
   * @param x Left.
   * @param y Top.
   * @param width Width.
   * @param height Height.
   */
  void setWindowFrame(int x, int y, int width, int height);

  /** @brief Whether the left dock strip is hidden. */
  bool hideLeftDock() const { return hide_left_dock_; }

  /** @brief Whether the right dock strip is hidden. */
  bool hideRightDock() const { return hide_right_dock_; }

  /** @brief Per-panel collapse layout records. */
  const std::vector<PanelLayoutConfig>& panelLayouts() const {
    return panel_layouts_;
  }

  /** @brief Visible panel name list. */
  const std::vector<std::string>& visiblePanels() const {
    return visible_panels_;
  }

  /** @brief Plot panel persist records. */
  const std::vector<PlotPanelPersistConfig>& plotPanels() const {
    return plot_panels_;
  }

  /** @brief Image panel persist records. */
  const std::vector<ImagePanelPersistConfig>& imagePanels() const {
    return image_panels_;
  }

  /** @brief Publish panel persist records. */
  const std::vector<PublishPanelPersistConfig>& publishPanels() const {
    return publish_panels_;
  }

  /** @brief Service Call panel persist records. */
  const std::vector<ServicePanelPersistConfig>& servicePanels() const {
    return service_panels_;
  }

  /** @brief Teleop panel persist records. */
  const std::vector<TeleopPanelPersistConfig>& teleopPanels() const {
    return teleop_panels_;
  }

  /** @brief Map panel persist records. */
  const std::vector<MapPanelPersistConfig>& mapPanels() const {
    return map_panels_;
  }

  /** @brief Channels sidebar persist state. */
  const ChannelsBrowserPersistConfig& channelsBrowser() const {
    return channels_browser_;
  }

  /** @brief Raw Messages panel persist state. */
  const RawMessagesPersistConfig& rawMessages() const { return raw_messages_; }

  /** @brief Table panel persist records. */
  const std::vector<TablePanelPersistConfig>& tablePanels() const {
    return table_panels_;
  }

  /** @brief Channel Graph panel persist records. */
  const std::vector<ChannelGraphPanelPersistConfig>& channelGraphPanels() const {
    return channel_graph_panels_;
  }

  /** @brief Transform Tree panel persist records. */
  const std::vector<TfTreePanelPersistConfig>& tfTreePanels() const {
    return tf_tree_panels_;
  }

  /** @brief Global plot-settings visibility. */
  bool plotSettingsVisible() const { return plot_settings_visible_; }

  /** @brief Stored window X (−1 = default). */
  int windowX() const { return window_x_; }

  /** @brief Stored window Y (−1 = default). */
  int windowY() const { return window_y_; }

  /** @brief Stored window width (−1 = default). */
  int windowWidth() const { return window_width_; }

  /** @brief Stored window height (−1 = default). */
  int windowHeight() const { return window_height_; }

  /** @brief Main window Qt state (base64). */
  std::string windowStateBase64() const { return window_state_b64_; }

  /** @brief Main panel Qt state (base64). */
  std::string mainPanelStateBase64() const { return main_panel_state_b64_; }

  /** @brief Main window geometry (base64). */
  std::string windowGeometryBase64() const { return window_geometry_b64_; }

  /**
   * @brief Active view-controller type name.
   * @return Type string (e.g. @c "Orbit").
   */
  const std::string& viewControllerName() const { return view_controller_name_; }

  /**
   * @brief Sets the view-controller type name and notifies the UI callback.
   * @param name Type id known to @ref ViewControllerRegistry.
   */
  void setViewControllerName(const std::string& name);

  /**
   * @brief Registers a callback when the view-controller type changes.
   * @param callback Receives the new type name.
   */
  void setViewControllerCallback(std::function<void(const std::string&)> callback);

  /**
   * @brief Active render backend name.
   * @return Backend string (e.g. @c "OpenGL").
   */
  const std::string& renderBackendName() const { return render_backend_name_; }

  /**
   * @brief Sets the render backend name and notifies the UI callback.
   * @param name Backend id.
   */
  void setRenderBackendName(const std::string& name);

  /**
   * @brief Registers a callback when the render backend changes.
   * @param callback Receives the new backend name.
   */
  void setRenderBackendCallback(std::function<void(const std::string&)> callback);

  /**
   * @brief Bag / recording playback controller.
   * @return Mutable @ref integration::PlaybackController.
   */
  integration::PlaybackController& playback() { return playback_; }

  /**
   * @brief Interactive tool manager.
   * @return Mutable @ref ToolManager.
   */
  ToolManager& tools() { return tool_manager_; }

  /**
   * @brief Const tool manager.
   * @return Const @ref ToolManager.
   */
  const ToolManager& tools() const { return tool_manager_; }

  /**
   * @brief Tool ids shown on the toolbar.
   * @return Ordered id list.
   */
  const std::vector<std::string>& toolbarTools() const {
    return tool_manager_.toolbarToolIds();
  }

  /**
   * @brief Replaces the toolbar tool id list.
   * @param tools Ordered tool ids.
   */
  void setToolbarTools(const std::vector<std::string>& tools) {
    tool_manager_.setToolbarToolIds(tools);
  }

  /**
   * @brief Wall-clock time in seconds since an arbitrary origin.
   * @return Seconds.
   */
  double wallClockSec() const;

  /**
   * @brief Wall-clock elapsed seconds since @ref initialize / @ref resetTime.
   * @return Elapsed seconds.
   */
  double wallClockElapsedSec() const;

  /**
   * @brief Simulation / synced time in seconds.
   * @return Seconds.
   */
  double simTimeSec() const;

  /**
   * @brief Simulation time elapsed since reset.
   * @return Elapsed seconds.
   */
  double simTimeElapsedSec() const;

  /**
   * @brief Whether sim time is paused.
   * @return Pause flag.
   */
  bool timePaused() const { return time_paused_; }

  /**
   * @brief Current time sync mode.
   * @return @ref TimeSyncMode.
   */
  TimeSyncMode timeSyncMode() const { return time_sync_mode_; }

  /**
   * @brief Channel / source used for time sync.
   * @return Source name string.
   */
  const std::string& timeSyncSource() const { return time_sync_source_; }

  /**
   * @brief Pauses or resumes sim time advancement.
   * @param paused @c true to freeze.
   */
  void setTimePaused(bool paused);

  /**
   * @brief Sets the time synchronization mode.
   * @param mode Sync strategy.
   */
  void setTimeSyncMode(TimeSyncMode mode);

  /**
   * @brief Sets the channel / source for time sync.
   * @param source Source name.
   */
  void setTimeSyncSource(const std::string& source);

  /**
   * @brief Resets wall and sim time origins.
   */
  void resetTime();

  /**
   * @brief Interactive marker registry shared by displays / tools.
   * @return Mutable registry.
   */
  display::InteractiveMarkerRegistry& interactiveMarkerRegistry() {
    return interactive_marker_registry_;
  }

  /**
   * @brief Shared TF buffer pointer (non-owning).
   * @return Buffer, or @c nullptr before initialize.
   */
  autoviz::transform::Buffer* tfBuffer() const { return tf_buffer_; }

 private:
  /**
   * @brief Applies a loaded session to runtime state (displays, views, layout).
   * @param config Session to apply.
   */
  void applySession(const SessionConfig& config);

  /**
   * @brief Refreshes @c display_context_ pointers from owned members.
   */
  void syncDisplayContext();

  /**
   * @brief Takes ownership of a display, enables it, and wires context.
   *
   * @param display New display instance.
   * @param enabled Initial enabled flag.
   */
  void attachDisplay(std::unique_ptr<display::Display> display, bool enabled);

  /**
   * @brief Recursively prepares a display (and children) after attach.
   * @param display Display to prepare.
   */
  void prepareDisplayTree(display::Display* display);

  /**
   * @brief Rebuilds @c display_views_ from @c displays_.
   */
  void rebuildDisplayViews();

  /**
   * @brief Removes and returns a display from the tree.
   *
   * @param index Top-level index.
   * @param child_index Child index (−1 = top-level).
   * @return Owned display, or empty unique_ptr on failure.
   */
  std::unique_ptr<display::Display> takeDisplay(std::size_t index,
                                                int child_index);

  integration::AutolinkContext autolink_;
  std::unique_ptr<integration::ChannelManager> channel_manager_;
  autoviz::transform::Buffer* tf_buffer_ = nullptr;
  autoviz::transform::Listener tf_listener_;
  rendering::SceneOverlay scene_overlay_;
  common::DisplayContext display_context_;
  std::vector<std::unique_ptr<display::Display>> displays_;
  std::vector<display::Display*> display_views_;
  std::vector<integration::ChannelInfo> cached_channels_;
  std::function<void()> redraw_callback_;
  std::function<void(const std::string&)> background_color_callback_;
  std::function<void(int)> frame_rate_callback_;
  std::function<void(const std::string&)> view_controller_callback_;
  std::function<void(const std::string&)> render_backend_callback_;
  integration::PlaybackController playback_;
  ToolManager tool_manager_;
  std::vector<VariablePersistConfig> session_variables_;
  FrameManager frame_manager_;
  TransformationManager transformation_manager_;
  ViewManager view_manager_;
  SelectionManager selection_manager_;
  PickRegistry pick_registry_;
  HandlerManager handler_manager_;
  display::InteractiveMarkerRegistry interactive_marker_registry_;
  std::string fixed_frame_ = "map";
  std::string background_color_ = "48;48;48";
  int target_frame_rate_ = 30;
  std::string view_controller_name_ = "Orbit";
  std::string render_backend_name_ = "Ogre";
  std::string window_state_b64_;
  std::string main_panel_state_b64_;
  std::string window_geometry_b64_;
  bool hide_left_dock_ = false;
  bool hide_right_dock_ = false;
  std::vector<PanelLayoutConfig> panel_layouts_;
  std::vector<std::string> visible_panels_;
  std::vector<PlotPanelPersistConfig> plot_panels_;
  std::vector<ImagePanelPersistConfig> image_panels_;
  std::vector<PublishPanelPersistConfig> publish_panels_;
  std::vector<ServicePanelPersistConfig> service_panels_;
  std::vector<TeleopPanelPersistConfig> teleop_panels_;
  std::vector<MapPanelPersistConfig> map_panels_;
  ChannelsBrowserPersistConfig channels_browser_;
  RawMessagesPersistConfig raw_messages_;
  std::vector<TablePanelPersistConfig> table_panels_;
  std::vector<ChannelGraphPanelPersistConfig> channel_graph_panels_;
  std::vector<TfTreePanelPersistConfig> tf_tree_panels_;
  bool plot_settings_visible_ = true;
  int window_x_ = -1;
  int window_y_ = -1;
  int window_width_ = -1;
  int window_height_ = -1;
  QImage latest_image_;
  QString latest_image_source_;
  std::function<void(const QString&, const QImage&)> image_update_callback_;
  bool initialized_ = false;
  bool updating_ = false;
  std::chrono::steady_clock::time_point wall_start_;
  double sim_origin_sec_ = 0.0;
  bool time_paused_ = false;
  double paused_sim_sec_ = 0.0;
  TimeSyncMode time_sync_mode_ = TimeSyncMode::kOff;
  std::string time_sync_source_;
};

}  // namespace common
}  // namespace autoviz
