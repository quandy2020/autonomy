/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

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
#include "autoviz/rendering/render_settings.hpp"
#include "autoviz/transform/listener.hpp"
#include "autoviz/variables/variable_store.hpp"

namespace autoviz {
namespace common {

enum class TimeSyncMode {
  kOff = 0,
  kExact = 1,
  kApproximate = 2,
};

class VisualizationManager {
 public:
  VisualizationManager();
  ~VisualizationManager();

  bool initialize(const char* binary_name);
  void shutdown();

  bool ok() const;
  void update();

  /** True while a display/tool draw pass is in progress (reentrancy guard). */
  bool isUpdating() const { return updating_; }

  rendering::SceneOverlay& sceneOverlay() { return scene_overlay_; }
  DisplayContext& displayContext() { return display_context_; }
  FrameManager& frameManager() { return frame_manager_; }
  TransformationManager& transformationManager() {
    return transformation_manager_;
  }
  const TransformationManager& transformationManager() const {
    return transformation_manager_;
  }
  ViewManager& viewManager() { return view_manager_; }
  SelectionManager& selectionManager() { return selection_manager_; }
  const SelectionManager& selectionManager() const {
    return selection_manager_;
  }
  PickRegistry& pickRegistry() { return pick_registry_; }
  HandlerManager& handlerManager() { return handler_manager_; }
  const std::vector<integration::ChannelInfo>& channels() const {
    return cached_channels_;
  }

  void refreshChannelList();
  const std::vector<display::Display*>& displays() const {
    return display_views_;
  }

  const std::string& fixedFrame() const { return fixed_frame_; }
  void setFixedFrame(const std::string& frame);

  bool showGrid() const { return show_grid_; }
  void setShowGrid(bool show);

  const rendering::ReferenceGridSettings& referenceGridSettings() const {
    return reference_grid_settings_;
  }

  const std::string& backgroundColor() const { return background_color_; }
  void setBackgroundColor(const std::string& color);
  void setBackgroundColorCallback(
      std::function<void(const std::string&)> callback);

  int targetFrameRate() const { return target_frame_rate_; }
  void setTargetFrameRate(int rate);
  void setFrameRateCallback(std::function<void(int)> callback);

  std::vector<std::string> channelNames() const;

  void setRedrawCallback(std::function<void()> callback);
  void setGridVisibilityCallback(std::function<void(bool)> callback);

  bool loadSession(const std::string& path);
  bool saveSession(const std::string& path) const;
  SessionConfig currentSession() const;

  const std::vector<SavedViewConfig>& savedViews() const {
    return view_manager_.savedViews();
  }
  void setSavedViews(const std::vector<SavedViewConfig>& views) {
    view_manager_.setSavedViews(views);
  }

  bool hasCurrentView() const { return view_manager_.hasCurrentView(); }
  const SavedViewConfig& currentView() const {
    return view_manager_.currentView();
  }
  void setCurrentView(const SavedViewConfig& view) {
    view_manager_.setCurrentView(view);
    view_controller_name_ = view.type;
  }

  void setSelectionChangedCallback(
      std::function<void(const std::vector<SelectionEntry>&)> callback);
  void setSelectionFocusCallback(std::function<void(const QVector3D&)> callback);

  std::shared_ptr<::autolink::Node> autolinkNode() const;

  void setImageUpdateCallback(
      std::function<void(const QString&, const QImage&)> callback);
  QImage latestImage() const { return latest_image_; }
  QString latestImageSource() const { return latest_image_source_; }

  display::Display* displayAt(std::size_t index, int child_index = -1);
  const display::Display* displayAt(std::size_t index, int child_index = -1) const;

  void setDisplayEnabled(std::size_t index, bool enabled, int child_index = -1);
  void setDisplayChannel(std::size_t index, const std::string& channel,
                         int child_index = -1);
  void setDisplayProperty(std::size_t index, const std::string& key,
                          const std::string& value, int child_index = -1);

  bool addDisplay(const DisplayConfig& config);
  bool removeDisplay(std::size_t index, int child_index = -1);
  bool duplicateDisplay(std::size_t index);
  void setDisplayName(std::size_t index, const std::string& name);
  /** Move a display into a Group, to root, or reorder at root (RViz-style). */
  bool moveDisplay(std::size_t from_index, int from_child_index,
                   std::size_t to_group_index, int to_child_index);
  bool moveDisplayToRoot(std::size_t from_index, int from_child_index,
                         std::size_t root_insert_index);
  bool reorderRootDisplay(std::size_t from_index, std::size_t to_index);

  static bool isDisplayInSubtree(const display::Display* root,
                                 const display::Display* target);

  void setWindowLayout(const std::string& state_b64,
                       const std::string& geometry_b64);
  void setMainPanelLayout(const std::string& state_b64);
  void setDockHideState(bool hide_left, bool hide_right);
  void setPanelLayouts(const std::vector<PanelLayoutConfig>& layouts);
  void setVisiblePanels(const std::vector<std::string>& panels);
  void setPlotPanels(const std::vector<PlotPanelPersistConfig>& panels);
  void setImagePanels(const std::vector<ImagePanelPersistConfig>& panels);
  void setStateTransitionPanels(
      const std::vector<StateTransitionPanelPersistConfig>& panels);
  void setPublishPanels(const std::vector<PublishPanelPersistConfig>& panels);
  void setPlotSettingsVisible(bool visible);
  void setWindowFrame(int x, int y, int width, int height);
  bool hideLeftDock() const { return hide_left_dock_; }
  bool hideRightDock() const { return hide_right_dock_; }
  const std::vector<PanelLayoutConfig>& panelLayouts() const {
    return panel_layouts_;
  }
  const std::vector<std::string>& visiblePanels() const {
    return visible_panels_;
  }
  const std::vector<PlotPanelPersistConfig>& plotPanels() const {
    return plot_panels_;
  }
  const std::vector<ImagePanelPersistConfig>& imagePanels() const {
    return image_panels_;
  }
  const std::vector<StateTransitionPanelPersistConfig>& stateTransitionPanels()
      const {
    return state_transition_panels_;
  }
  const std::vector<PublishPanelPersistConfig>& publishPanels() const {
    return publish_panels_;
  }
  bool plotSettingsVisible() const { return plot_settings_visible_; }
  int windowX() const { return window_x_; }
  int windowY() const { return window_y_; }
  int windowWidth() const { return window_width_; }
  int windowHeight() const { return window_height_; }
  std::string windowStateBase64() const { return window_state_b64_; }
  std::string mainPanelStateBase64() const { return main_panel_state_b64_; }
  std::string windowGeometryBase64() const { return window_geometry_b64_; }

  const std::string& viewControllerName() const { return view_controller_name_; }
  void setViewControllerName(const std::string& name);
  void setViewControllerCallback(std::function<void(const std::string&)> callback);

  const std::string& renderBackendName() const { return render_backend_name_; }
  void setRenderBackendName(const std::string& name);
  void setRenderBackendCallback(std::function<void(const std::string&)> callback);

  integration::PlaybackController& playback() { return playback_; }
  ToolManager& tools() { return tool_manager_; }
  const ToolManager& tools() const { return tool_manager_; }
  variables::VariableStore& variableStore() { return variable_store_; }
  const variables::VariableStore& variableStore() const {
    return variable_store_;
  }

  const std::vector<std::string>& toolbarTools() const {
    return tool_manager_.toolbarToolIds();
  }
  void setToolbarTools(const std::vector<std::string>& tools) {
    tool_manager_.setToolbarToolIds(tools);
  }

  double wallClockSec() const;
  double wallClockElapsedSec() const;
  double simTimeSec() const;
  double simTimeElapsedSec() const;
  bool timePaused() const { return time_paused_; }
  TimeSyncMode timeSyncMode() const { return time_sync_mode_; }
  const std::string& timeSyncSource() const { return time_sync_source_; }
  void setTimePaused(bool paused);
  void setTimeSyncMode(TimeSyncMode mode);
  void setTimeSyncSource(const std::string& source);
  void resetTime();

  display::InteractiveMarkerRegistry& interactiveMarkerRegistry() {
    return interactive_marker_registry_;
  }
  autoviz::transform::Buffer* tfBuffer() const { return tf_buffer_; }

 private:
  void applySession(const SessionConfig& config);
  void syncDisplayContext();
  void syncReferenceGridFromDisplays();
  void attachDisplay(std::unique_ptr<display::Display> display, bool enabled);
  void prepareDisplayTree(display::Display* display);
  void rebuildDisplayViews();
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
  std::function<void(bool)> grid_visibility_callback_;
  std::function<void(const std::string&)> background_color_callback_;
  std::function<void(int)> frame_rate_callback_;
  std::function<void(const std::string&)> view_controller_callback_;
  std::function<void(const std::string&)> render_backend_callback_;
  integration::PlaybackController playback_;
  ToolManager tool_manager_;
  variables::VariableStore variable_store_;
  FrameManager frame_manager_;
  TransformationManager transformation_manager_;
  ViewManager view_manager_;
  SelectionManager selection_manager_;
  PickRegistry pick_registry_;
  HandlerManager handler_manager_;
  display::InteractiveMarkerRegistry interactive_marker_registry_;
  std::string fixed_frame_ = "map";
  bool show_grid_ = true;
  rendering::ReferenceGridSettings reference_grid_settings_;
  std::string background_color_ = "48;48;48";
  int target_frame_rate_ = 30;
  std::string view_controller_name_ = "Orbit";
  std::string render_backend_name_ = "OpenGL";
  std::string window_state_b64_;
  std::string main_panel_state_b64_;
  std::string window_geometry_b64_;
  bool hide_left_dock_ = false;
  bool hide_right_dock_ = false;
  std::vector<PanelLayoutConfig> panel_layouts_;
  std::vector<std::string> visible_panels_;
  std::vector<PlotPanelPersistConfig> plot_panels_;
  std::vector<ImagePanelPersistConfig> image_panels_;
  std::vector<StateTransitionPanelPersistConfig> state_transition_panels_;
  std::vector<PublishPanelPersistConfig> publish_panels_;
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
