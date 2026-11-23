/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/visualization_manager.hpp"

#include "autoviz/bridge/ws_bridge.hpp"
#include "autolink/time/time.hpp"
#include <automsgs/msgs/time_utils.hpp>
#include "autoviz/common/display_catalog.hpp"
#include "autoviz/common/display_factory.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/common/display_context.hpp"
#include "autolink/service_discovery/topology_manager.hpp"
#include "autoviz/common/interaction/channel_reader_registry.hpp"
#include "autoviz/common/interaction/channel_stats_registry.hpp"
#include "autoviz/common/interaction/service_client_registry.hpp"
#include "autoviz/common/interaction/channel_writer_registry.hpp"
#include "autoviz/common/interaction/message_queue.hpp"
#include "autoviz/plugins/displays/display.hpp"
#include "autoviz/plugins/displays/display_group.hpp"
#include "autoviz/plugins/displays/interactive_markers/interactive_marker_display.hpp"
#include "autoviz/plugins/displays/interactive_markers/interactive_marker_registry.hpp"
#include "autoviz/rendering/mesh_resource.hpp"
#include "autoviz/rendering/ogre_scene_host.hpp"

#include <glog/logging.h>

#include <algorithm>
#include <functional>
#include <map>
#include <unordered_set>

namespace autoviz {
namespace common {

VisualizationManager::VisualizationManager() = default;

VisualizationManager::~VisualizationManager() { shutdown(); }

bool VisualizationManager::initialize(const char* binary_name) {
  if (initialized_) {
    return true;
  }
  if (!autolink_.initialize(binary_name, "autoviz")) {
    return false;
  }
  tf_buffer_ = autoviz::transform::Buffer::Instance();
  frame_manager_.setBuffer(tf_buffer_);
  transformation_manager_.initialize(tf_buffer_);
  transformation_manager_.setFrameManager(&frame_manager_);
  display_context_.autolink = &autolink_;
  integration::ChannelReaderRegistry::instance().setNode(autolink_.node());
  integration::ChannelWriterRegistry::instance().setNode(autolink_.node());
  integration::ServiceClientRegistry::instance().setNode(autolink_.node());
  display_context_.tf_buffer = tf_buffer_;
  tf_listener_.start(tf_buffer_);
  display_context_.frame_manager = &frame_manager_;
  display_context_.transformation_manager = &transformation_manager_;
  display_context_.pick_registry = &pick_registry_;
  display_context_.handler_manager = &handler_manager_;
  display_context_.selection_manager = &selection_manager_;
  display_context_.tool_manager = &tool_manager_;
  display_context_.view_manager = &view_manager_;
  playback_.setNode(autolink_.node());

  auto topology = ::autolink::service_discovery::TopologyManager::Instance();
  channel_manager_ = std::make_unique<integration::ChannelManager>(
      topology->channel_manager());

  if (ws_options_.enabled && ws_options_.port != 0) {
    ws_bridge_ =
        std::make_shared<bridge::WsBridge>(autolink_.node(), ws_options_);
    if (!ws_bridge_->Start()) {
      LOG(WARNING) << "WebSocket server failed to bind " << ws_options_.host
                   << ":" << ws_options_.port;
      ws_bridge_.reset();
    }
  }

  // Apply defaults. Channel-backed displays start disabled to avoid racing
  // loadConfig() (teardown can stall in RemoveCRoutine under publisher flood).
  // Channel-less displays (e.g. Grid) stay enabled so the default scene shows
  // a reference grid, matching RViz2.
  SessionConfig startup_config = SessionConfigIO::defaultConfig();
  std::function<void(std::vector<DisplayConfig>&)> disable_channel_displays =
      [&](std::vector<DisplayConfig>& displays) {
        for (auto& display : displays) {
          if (!display.channel.empty()) {
            display.enabled = false;
          }
          disable_channel_displays(display.children);
        }
      };
  disable_channel_displays(startup_config.displays);
  applySession(startup_config);
  refreshChannelList();
  // RViz-style package:// for bolza/ocs2_robotic_assets (env / install / source).
  rendering::MeshResourceResolver::instance().ensureDefaultPackagesRegistered();
  wall_start_ = std::chrono::steady_clock::now();
  sim_origin_sec_ = simTimeSec();
  initialized_ = true;
  return true;
}

void VisualizationManager::detachDisplaysFromScene() {
  // Suppress sync redraw while tearing down GPU-backed display state.
  redraw_callback_ = nullptr;
  display_context_.request_redraw = nullptr;
  for (auto& display : displays_) {
    if (display != nullptr) {
      display->setEnabled(false);
    }
  }
}

void VisualizationManager::setWsServerOptions(bridge::WsServerOptions options) {
  ws_options_ = std::move(options);
}

bool VisualizationManager::wsServerRunning() const {
  return ws_bridge_ != nullptr && ws_bridge_->IsRunning();
}

void VisualizationManager::shutdown() {
  if (!initialized_) {
    return;
  }
  // Suppress sync redraw while destroying displays (see applySession).
  redraw_callback_ = nullptr;
  display_context_.request_redraw = nullptr;
  for (auto& display : displays_) {
    if (display != nullptr) {
      display->setEnabled(false);
    }
  }
  displays_.clear();
  display_views_.clear();
  interactive_marker_registry_ = display::InteractiveMarkerRegistry{};
  channel_manager_.reset();
  if (ws_bridge_) {
    ws_bridge_->Stop();
    ws_bridge_.reset();
  }
  tf_listener_.stop();
  integration::ChannelReaderRegistry::instance().setNode(nullptr);
  integration::ChannelWriterRegistry::instance().setNode(nullptr);
  integration::ServiceClientRegistry::instance().setNode(nullptr);
  autolink_.shutdown();
  cached_channels_.clear();
  initialized_ = false;
}

bool VisualizationManager::ok() const { return autolink_.ok(); }

void VisualizationManager::setRedrawCallback(std::function<void()> callback) {
  redraw_callback_ = std::move(callback);
  syncDisplayContext();
}

void VisualizationManager::syncDisplayContext() {
  display_context_.fixed_frame = fixed_frame_;
  frame_manager_.setFixedFrame(fixed_frame_);
  display_context_.request_redraw = redraw_callback_;
  display_context_.image_updated =
      [this](const std::string& display_name, const QImage& image) {
        latest_image_ = image;
        latest_image_source_ = QString::fromStdString(display_name);
        if (image_update_callback_) {
          image_update_callback_(latest_image_source_, latest_image_);
        }
        if (redraw_callback_) {
          redraw_callback_();
        }
      };
}

void VisualizationManager::applySession(const SessionConfig& config) {
  // Tear-down must not invoke request_redraw: the frame callback calls
  // manager_->update() synchronously and would touch Displays mid-destroy.
  auto saved_redraw = std::move(redraw_callback_);
  redraw_callback_ = nullptr;
  display_context_.request_redraw = nullptr;

  for (auto& display : displays_) {
    display->setEnabled(false);
  }
  displays_.clear();
  display_views_.clear();
  interactive_marker_registry_ = display::InteractiveMarkerRegistry{};

  fixed_frame_ = config.fixed_frame;
  target_frame_rate_ = std::clamp(config.frame_rate, 1, 120);
  time_sync_mode_ = static_cast<TimeSyncMode>(
      std::clamp(config.time_sync_mode, 0, 2));
  time_sync_source_ = config.time_sync_source;
  background_color_ = config.background_color;
  view_controller_name_ = config.view_controller;
  render_backend_name_ =
      (config.render_backend == "Ogre") ? config.render_backend : "Ogre";
  window_state_b64_ = config.window_state_b64;
  main_panel_state_b64_ = config.main_panel_state_b64;
  window_geometry_b64_ = config.window_geometry_b64;
  hide_left_dock_ = config.hide_left_dock;
  hide_right_dock_ = config.hide_right_dock;
  panel_layouts_ = config.panel_layouts;
  visible_panels_ = config.visible_panels;
  plot_panels_ = config.plot_panels;
  image_panels_ = config.image_panels;
  publish_panels_ = config.publish_panels;
  service_panels_ = config.service_panels;
  teleop_panels_ = config.teleop_panels;
  map_panels_ = config.map_panels;
  channels_browser_ = config.channels_browser;
  raw_messages_ = config.raw_messages;
  table_panels_ = config.table_panels;
  channel_graph_panels_ = config.channel_graph_panels;
  tf_tree_panels_ = config.tf_tree_panels;
  session_variables_ = config.variables;
  plot_settings_visible_ = config.plot_settings_visible;
  window_x_ = config.window_x;
  window_y_ = config.window_y;
  window_width_ = config.window_width;
  window_height_ = config.window_height;
  view_manager_.loadFromSession(config);
  transformation_manager_.loadFromSession(config);
  view_controller_name_ = view_manager_.currentTypeName();
  tool_manager_.applyToolConfigs(config.tools);
  tool_manager_.setToolbarToolIds(config.toolbar_tools);
  if (!config.active_tool.empty() &&
      tool_manager_.toolById(config.active_tool) != nullptr) {
    tool_manager_.setActiveTool(config.active_tool);
  }
  syncDisplayContext();
  if (background_color_callback_) {
    background_color_callback_(background_color_);
  }
  if (frame_rate_callback_) {
    frame_rate_callback_(target_frame_rate_);
  }
  if (view_controller_callback_) {
    view_controller_callback_(view_controller_name_);
  }
  if (render_backend_callback_) {
    render_backend_callback_(render_backend_name_);
  }

  for (const auto& entry : config.displays) {
    auto display = DisplayFactory::create(entry);
    if (display == nullptr) {
      continue;
    }
    attachDisplay(std::move(display), entry.enabled);
  }
  // Bind session displays to the already-open primary 3D View (if any).
  if (active_viewport_bit_ != 0) {
    claimOrphanDisplaysForViewport(active_viewport_bit_);
  }

  redraw_callback_ = std::move(saved_redraw);
  syncDisplayContext();
  if (redraw_callback_) {
    redraw_callback_();
  }
}

void VisualizationManager::prepareDisplayTree(display::Display* display) {
  if (display == nullptr) {
    return;
  }
  if (auto* interactive_display =
          dynamic_cast<display::InteractiveMarkerDisplay*>(display)) {
    interactive_display->setRegistry(&interactive_marker_registry_);
  }
  display->setContext(&display_context_);
  if (auto* group = dynamic_cast<display::DisplayGroup*>(display)) {
    for (std::size_t i = 0; i < group->children().size(); ++i) {
      prepareDisplayTree(group->child(i));
    }
  }
}

void VisualizationManager::rebuildDisplayViews() {
  display_views_.clear();
  display_views_.reserve(displays_.size());
  for (auto& display : displays_) {
    if (display == nullptr) {
      continue;
    }
    // When a 3D View is active, Displays panel only lists that view's displays.
    if (active_viewport_bit_ != 0 &&
        !display->visibleInViewport(active_viewport_bit_)) {
      continue;
    }
    display_views_.push_back(display.get());
  }
}

uint32_t VisualizationManager::allocViewportBit() {
  return viewport_bit_allocator_.allocBit();
}

void VisualizationManager::freeViewportBit(uint32_t bit) {
  if (bit == 0) {
    return;
  }
  viewport_bit_allocator_.freeBits(bit);
  if (active_viewport_bit_ == bit) {
    active_viewport_bit_ = 0;
    display_context_.default_visibility_bit = 0x00000001u;
  }
}

void VisualizationManager::setActiveViewportBit(uint32_t bit) {
  active_viewport_bit_ = bit;
  if (bit != 0) {
    display_context_.default_visibility_bit = bit;
  }
  rebuildDisplayViews();
}

void VisualizationManager::claimOrphanDisplaysForViewport(uint32_t bit) {
  if (bit == 0) {
    return;
  }
  for (auto& display : displays_) {
    if (display == nullptr) {
      continue;
    }
    const uint32_t bits = display->visibilityBits();
    // Unassigned (0) or legacy "visible everywhere" (all bits) → primary view.
    if (bits == 0 || bits == 0xFFFFFFFFu) {
      display->assignVisibilityBits(bit);
    }
  }
  rebuildDisplayViews();
}

int VisualizationManager::seedViewportDisplaysFromPeer(uint32_t bit) {
  if (bit == 0) {
    return 0;
  }
  for (const auto& display : displays_) {
    if (display != nullptr && display->visibleInViewport(bit)) {
      return 0;  // Already has a tree — do not duplicate.
    }
  }

  // Prefer cloning from the densest peer viewport (session primary).
  std::map<uint32_t, int> counts;
  for (const auto& display : displays_) {
    if (display == nullptr) {
      continue;
    }
    const uint32_t bits = display->visibilityBits();
    if (bits == 0 || bits == 0xFFFFFFFFu || (bits & bit) != 0) {
      continue;
    }
    // Single-bit owners only (ignore multi-bit edge cases).
    if ((bits & (bits - 1)) == 0) {
      ++counts[bits];
    }
  }
  uint32_t source_bit = 0;
  int best = 0;
  for (const auto& entry : counts) {
    if (entry.second > best) {
      best = entry.second;
      source_bit = entry.first;
    }
  }
  if (source_bit == 0 || best == 0) {
    return 0;
  }

  std::vector<DisplayConfig> configs;
  configs.reserve(static_cast<std::size_t>(best));
  for (const auto& display : displays_) {
    if (display == nullptr || !display->visibleInViewport(source_bit)) {
      continue;
    }
    DisplayConfig config;
    config.type = display->typeId();
    config.name = display->name();
    config.channel = display->channel();
    config.enabled = display->enabled();
    config.properties = display->properties();
    configs.push_back(std::move(config));
  }
  if (configs.empty()) {
    return 0;
  }

  const uint32_t previous = active_viewport_bit_;
  setActiveViewportBit(bit);
  int added = 0;
  for (const auto& config : configs) {
    if (addDisplay(config)) {
      ++added;
    }
  }
  setActiveViewportBit(previous != 0 ? previous : bit);
  return added;
}

void VisualizationManager::detachDisplaysFromViewport(uint32_t bit) {
  if (bit == 0) {
    return;
  }
  for (std::size_t i = 0; i < displays_.size();) {
    auto& display = displays_[i];
    if (display == nullptr) {
      ++i;
      continue;
    }
    display->unsetVisibilityBits(bit);
    if (display->visibilityBits() == 0) {
      display->setEnabled(false);
      displays_.erase(displays_.begin() + static_cast<std::ptrdiff_t>(i));
      continue;
    }
    ++i;
  }
  rebuildDisplayViews();
}

bool VisualizationManager::isDisplayInSubtree(
    const display::Display* root, const display::Display* target) {
  if (root == nullptr || target == nullptr) {
    return false;
  }
  if (root == target) {
    return true;
  }
  if (const auto* group = dynamic_cast<const display::DisplayGroup*>(root)) {
    for (std::size_t i = 0; i < group->children().size(); ++i) {
      if (isDisplayInSubtree(group->child(i), target)) {
        return true;
      }
    }
  }
  return false;
}

std::unique_ptr<display::Display> VisualizationManager::takeDisplay(
    std::size_t index, int child_index) {
  if (index >= displays_.size()) {
    return nullptr;
  }
  if (child_index < 0) {
    auto display = std::move(displays_[index]);
    displays_.erase(displays_.begin() + static_cast<std::ptrdiff_t>(index));
    rebuildDisplayViews();
    return display;
  }
  auto* group = dynamic_cast<display::DisplayGroup*>(displays_[index].get());
  if (group == nullptr) {
    return nullptr;
  }
  return group->takeChild(static_cast<std::size_t>(child_index));
}

void VisualizationManager::attachDisplay(
    std::unique_ptr<display::Display> display, bool enabled) {
  // Factory construction enables the display before it has a context, so
  // onEnable returns without subscribing and a later setEnabled(true) is a
  // no-op. Turn it off, attach the context, then enable once.
  const bool created_enabled = display->enabled();
  if (created_enabled) {
    display->setEnabled(false);
  }
  prepareDisplayTree(display.get());
  display->setEnabled(enabled);
  display_views_.push_back(display.get());
  displays_.push_back(std::move(display));
}

bool VisualizationManager::loadSession(const std::string& path) {
  SessionConfig config;
  if (!SessionConfigIO::load(path, &config)) {
    return false;
  }
  applySession(config);
  return true;
}

bool VisualizationManager::saveSession(const std::string& path) const {
  return SessionConfigIO::save(path, currentSession());
}

namespace {

void FillDisplayConfig(const display::Display* display, DisplayConfig* entry) {
  if (display == nullptr || entry == nullptr) {
    return;
  }
  display->saveToConfig(entry);
}

display::Display* ResolveDisplay(display::Display* display, int child_index) {
  if (display == nullptr || child_index < 0) {
    return display;
  }
  auto* group = dynamic_cast<display::DisplayGroup*>(display);
  if (group == nullptr) {
    return nullptr;
  }
  return group->child(static_cast<std::size_t>(child_index));
}

}  // namespace

SessionConfig VisualizationManager::currentSession() const {
  SessionConfig config;
  config.fixed_frame = fixed_frame_;
  config.frame_rate = target_frame_rate_;
  config.time_sync_mode = static_cast<int>(time_sync_mode_);
  config.time_sync_source = time_sync_source_;
  config.background_color = background_color_;
  config.view_controller = view_controller_name_;
  config.render_backend = render_backend_name_;
  config.window_state_b64 = window_state_b64_;
  config.main_panel_state_b64 = main_panel_state_b64_;
  config.window_geometry_b64 = window_geometry_b64_;
  config.hide_left_dock = hide_left_dock_;
  config.hide_right_dock = hide_right_dock_;
  config.panel_layouts = panel_layouts_;
  config.visible_panels = visible_panels_;
  config.plot_panels = plot_panels_;
  config.image_panels = image_panels_;
  config.publish_panels = publish_panels_;
  config.service_panels = service_panels_;
  config.teleop_panels = teleop_panels_;
  config.map_panels = map_panels_;
  config.channels_browser = channels_browser_;
  config.raw_messages = raw_messages_;
  config.table_panels = table_panels_;
  config.channel_graph_panels = channel_graph_panels_;
  config.tf_tree_panels = tf_tree_panels_;
  config.variables = session_variables_;
  config.plot_settings_visible = plot_settings_visible_;
  config.window_x = window_x_;
  config.window_y = window_y_;
  config.window_width = window_width_;
  config.window_height = window_height_;
  view_manager_.saveToSession(&config);
  transformation_manager_.saveToSession(&config);
  config.toolbar_tools = tool_manager_.toolbarToolIds();
  config.active_tool = tool_manager_.activeToolId();
  config.tools = tool_manager_.currentToolConfigs();
  for (const auto* display : display_views_) {
    DisplayConfig entry;
    FillDisplayConfig(display, &entry);
    config.displays.push_back(std::move(entry));
  }
  return config;
}

display::Display* VisualizationManager::displayAt(std::size_t index,
                                                  int child_index) {
  if (index >= displays_.size()) {
    return nullptr;
  }
  return ResolveDisplay(displays_[index].get(), child_index);
}

const display::Display* VisualizationManager::displayAt(std::size_t index,
                                                        int child_index) const {
  if (index >= displays_.size()) {
    return nullptr;
  }
  return ResolveDisplay(displays_[index].get(), child_index);
}

void VisualizationManager::setDisplayEnabled(std::size_t index, bool enabled,
                                              int child_index) {
  display::Display* display = displayAt(index, child_index);
  if (display == nullptr) {
    return;
  }
  display->setEnabled(enabled);
  if (display_context_.request_redraw) {
    display_context_.request_redraw();
  }
}

void VisualizationManager::setDisplayChannel(std::size_t index,
                                             const std::string& channel,
                                             int child_index) {
  display::Display* display = displayAt(index, child_index);
  if (display == nullptr) {
    return;
  }
  display->setChannel(channel);
}

void VisualizationManager::setDisplayProperty(std::size_t index,
                                              const std::string& key,
                                              const std::string& value,
                                              int child_index) {
  display::Display* display = displayAt(index, child_index);
  if (display == nullptr) {
    return;
  }
  display->setPropertyValue(key, value);
}

void VisualizationManager::setSelectionChangedCallback(
    std::function<void(const std::vector<SelectionEntry>&)> callback) {
  selection_manager_.setChangedCallback(std::move(callback));
}

void VisualizationManager::setSelectionFocusCallback(
    std::function<void(const QVector3D& target)> callback) {
  selection_manager_.setFocusCallback(std::move(callback));
}

void VisualizationManager::update() {
  if (!initialized_ || updating_) {
    return;
  }
  updating_ = true;
  tf_listener_.poll();
  frame_manager_.setPause(time_paused_);
  frame_manager_.setSyncMode(
      static_cast<FrameManager::SyncMode>(static_cast<int>(time_sync_mode_)));
  frame_manager_.syncTime(simTimeSec());
  frame_manager_.update();
  pick_registry_.clear();
  handler_manager_.clear();
  // Drain / update every display once. Drawing is per-viewport so split 3D
  // Views do not inherit each other's overlay geometry.
  for (auto& display : displays_) {
    if (display != nullptr) {
      display->update();
    }
  }
  display_context_.incrementFrameCount();
  updating_ = false;
}

void VisualizationManager::drawViewport(uint32_t viewport_bit,
                                        rendering::SceneOverlay& overlay,
                                        rendering::OgreSceneHost* ogre_host,
                                        bool draw_tools) {
  if (!initialized_ || viewport_bit == 0) {
    return;
  }
  // Bind this pane's Ogre host for the draw pass. LaserScan / DepthCloud upload
  // only to context->ogre_scene_host; using the "active" host for every pane
  // made split views share the same cloud.
  rendering::OgreSceneHost* previous_host = display_context_.ogre_scene_host;
  display_context_.ogre_scene_host = ogre_host;

  if (ogre_host != nullptr) {
    // Drop geometry that belongs to other 3D Views (stale uploads). Walk groups
    // so child LaserScan names are cleared, not only the Group folder name.
    // Group membership uses the group's visibility bit (children often stay 0).
    //
    // Dual-pane sessions clone displays with the SAME name ("points PointCloud2").
    // Scrubbing a foreign clone by name must not wipe this pane's clone — that
    // caused depth / lidar clouds to flicker every frame across 3D Views.
    auto nameStillNeededHere = [this, viewport_bit](const std::string& name) {
      for (const auto& other : displays_) {
        if (other == nullptr || other->name() != name) {
          continue;
        }
        if (other->visibleInViewport(viewport_bit)) {
          return true;
        }
      }
      return false;
    };
    std::function<void(display::Display*, bool visible)> scrub;
    scrub = [&](display::Display* display, bool visible) {
      if (display == nullptr) {
        return;
      }
      if (auto* group = dynamic_cast<display::DisplayGroup*>(display)) {
        for (std::size_t i = 0; i < group->children().size(); ++i) {
          scrub(group->child(i), visible);
        }
      }
      if (!visible && !nameStillNeededHere(display->name())) {
        ogre_host->removeDisplaysWithPrefix(display->name());
      }
    };
    for (auto& display : displays_) {
      if (display == nullptr) {
        continue;
      }
      scrub(display.get(), display->visibleInViewport(viewport_bit));
    }
  }

  overlay.setPickRegistry(&pick_registry_);
  overlay.setHandlerManager(&handler_manager_);
  overlay.clear();
  for (auto& display : displays_) {
    if (display == nullptr || !display->visibleInViewport(viewport_bit)) {
      continue;
    }
    display->draw(overlay);
  }
  overlay.setPickSource(nullptr, nullptr);
  if (draw_tools) {
    tool_manager_.onDraw(overlay);
  }

  display_context_.ogre_scene_host = previous_host;
}

void VisualizationManager::refreshChannelList() {
  if (channel_manager_ == nullptr) {
    cached_channels_.clear();
    return;
  }
  cached_channels_ = channel_manager_->listWritableChannels();
}

void VisualizationManager::setFixedFrame(const std::string& frame) {
  fixed_frame_ = frame;
  frame_manager_.setFixedFrame(frame);
  syncDisplayContext();
}

void VisualizationManager::setBackgroundColor(const std::string& color) {
  background_color_ = color;
  if (background_color_callback_) {
    background_color_callback_(background_color_);
  }
}

void VisualizationManager::setBackgroundColorCallback(
    std::function<void(const std::string&)> callback) {
  background_color_callback_ = std::move(callback);
  if (background_color_callback_) {
    background_color_callback_(background_color_);
  }
}

void VisualizationManager::setTargetFrameRate(int rate) {
  const int clamped = std::clamp(rate, 1, 120);
  if (target_frame_rate_ == clamped) {
    return;
  }
  target_frame_rate_ = clamped;
  if (frame_rate_callback_) {
    frame_rate_callback_(target_frame_rate_);
  }
}

void VisualizationManager::setFrameRateCallback(
    std::function<void(int)> callback) {
  frame_rate_callback_ = std::move(callback);
  if (frame_rate_callback_) {
    frame_rate_callback_(target_frame_rate_);
  }
}

std::vector<std::string> VisualizationManager::channelNames() const {
  std::vector<std::string> names;
  names.reserve(cached_channels_.size());
  for (const auto& channel : cached_channels_) {
    names.push_back(channel.channel_name);
  }
  return names;
}

int VisualizationManager::ensureDisplaysForRecordChannels(
    const std::unordered_map<std::string, std::string>& channel_types,
    int max_new) {
  if (channel_types.empty() || max_new <= 0) {
    return 0;
  }

  std::unordered_set<std::string> covered_channels;
  bool has_tf_display = false;
  bool has_robot_model = false;
  std::function<void(display::Display*)> walk;
  walk = [&](display::Display* display) {
    if (display == nullptr) {
      return;
    }
    if (display->typeId() == "TF") {
      has_tf_display = true;
    }
    if (display->typeId() == "RobotModel") {
      has_robot_model = true;
    }
    if (!display->channel().empty()) {
      covered_channels.insert(display->channel());
    }
    if (auto* group = dynamic_cast<display::DisplayGroup*>(display)) {
      for (const auto& child : group->children()) {
        walk(child.get());
      }
    }
  };
  for (const auto& display : displays_) {
    walk(display.get());
  }

  auto prefer_type = [](const std::vector<std::string>& types) -> std::string {
    // Image / TwistStamped are hidden from Displays (ImageDock / Teleop).
    static const char* kPreferred[] = {"PointCloud2", "Imu",         "Odometry",
                                       "LaserScan",   "Path",        "Marker",
                                       "MarkerArray", "RobotModel",  "Camera",
                                       "CameraInfo",  "Pose",        "PoseStamped",
                                       "TF"};
    for (const char* preferred : kPreferred) {
      if (std::find(types.begin(), types.end(), preferred) != types.end()) {
        return preferred;
      }
    }
    for (const std::string& type : types) {
      if (!DisplayCatalog::isHiddenFromDisplaysPanel(type)) {
        return type;
      }
    }
    return {};
  };

  int added = 0;
  // TF first: /tf + /tf_static feed the shared listener / TF display.
  if (!has_tf_display) {
    for (const auto& entry : channel_types) {
      const auto matches =
          DisplayCatalog::typesForChannel(entry.first, entry.second);
      if (std::find(matches.begin(), matches.end(), "TF") == matches.end() &&
          entry.first != "/tf" && entry.first != "/tf_static" &&
          entry.first.find("tf2_msgs") == std::string::npos &&
          entry.second.find("TFMessage") == std::string::npos) {
        continue;
      }
      DisplayConfig config;
      config.type = "TF";
      config.name = "TF";
      config.channel = "";
      config.enabled = true;
      if (addDisplay(config)) {
        ++added;
        has_tf_display = true;
      }
      break;
    }
  }

  auto channel_base = [](const std::string& channel) {
    const auto slash = channel.find_last_of('/');
    return slash == std::string::npos ? channel : channel.substr(slash + 1);
  };
  auto cover_robot_channels = [&]() {
    for (const auto& entry : channel_types) {
      const std::string base = channel_base(entry.first);
      if (base == "robot_description" || base == "joint_states") {
        covered_channels.insert(entry.first);
      }
    }
  };
  if (has_robot_model) {
    // The startup RobotModel is created disabled. Turn it on once the
    // description or joint topic exists, so the model shows without a manual add.
    std::function<void(display::Display*)> enable_robots;
    enable_robots = [&](display::Display* display) {
      if (display == nullptr) {
        return;
      }
      if (display->typeId() == "RobotModel" && !display->enabled()) {
        display->setEnabled(true);
      }
      if (auto* group = dynamic_cast<display::DisplayGroup*>(display)) {
        for (const auto& child : group->children()) {
          enable_robots(child.get());
        }
      }
    };
    for (const auto& display : displays_) {
      enable_robots(display.get());
    }
    cover_robot_channels();
  } else {
    std::string description;
    std::string joints;
    for (const auto& entry : channel_types) {
      const std::string base = channel_base(entry.first);
      if (base == "robot_description" &&
          (description.empty() || entry.first == "/robot_description")) {
        description = entry.first;
      } else if (base == "joint_states" &&
                 (joints.empty() || entry.first == "/joint_states")) {
        joints = entry.first;
      }
    }
    if (!description.empty() || !joints.empty()) {
      if (joints.empty()) {
        joints = "/joint_states";
      }
      if (description.empty()) {
        description = "/robot_description";
      }
      DisplayConfig config;
      config.type = "RobotModel";
      config.name = "RobotModel";
      config.channel = joints;
      config.enabled = true;
      config.properties["description_source"] = "Topic";
      config.properties["description_channel"] = description;
      config.properties["urdf_path"] = "";
      config.properties["root_link"] = "";
      config.properties["show_axes"] = "false";
      if (added < max_new && addDisplay(config)) {
        ++added;
        has_robot_model = true;
      }
      cover_robot_channels();
    }
  }

  for (const auto& entry : channel_types) {
    if (added >= max_new) {
      break;
    }
    const std::string& channel = entry.first;
    const std::string& message_type = entry.second;
    if (channel.empty() || channel.rfind("/autolink/", 0) == 0) {
      continue;
    }
    if (covered_channels.count(channel) > 0) {
      continue;
    }
    const std::string base = channel_base(channel);
    // One RobotModel owns both topics. A second display would lose CreateReader.
    if (base == "robot_description" || base == "joint_states") {
      continue;
    }
    // Teleop command topic — not a Displays panel entry.
    if (base == "cmd_vel") {
      continue;
    }
    if (DisplayCatalog::isStaticTfChannel(channel) || channel == "/tf" ||
        channel == "/tf_static" || message_type.find("TFMessage") != std::string::npos) {
      covered_channels.insert(channel);
      continue;
    }
    const auto matches =
        DisplayCatalog::typesForChannel(channel, message_type);
    const std::string type = prefer_type(matches);
    if (type.empty() || type == "TF") {
      continue;
    }
    DisplayConfig config;
    config.type = type;
    config.name = channel;
    config.channel = channel;
    config.enabled = true;
    if (addDisplay(config)) {
      covered_channels.insert(channel);
      ++added;
    }
  }
  return added;
}

bool VisualizationManager::addDisplay(const DisplayConfig& config) {
  auto display = DisplayFactory::create(config);
  if (display == nullptr) {
    return false;
  }
  attachDisplay(std::move(display), config.enabled);
  return true;
}

bool VisualizationManager::removeDisplay(std::size_t index, int child_index) {
  if (index >= displays_.size()) {
    return false;
  }
  if (child_index < 0) {
    displays_[index]->setEnabled(false);
    displays_.erase(displays_.begin() + static_cast<std::ptrdiff_t>(index));
    rebuildDisplayViews();
    return true;
  }
  auto* group = dynamic_cast<display::DisplayGroup*>(displays_[index].get());
  if (group == nullptr ||
      static_cast<std::size_t>(child_index) >= group->children().size()) {
    return false;
  }
  if (display::Display* child = group->child(static_cast<std::size_t>(child_index))) {
    child->setEnabled(false);
  }
  group->takeChild(static_cast<std::size_t>(child_index));
  return true;
}

bool VisualizationManager::moveDisplay(std::size_t from_index, int from_child_index,
                                       std::size_t to_group_index,
                                       int to_child_index) {
  if (from_index >= displays_.size() || to_group_index >= displays_.size()) {
    return false;
  }

  if (from_child_index >= 0 && from_index == to_group_index) {
    auto* group =
        dynamic_cast<display::DisplayGroup*>(displays_[from_index].get());
    if (group == nullptr) {
      return false;
    }
    const std::size_t from = static_cast<std::size_t>(from_child_index);
    std::size_t to = to_child_index < 0
                         ? group->children().size()
                         : static_cast<std::size_t>(to_child_index);
    if (to > group->children().size()) {
      to = group->children().size();
    }
    if (from == to || (from + 1 == to && to <= group->children().size())) {
      return false;
    }
    std::unique_ptr<display::Display> moved = group->takeChild(from);
    if (moved == nullptr) {
      return false;
    }
    if (to > from) {
      --to;
    }
    group->insertChild(to, std::move(moved));
    return true;
  }

  display::Display* source = displayAt(from_index, from_child_index);
  display::Display* target_group_display = displays_[to_group_index].get();
  auto* target_group =
      dynamic_cast<display::DisplayGroup*>(target_group_display);
  if (source == nullptr || target_group == nullptr) {
    return false;
  }
  if (source == target_group_display) {
    return false;
  }
  if (isDisplayInSubtree(source, target_group_display)) {
    return false;
  }

  std::unique_ptr<display::Display> moved =
      takeDisplay(from_index, from_child_index);
  if (moved == nullptr) {
    return false;
  }

  if (to_group_index > from_index && from_child_index < 0) {
    --to_group_index;
  }

  target_group = dynamic_cast<display::DisplayGroup*>(displays_[to_group_index].get());
  if (target_group == nullptr) {
    displays_.push_back(std::move(moved));
    rebuildDisplayViews();
    return false;
  }

  prepareDisplayTree(moved.get());
  if (to_child_index < 0 ||
      static_cast<std::size_t>(to_child_index) >= target_group->children().size()) {
    target_group->addChild(std::move(moved));
  } else {
    target_group->insertChild(static_cast<std::size_t>(to_child_index),
                              std::move(moved));
  }
  return true;
}

bool VisualizationManager::moveDisplayToRoot(std::size_t from_index,
                                           int from_child_index,
                                           std::size_t root_insert_index) {
  if (from_index >= displays_.size()) {
    return false;
  }
  if (root_insert_index > displays_.size()) {
    root_insert_index = displays_.size();
  }

  std::unique_ptr<display::Display> moved =
      takeDisplay(from_index, from_child_index);
  if (moved == nullptr) {
    return false;
  }

  if (from_child_index < 0 && root_insert_index > from_index) {
    --root_insert_index;
  }
  if (root_insert_index > displays_.size()) {
    root_insert_index = displays_.size();
  }

  prepareDisplayTree(moved.get());
  displays_.insert(displays_.begin() + static_cast<std::ptrdiff_t>(root_insert_index),
                   std::move(moved));
  rebuildDisplayViews();
  return true;
}

bool VisualizationManager::reorderRootDisplay(std::size_t from_index,
                                              std::size_t to_index) {
  if (from_index >= displays_.size() || to_index >= displays_.size() ||
      from_index == to_index) {
    return false;
  }
  auto display = std::move(displays_[from_index]);
  displays_.erase(displays_.begin() + static_cast<std::ptrdiff_t>(from_index));
  if (to_index > from_index) {
    --to_index;
  }
  displays_.insert(displays_.begin() + static_cast<std::ptrdiff_t>(to_index),
                   std::move(display));
  rebuildDisplayViews();
  return true;
}

bool VisualizationManager::duplicateDisplay(std::size_t index) {
  if (index >= display_views_.size()) {
    return false;
  }
  const auto* source = display_views_[index];
  DisplayConfig config;
  config.type = source->typeId();
  config.name = source->name() + " copy";
  config.channel = source->channel();
  config.enabled = source->enabled();
  config.properties = source->properties();
  return addDisplay(config);
}

void VisualizationManager::setDisplayName(std::size_t index,
                                          const std::string& name) {
  if (index >= displays_.size()) {
    return;
  }
  displays_[index]->setDisplayName(name);
}

void VisualizationManager::setWindowLayout(const std::string& state_b64,
                                           const std::string& geometry_b64) {
  window_state_b64_ = state_b64;
  window_geometry_b64_ = geometry_b64;
}

void VisualizationManager::setMainPanelLayout(const std::string& state_b64) {
  main_panel_state_b64_ = state_b64;
}

void VisualizationManager::setDockHideState(bool hide_left, bool hide_right) {
  hide_left_dock_ = hide_left;
  hide_right_dock_ = hide_right;
}

void VisualizationManager::setPanelLayouts(
    const std::vector<PanelLayoutConfig>& layouts) {
  panel_layouts_ = layouts;
}

void VisualizationManager::setVisiblePanels(
    const std::vector<std::string>& panels) {
  visible_panels_ = panels;
}

void VisualizationManager::setPlotPanels(
    const std::vector<PlotPanelPersistConfig>& panels) {
  plot_panels_ = panels;
}

void VisualizationManager::setImagePanels(
    const std::vector<ImagePanelPersistConfig>& panels) {
  image_panels_ = panels;
}

void VisualizationManager::setPublishPanels(
    const std::vector<PublishPanelPersistConfig>& panels) {
  publish_panels_ = panels;
}

void VisualizationManager::setServicePanels(
    const std::vector<ServicePanelPersistConfig>& panels) {
  service_panels_ = panels;
}

void VisualizationManager::setTeleopPanels(
    const std::vector<TeleopPanelPersistConfig>& panels) {
  teleop_panels_ = panels;
}

void VisualizationManager::setMapPanels(
    const std::vector<MapPanelPersistConfig>& panels) {
  map_panels_ = panels;
}

void VisualizationManager::setChannelsBrowser(
    const ChannelsBrowserPersistConfig& config) {
  channels_browser_ = config;
}

void VisualizationManager::setRawMessages(const RawMessagesPersistConfig& config) {
  raw_messages_ = config;
}

void VisualizationManager::setTablePanels(
    const std::vector<TablePanelPersistConfig>& panels) {
  table_panels_ = panels;
}

void VisualizationManager::setChannelGraphPanels(
    const std::vector<ChannelGraphPanelPersistConfig>& panels) {
  channel_graph_panels_ = panels;
}

void VisualizationManager::setTfTreePanels(
    const std::vector<TfTreePanelPersistConfig>& panels) {
  tf_tree_panels_ = panels;
}

void VisualizationManager::setPlotSettingsVisible(bool visible) {
  plot_settings_visible_ = visible;
}

void VisualizationManager::setWindowFrame(int x, int y, int width, int height) {
  window_x_ = x;
  window_y_ = y;
  window_width_ = width;
  window_height_ = height;
}

void VisualizationManager::setViewControllerName(const std::string& name) {
  view_controller_name_ = name;
  if (view_controller_callback_) {
    view_controller_callback_(view_controller_name_);
  }
}

void VisualizationManager::setViewControllerCallback(
    std::function<void(const std::string&)> callback) {
  view_controller_callback_ = std::move(callback);
  if (view_controller_callback_) {
    view_controller_callback_(view_controller_name_);
  }
}

void VisualizationManager::setRenderBackendName(const std::string& name) {
  // Viewport is Ogre-only; ignore legacy "OpenGL" session values.
  render_backend_name_ = (name == "Ogre") ? name : "Ogre";
  if (render_backend_callback_) {
    render_backend_callback_(render_backend_name_);
  }
}

void VisualizationManager::setRenderBackendCallback(
    std::function<void(const std::string&)> callback) {
  render_backend_callback_ = std::move(callback);
  if (render_backend_callback_) {
    render_backend_callback_(render_backend_name_);
  }
}

std::shared_ptr<::autolink::Node> VisualizationManager::autolinkNode() const {
  return autolink_.node();
}

void VisualizationManager::setImageUpdateCallback(
    std::function<void(const QString&, const QImage&)> callback) {
  image_update_callback_ = std::move(callback);
}

double VisualizationManager::wallClockSec() const {
  const auto now = std::chrono::system_clock::now();
  return std::chrono::duration<double>(now.time_since_epoch()).count();
}

double VisualizationManager::wallClockElapsedSec() const {
  const auto now = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(now - wall_start_).count();
}

double VisualizationManager::simTimeSec() const {
  if (time_paused_) {
    return paused_sim_sec_;
  }
  if (!playback_.currentFile().empty() &&
      (playback_.isPlaying() || playback_.isPaused())) {
    return playback_.currentTimeSec();
  }
  return static_cast<double>(automsgs::msgs::builtin_interfaces::TimeToNanoseconds(automsgs::msgs::builtin_interfaces::TimeNow())) * 1e-9;
}

double VisualizationManager::simTimeElapsedSec() const {
  return simTimeSec() - sim_origin_sec_;
}

void VisualizationManager::setTimePaused(bool paused) {
  if (paused && !time_paused_) {
    if (!playback_.currentFile().empty() &&
        (playback_.isPlaying() || playback_.isPaused())) {
      paused_sim_sec_ = playback_.currentTimeSec();
    } else {
      paused_sim_sec_ = static_cast<double>(automsgs::msgs::builtin_interfaces::TimeToNanoseconds(automsgs::msgs::builtin_interfaces::TimeNow())) * 1e-9;
    }
  }
  time_paused_ = paused;
}

void VisualizationManager::setTimeSyncMode(TimeSyncMode mode) {
  time_sync_mode_ = mode;
}

void VisualizationManager::setTimeSyncSource(const std::string& source) {
  time_sync_source_ = source;
}

void VisualizationManager::resetTime() {
  // Drop every pending channel payload (Displays + Image/Plot/Raw panels).
  integration::MessageQueue::clearAllPending();
  integration::ChannelStatsRegistry::instance().reset();

  wall_start_ = std::chrono::steady_clock::now();
  time_paused_ = false;
  paused_sim_sec_ = 0.0;
  sim_origin_sec_ = simTimeSec();
  tf_listener_.clearPending();
  if (tf_buffer_ != nullptr) {
    tf_buffer_->clear();
  }
  for (auto& display : displays_) {
    if (display != nullptr) {
      display->reset();
    }
  }
  scene_overlay_.clear();
  if (redraw_callback_) {
    redraw_callback_();
  }
}

}  // namespace common
}  // namespace autoviz
