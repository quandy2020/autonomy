/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/visualization_manager.hpp"

#include "autolink/time/time.hpp"
#include "autoviz/common/display_factory.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/common/display_context.hpp"
#include "autolink/service_discovery/topology_manager.hpp"
#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/display/display.hpp"
#include "autoviz/display/display_group.hpp"
#include "autoviz/display/interactive_marker_display.hpp"
#include "autoviz/display/interactive_marker_registry.hpp"

#include <algorithm>
#include <functional>

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
  display_context_.tf_buffer = tf_buffer_;
  display_context_.frame_manager = &frame_manager_;
  display_context_.pick_registry = &pick_registry_;
  display_context_.handler_manager = &handler_manager_;
  display_context_.selection_manager = &selection_manager_;
  display_context_.tool_manager = &tool_manager_;
  display_context_.view_manager = &view_manager_;
  playback_.setNode(autolink_.node());

  auto topology = ::autolink::service_discovery::TopologyManager::Instance();
  channel_manager_ = std::make_unique<integration::ChannelManager>(
      topology->channel_manager());

  applySession(SessionConfigIO::defaultConfig());
  refreshChannelList();
  wall_start_ = std::chrono::steady_clock::now();
  sim_origin_sec_ = simTimeSec();
  initialized_ = true;
  return true;
}

void VisualizationManager::shutdown() {
  if (!initialized_) {
    return;
  }
  for (auto& display : displays_) {
    display->setEnabled(false);
  }
  displays_.clear();
  display_views_.clear();
  interactive_marker_registry_ = display::InteractiveMarkerRegistry{};
  channel_manager_.reset();
  integration::ChannelReaderRegistry::instance().setNode(nullptr);
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
  for (auto& display : displays_) {
    display->setEnabled(false);
  }
  displays_.clear();
  display_views_.clear();
  interactive_marker_registry_ = display::InteractiveMarkerRegistry{};

  fixed_frame_ = config.fixed_frame;
  show_grid_ = config.show_grid;
  target_frame_rate_ = std::clamp(config.frame_rate, 1, 120);
  background_color_ = config.background_color;
  view_controller_name_ = config.view_controller;
  render_backend_name_ = config.render_backend;
  window_state_b64_ = config.window_state_b64;
  window_geometry_b64_ = config.window_geometry_b64;
  hide_left_dock_ = config.hide_left_dock;
  hide_right_dock_ = config.hide_right_dock;
  panel_layouts_ = config.panel_layouts;
  visible_panels_ = config.visible_panels;
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
  if (grid_visibility_callback_) {
    grid_visibility_callback_(show_grid_);
  }
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
    display_views_.push_back(display.get());
  }
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
  config.show_grid = show_grid_;
  config.frame_rate = target_frame_rate_;
  config.background_color = background_color_;
  config.view_controller = view_controller_name_;
  config.render_backend = render_backend_name_;
  config.window_state_b64 = window_state_b64_;
  config.window_geometry_b64 = window_geometry_b64_;
  config.hide_left_dock = hide_left_dock_;
  config.hide_right_dock = hide_right_dock_;
  config.panel_layouts = panel_layouts_;
  config.visible_panels = visible_panels_;
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
  frame_manager_.setPause(time_paused_);
  frame_manager_.setSyncMode(
      static_cast<FrameManager::SyncMode>(static_cast<int>(time_sync_mode_)));
  frame_manager_.syncTime(simTimeSec());
  frame_manager_.update();
  pick_registry_.clear();
  handler_manager_.clear();
  scene_overlay_.setPickRegistry(&pick_registry_);
  scene_overlay_.setHandlerManager(&handler_manager_);
  for (auto& display : displays_) {
    display->update();
  }
  syncReferenceGridFromDisplays();
  scene_overlay_.clear();
  for (auto& display : displays_) {
    display->draw(scene_overlay_);
  }
  scene_overlay_.setPickSource(nullptr, nullptr);
  tool_manager_.onDraw(scene_overlay_);
  display_context_.incrementFrameCount();
  updating_ = false;
}

void VisualizationManager::syncReferenceGridFromDisplays() {
  bool found = false;
  std::function<bool(const display::Display*)> visit =
      [&](const display::Display* display) -> bool {
    if (display == nullptr) {
      return false;
    }
    if (display->enabled() && display->typeId() == "Grid") {
      rendering::ReferenceGridSettings settings;
      settings.half_cell_count = static_cast<int>(common::ParseFloatProperty(
          display->propertyValue("cell_count", "20"), 20.f));
      settings.cell_length = common::ParseFloatProperty(
          display->propertyValue("cell_size", "1.0"), 1.f);
      *settings.mutable_color() = common::ParseColorProperty(
          display->propertyValue("color", "80;80;80"), QColor(80, 80, 80));
      settings.alpha = common::ParseFloatProperty(
          display->propertyValue("alpha", "1.0"), 1.f);
      settings.show_axes = true;
      reference_grid_settings_ = settings;
      return true;
    }
    if (const auto* group =
            dynamic_cast<const display::DisplayGroup*>(display)) {
      for (std::size_t i = 0; i < group->children().size(); ++i) {
        if (visit(group->child(i))) {
          return true;
        }
      }
    }
    return false;
  };

  for (const auto* display : display_views_) {
    if (visit(display)) {
      found = true;
      break;
    }
  }
  (void)found;
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

void VisualizationManager::setShowGrid(bool show) {
  show_grid_ = show;
  if (grid_visibility_callback_) {
    grid_visibility_callback_(show_grid_);
  }
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

void VisualizationManager::setGridVisibilityCallback(
    std::function<void(bool)> callback) {
  grid_visibility_callback_ = std::move(callback);
  if (grid_visibility_callback_) {
    grid_visibility_callback_(show_grid_);
  }
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
  render_backend_name_ = name;
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
  return autolink::automsgs::msgs::builtin_interfaces::TimeNow().ToSecond();
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
      paused_sim_sec_ = autolink::automsgs::msgs::builtin_interfaces::TimeNow().ToSecond();
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
  wall_start_ = std::chrono::steady_clock::now();
  sim_origin_sec_ = autolink::automsgs::msgs::builtin_interfaces::TimeNow().ToSecond();
  time_paused_ = false;
  paused_sim_sec_ = 0.0;
}

}  // namespace common
}  // namespace autoviz
