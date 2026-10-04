/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file display.hpp
 * @brief Base class for Autoviz 3D visualization plugins (RViz @c Display
 *        analogue).
 *
 * Every plugin shown in the Displays panel derives from @ref Display. The base
 * owns enable state, string-keyed properties, status text, visibility bits, and
 * the fixed update/draw pipeline invoked by
 * @ref common::VisualizationManager.
 *
 * ## Lifecycle
 *
 * @code
 * setContext() → setEnabled(true) → onEnable()
 *   → update() { onUpdate() } → draw() { onDraw() }
 * setEnabled(false) → onDisable()
 * reset()  // Time panel / user Reset
 * @endcode
 *
 * ## Persistence
 *
 * @ref load / @ref save mirror @c rviz_common::Display Config trees;
 * @ref loadFromConfig / @ref saveToConfig map the session
 * @ref common::DisplayConfig protobuf-friendly struct.
 *
 * @see DisplayGroup
 * @see ChannelDisplay
 * @see common::DisplayContext
 * @see common::DisplayPropertySpec
 */

#pragma once

#include <cstdint>
#include <map>
#include <string>

#include <QImage>

#include "autoviz/transform/buffer.hpp"
#include "autoviz/common/display_context.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/common/config.hpp"
#include "autoviz/common/session_config.hpp"
#include "autoviz/common/display_status.hpp"
#include "autoviz/integration/autolink_context.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

namespace autoviz {
namespace display {

/**
 * @class Display
 * @brief Abstract visualization plugin: properties, status, enable, update,
 *        and scene overlay drawing.
 *
 * Subclasses must implement @ref onDraw. Channel-backed displays typically
 * also override @ref onEnable / @ref onDisable / @ref onUpdate (or inherit
 * @ref ChannelDisplay).
 *
 * @note The display does not own @c context_; VisualizationManager keeps the
 *       DisplayContext lifetime and calls @ref setContext before enable.
 *
 * @see DisplayGroup
 * @see FailedDisplay
 * @see ChannelDisplay
 */
class Display {
 public:
  /**
   * @brief Virtual destructor for polymorphic ownership in
   *        @ref DisplayGroup and the display registry.
   */
  virtual ~Display() = default;

  /**
   * @brief Stable type identifier used by the display catalog / factory.
   *
   * @return Plugin type string (e.g. @c "Grid", @c "Marker"). Default
   *         @c "Display".
   */
  virtual std::string typeId() const { return "Display"; }

  /**
   * @brief User-visible name in the Displays tree.
   *
   * @return @c display_name_ when set; otherwise @ref typeId().
   * @see setDisplayName()
   */
  virtual std::string name() const {
    return display_name_.empty() ? typeId() : display_name_;
  }

  /**
   * @brief Primary Autolink channel (topic) for channel-backed displays.
   *
   * @return Channel name, or empty string when the display has no topic.
   * @see setChannel()
   */
  virtual std::string channel() const { return ""; }

  /**
   * @brief Changes the primary channel; channel displays re-subscribe when
   *        enabled.
   *
   * @param channel New channel name (empty clears subscription).
   */
  virtual void setChannel(const std::string& /*channel*/) {}

  /**
   * @brief Whether this display is currently enabled (subscribed / drawn).
   *
   * @return @c true when enabled.
   * @see setEnabled()
   */
  virtual bool enabled() const { return enabled_; }

  /**
   * @brief Aggregated status level and message for the Displays tree icon.
   *
   * @return Copy of the current @ref DisplayStatus.
   * @see setStatusOk()
   * @see setStatusWarn()
   * @see setStatusError()
   */
  DisplayStatus status() const {
    return {status_level_, status_message_};
  }

  /**
   * @brief Property schema for the Displays property editor.
   *
   * @return Specs describing keys, labels, defaults, and editor kinds.
   *         Base returns an empty list.
   * @see common::DisplayPropertySpec
   */
  virtual std::vector<common::DisplayPropertySpec> propertySpecs() const {
    return {};
  }

  /**
   * @brief Replaces the entire property map and notifies subclasses.
   *
   * Typically calls @ref onPropertyChanged for each changed key.
   *
   * @param properties New key → string value map.
   * @see properties()
   * @see setPropertyValue()
   */
  void setProperties(const common::DisplayPropertyMap& properties);

  /**
   * @brief Returns the live property key → value map.
   *
   * @return Const reference to @c properties_.
   */
  const common::DisplayPropertyMap& properties() const { return properties_; }

  /**
   * @brief Looks up a single property with a fallback default.
   *
   * @param key Property key (e.g. @c "alpha").
   * @param default_value Value returned when @p key is absent.
   * @return Stored string value or @p default_value.
   */
  std::string propertyValue(const std::string& key,
                            const std::string& default_value) const;

  /**
   * @brief Sets one property and invokes @ref onPropertyChanged.
   *
   * @param key Property key.
   * @param value String encoding (floats, @c "R;G;B" colors, enums, …).
   */
  void setPropertyValue(const std::string& key, const std::string& value);

  /**
   * @brief Loads state from an RViz-style Config tree.
   *
   * Default reads Name, Enabled, and Topic/channel plus known property keys.
   *
   * @param config Config node for this display.
   * @see save()
   */
  virtual void load(const common::Config& config);

  /**
   * @brief Writes state into an RViz-style Config tree.
   *
   * @param config Mutable config node (value semantics / shared underlying
   *        map depending on @ref common::Config).
   * @see load()
   */
  virtual void save(common::Config config) const;

  /**
   * @brief Loads from session @ref common::DisplayConfig.
   *
   * @param config Session display entry (type, name, channel, properties).
   * @see saveToConfig()
   */
  void loadFromConfig(const common::DisplayConfig& config);

  /**
   * @brief Serializes into session @ref common::DisplayConfig.
   *
   * @param config Output pointer; must be non-null.
   * @see loadFromConfig()
   */
  virtual void saveToConfig(common::DisplayConfig* config) const;

  /**
   * @brief Sets the user-visible display name.
   *
   * @param name Tree label; empty falls back to @ref typeId() in @ref name().
   */
  void setDisplayName(const std::string& name) { display_name_ = name; }

  /**
   * @brief Attaches the shared runtime context (TF, Autolink, redraw).
   *
   * @param context Non-owning; owned by VisualizationManager.
   */
  void setContext(common::DisplayContext* context) { context_ = context; }

  /**
   * @brief Enables or disables the display, calling @ref onEnable /
   *        @ref onDisable on transitions.
   *
   * @param enabled Desired enable state.
   */
  void setEnabled(bool enabled);

  /**
   * @brief Drops cached messages / TF snapshots (RViz @c Display::reset).
   *
   * Invoked from the Time panel Reset action. Base clears status; subclasses
   * clear geometry and queues.
   */
  virtual void reset();

  /**
   * @brief Per-frame update entry: no-op when disabled; else @ref onUpdate.
   */
  void update();

  /**
   * @brief Per-frame draw entry: no-op when disabled; else @ref onDraw.
   *
   * @param scene Overlay collector for lines, meshes, images, etc.
   */
  void draw(rendering::SceneOverlay& scene);

  /**
   * @brief ORs visibility bits used by multi-viewport masking.
   *
   * @param bits Bitmask to set on @c visibility_bits_.
   * @see unsetVisibilityBits()
   * @see visibilityBits()
   */
  void setVisibilityBits(uint32_t bits);

  /**
   * @brief Clears visibility bits.
   *
   * @param bits Bitmask to clear from @c visibility_bits_.
   */
  void unsetVisibilityBits(uint32_t bits);

  /**
   * @brief Current visibility bitmask (default all bits set).
   *
   * @return @c visibility_bits_.
   */
  uint32_t visibilityBits() const { return visibility_bits_; }

 protected:
  /**
   * @brief Called when transitioning to enabled; subscribe / allocate here.
   */
  virtual void onEnable() {}

  /**
   * @brief Called when transitioning to disabled; unsubscribe / free here.
   */
  virtual void onDisable() {}

  /**
   * @brief Called each frame while enabled (drain queues, update status).
   */
  virtual void onUpdate() {}

  /**
   * @brief Called after a property value changes.
   *
   * @param key Changed property key.
   */
  virtual void onPropertyChanged(const std::string& /*key*/) {}

  /**
   * @brief Appends geometry / overlays for this frame.
   *
   * @param scene Scene overlay destination.
   */
  virtual void onDraw(rendering::SceneOverlay& scene) = 0;

  /**
   * @brief Clears status to empty OK (no message).
   */
  void resetStatus();

  /**
   * @brief Sets status to OK with an empty message.
   */
  void setStatusOk();

  /**
   * @brief Sets status to OK with a user-visible message.
   *
   * @param message Status text shown in the Displays tree.
   */
  void setStatusOk(const std::string& message);

  /**
   * @brief Sets status to warning.
   *
   * @param message Warning text (e.g. @c "No messages received").
   */
  void setStatusWarn(const std::string& message);

  /**
   * @brief Sets status to error.
   *
   * @param message Error text (e.g. @c "Failed to subscribe channel").
   */
  void setStatusError(const std::string& message);

  /**
   * @brief Non-owning runtime context (TF buffer, Autolink, redraw callback).
   */
  common::DisplayContext* context_ = nullptr;

 private:
  /** User-visible name; empty → @ref typeId(). */
  std::string display_name_;

  /** Key → string property values edited in the Displays panel. */
  common::DisplayPropertyMap properties_;

  /** Whether @ref update / @ref draw invoke subclass hooks. */
  bool enabled_ = true;

  /** Reserved / legacy subscription flag (kept for RViz parity). */
  bool subscribed_ = false;

  /** Multi-viewport visibility mask; default all bits set. */
  uint32_t visibility_bits_ = 0xFFFFFFFFu;

  /** Current status severity for the Displays tree. */
  DisplayStatusLevel status_level_ = DisplayStatusLevel::kOk;

  /** Current status message text. */
  std::string status_message_;
};

}  // namespace display
}  // namespace autoviz
