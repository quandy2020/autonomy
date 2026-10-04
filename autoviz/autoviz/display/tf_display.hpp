/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file tf_display.hpp
 * @brief Channel display for the TF tree (@c tf2_msgs/TFMessage + buffer).
 *
 * Visualizes frames as axes, parent→child arrows, and optional name labels,
 * with whitelist/blacklist filters and RViz2-style frame-timeout aging.
 *
 * Listens both to the configured TF channel and to
 * @c transform::tf2::VoidSignal transform-changed notifications so the tree
 * stays in sync with the shared TF buffer.
 *
 * ## Properties
 *
 * - @c show_names / @c show_axes / @c show_arrows
 * - @c marker_scale / @c update_interval / @c frame_timeout
 * - @c filter_whitelist / @c filter_blacklist (regex)
 *
 * @see TfFrameSnapshot
 * @see tf_display_utils.hpp
 * @see ChannelDisplay
 * @see FilterTfFrameNames()
 * @see TfAgeVisualForTimeout()
 */

#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <QImage>
#include <QQuaternion>
#include <QRgb>
#include <QVector3D>

#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>

#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"
#include "autoviz/transform/tf2/signal.hpp"

namespace autoviz {
namespace display {

/**
 * @struct TfFrameSnapshot
 * @brief Public read-only snapshot of one TF frame for UI / tree panels.
 */
struct TfFrameSnapshot {
  std::string name;              /**< Frame id. */
  std::string parent;            /**< Parent frame id (empty = root). */
  bool enabled = true;           /**< Per-frame visibility toggle. */
  QVector3D position;            /**< Pose in fixed frame (if known). */
  QQuaternion orientation;       /**< Orientation in fixed frame. */
  QVector3D rel_position;        /**< Translation relative to parent. */
  QQuaternion rel_orientation;   /**< Rotation relative to parent. */
  bool have_fixed_pose = false;  /**< Whether fixed-frame pose is valid. */
};

/**
 * @class TfDisplay
 * @brief RViz2-style TF tree visualization display.
 *
 * ## Data flow
 *
 * - **In:** TF messages + buffer change signal → @ref updateFrames() refreshes
 *   @c frames_ (poses, parents, aging timestamps).
 * - **Out:** @ref onDraw() emits axes / arrows / names for enabled frames.
 * - **UI:** @ref frameSnapshots() / @ref treeEdges() feed the TF tree panel;
 *   @ref setFrameEnabled() persists into display properties / config.
 *
 * @note Destructor disconnects the transforms-changed signal connection.
 */
class TfDisplay : public ChannelDisplay<automsgs::msgs::tf2_msgs::TFMessage> {
 public:
  /**
   * @brief Constructs a TF display (default channel @c "/tf").
   *
   * @param channel Autolink / topic name for @c tf2_msgs/TFMessage.
   */
  explicit TfDisplay(std::string channel = "/tf");

  /**
   * @brief Disconnects the TF buffer listener.
   */
  ~TfDisplay() override;

  /**
   * @brief Catalog type id used by the display registry.
   * @return Always @c "TF".
   */
  std::string typeId() const override { return "TF"; }

  /**
   * @brief Declares show-*, scale, timeout, and filter properties.
   * @return Property specification list for the Displays tree.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

  /**
   * @brief Returns a snapshot of all known frames for UI consumers.
   * @return Vector of @ref TfFrameSnapshot (copy).
   * @see treeEdges()
   */
  std::vector<TfFrameSnapshot> frameSnapshots() const;

  /**
   * @brief Returns parent links as (child, parent) pairs.
   *
   * Empty parent means the child is treated as a tree root.
   *
   * @return Edge list for building a TF tree widget.
   */
  std::vector<std::pair<std::string, std::string>> treeEdges() const;

  /**
   * @brief Enables or disables drawing for every frame.
   *
   * @param enabled Master enable applied to all frames.
   * @see setFrameEnabled()
   */
  void setAllFramesEnabled(bool enabled);

  /**
   * @brief Enables or disables a single frame by id.
   *
   * @param frame Frame id.
   * @param enabled Per-frame visibility.
   */
  void setFrameEnabled(const std::string& frame, bool enabled);

  /**
   * @brief Whether the global “all frames enabled” flag is set.
   * @return Current @c all_enabled_.
   */
  bool allFramesEnabled() const { return all_enabled_; }

  /**
   * @brief Clears frames and receive counters, then base @ref ChannelDisplay::reset.
   */
  void reset() override;

  /**
   * @brief Loads display config including per-frame enabled map.
   *
   * @param config RViz-style Config node for this display.
   */
  void load(const common::Config& config) override;

  /**
   * @brief Saves display config including per-frame enabled map.
   *
   * @param config Destination Config node.
   */
  void save(common::Config config) const override;

  /**
   * @brief Writes enabled-frame state into session @ref common::DisplayConfig.
   *
   * @param config Destination display config; must be non-null.
   */
  void saveToConfig(common::DisplayConfig* config) const override;

 protected:
  /**
   * @brief Subscribes to TF and attaches the transforms-changed listener.
   */
  void onEnable() override;

  /**
   * @brief Unsubscribes and detaches the transforms-changed listener.
   */
  void onDisable() override;

  /**
   * @brief Rate-limited frame refresh driven by @c update_interval.
   */
  void onUpdate() override;

  /**
   * @brief Reacts to filter / show-* / timeout property edits.
   *
   * @param key Changed property key.
   */
  void onPropertyChanged(const std::string& key) override;

  /**
   * @brief Notes that a TFMessage arrived (pose refresh is buffer-driven).
   *
   * @param message Incoming @c tf2_msgs/TFMessage.
   */
  void processMessage(
      const automsgs::msgs::tf2_msgs::TFMessage& message) override;

  /**
   * @brief Draws axes, arrows, and names for visible frames.
   *
   * @param scene Scene overlay / Ogre host for the current frame.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  /**
   * @struct FrameInfo
   * @brief Internal per-frame state including aging and cached name label.
   */
  struct FrameInfo {
    std::string name;            /**< Frame id. */
    std::string parent;          /**< Parent frame id. */
    bool enabled = true;         /**< Per-frame visibility. */
    QVector3D position;          /**< Fixed-frame translation. */
    QQuaternion orientation = QQuaternion(1.f, 0.f, 0.f, 0.f); /**< Fixed-frame rotation. */
    QVector3D rel_position;      /**< Relative translation to parent. */
    QQuaternion rel_orientation = QQuaternion(1.f, 0.f, 0.f, 0.f); /**< Relative rotation. */
    /** Wall time when transform-to-fixed last changed (RViz last_update_). */
    int64_t last_update_ns = 0;
    /** Last observed TF stamp / common time for aging (RViz last_time_to_fixed_). */
    int64_t last_time_to_fixed_ns = -1;
    bool have_fixed_pose = false; /**< Whether @c position / @c orientation are valid. */
    /** Cached Show Names label (avoid QImage raster every draw). */
    QImage name_label;
    QRgb name_label_rgba = 0;     /**< Color used when @c name_label was baked. */
    int name_label_pixel_height = 0; /**< Pixel height of cached label. */
  };

  /**
   * @struct CachedProps
   * @brief Snapshot of frequently read properties for one update/draw cycle.
   */
  struct CachedProps {
    bool show_names = false;       /**< Draw frame name labels. */
    bool show_axes = true;         /**< Draw RGB axes. */
    bool show_arrows = true;       /**< Draw parent→child arrows. */
    float marker_scale = 1.f;      /**< Multiplier on default axis length. */
    float update_interval = 0.f;   /**< Seconds between frame refreshes; 0 = every tick. */
    float frame_timeout = 15.f;    /**< Aging timeout (seconds); ≤0 disables. */
    std::string filter_whitelist;  /**< Regex whitelist (empty = pass-all). */
    std::string filter_blacklist;  /**< Regex blacklist (empty = ban-none). */
  };

  /**
   * @brief Rebuilds @c frames_ from the TF buffer and filters.
   */
  void updateFrames();

  /**
   * @brief Copies property values into @c props_.
   */
  void refreshCachedProps();

  /**
   * @brief Connects to the TF buffer transforms-changed signal once.
   */
  void ensureTransformsListener();

  /**
   * @brief Marks that @ref updateFrames() should run on the next opportunity.
   */
  void markFramesDirty();

  /**
   * @brief Applies persisted enable state from config onto @p info.
   *
   * @param info Frame to update; must be non-null.
   */
  void applyEnabledFromConfig(FrameInfo* info);

  /**
   * @brief Writes current per-frame enabled flags back into properties.
   */
  void persistEnabledIntoProperties();

  /**
   * @brief Updates aging timestamps when the transform-to-fixed time changes.
   *
   * @param info Frame to update; must be non-null.
   * @param latest_time_ns Latest observed stamp / common time (ns).
   */
  void noteFrameTransformTime(FrameInfo* info, int64_t latest_time_ns);

  /**
   * @brief Sets OK status summarizing frame counts and last draw stats.
   */
  void setOkStatus();

  std::map<std::string, FrameInfo> frames_; /**< Frame id → state. */
  std::map<std::string, bool> frame_enabled_from_config_; /**< Loaded enable map. */
  CachedProps props_;                      /**< Cached property snapshot. */
  bool all_enabled_ = true;                /**< Global enable flag. */
  bool changing_single_frame_ = false;     /**< Guard against recursive property writes. */
  bool frames_dirty_ = true;               /**< Needs @ref updateFrames(). */
  double update_timer_sec_ = 0.0;          /**< Accumulator for update_interval. */
  double last_wall_sec_ = 0.0;             /**< Last wall time used for dt. */
  int64_t last_msg_wall_ns_ = 0;           /**< Wall time of last TF message. */
  uint64_t tf_messages_received_ = 0;      /**< Received TFMessage count. */
  int last_drew_frames_ = 0;               /**< Frames drawn last @ref onDraw. */
  int last_posed_frames_ = 0;              /**< Frames with valid pose last update. */
  std::string filter_error_;               /**< Regex compile error text, if any. */
  std::string fixed_frame_cached_;         /**< Fixed frame at last update. */
  bool transforms_listener_attached_ = false; /**< Whether signal is connected. */
  transform::tf2::VoidSignal::Connection transforms_changed_connection_; /**< Signal link. */
};

}  // namespace display
}  // namespace autoviz
