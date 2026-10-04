/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file channels_panel.hpp
 * @brief Foxglove-style Channels sidebar — browse Autolink channels and drag
 *        fields into Plot / Raw / Image / Map panels.
 *
 * Lists live channels from @ref common::VisualizationManager / ChannelManager,
 * shows lightweight stats (optional probe), type chips, and expand-on-demand
 * schema browsing with last-value preview for numeric fields.
 *
 * @see RawMessagesPanel
 * @see ChannelGraphPanel
 * @see common::VisualizationManager
 */

#pragma once

#include <QWidget>

#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>

#include "autoviz/common/session_config.hpp"

class QLabel;
class QLineEdit;
class QToolButton;
class QTreeWidget;
class QTreeWidgetItem;
class QTimer;
class QPoint;

namespace autoviz {
namespace common {
class VisualizationManager;
}

/**
 * @class ChannelsPanel
 * @brief Sidebar tree of Autolink channels with filter, type chips, and stats.
 *
 * ## Layout
 *
 * @code
 * ┌──────────────────────────────────────────────┐
 * │ Filter [____] [All|Num|Img|Geo|TF] Probe  n  │
 * ├──────────────────────────────────────────────┤
 * │ ▾ /robot/odom   Odometry  50  12k  —         │
 * │     pose.position.x        —   —   1.24      │
 * └──────────────────────────────────────────────┘
 * @endcode
 *
 * ## Refresh strategy
 *
 * - @ref refreshChannels() rebuilds when the channel set changes.
 * - @ref refreshStats() updates Hz / Count / Value in place.
 * - Optional stats probe (@ref syncStatsProbes) short-subscribes visible leaves
 *   so Hz is trustworthy without permanent full-topology subscriptions.
 *
 * @note Non-owning @c manager_; lifetime owned by VisualizationFrame.
 */
class ChannelsPanel : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs the panel, applies chrome styles, and starts the stats
   *        timer.
   *
   * @param manager Non-owning visualization manager for channel enumeration.
   * @param parent Qt parent widget (typically the Channels dock).
   */
  explicit ChannelsPanel(common::VisualizationManager* manager,
                         QWidget* parent = nullptr);

  /**
   * @brief Unsubscribes stats probes.
   */
  ~ChannelsPanel() override;

  /**
   * @brief Rebuilds the channel tree when structure changes; otherwise a no-op.
   *
   * Compares against @c cached_channel_keys_ via @ref channelsStructureChanged().
   *
   * @see rebuildTree()
   */
  void refreshChannels();

  /**
   * @brief Refreshes per-channel stats columns and the status chip.
   *
   * Safe to call at a high rate; does not rebuild tree structure.
   *
   * @see updateChannelStatsColumns()
   * @see updateStatusChip()
   */
  void refreshStats();

  /**
   * @brief Snapshot of filter / type chip / probe / expanded channels.
   *
   * @return Persistable Channels browser state.
   */
  common::ChannelsBrowserPersistConfig config() const;

  /**
   * @brief Restores filter / type chip / probe / expanded channels.
   *
   * @param config Persist state from session.
   */
  void setConfig(const common::ChannelsBrowserPersistConfig& config);

 signals:
  /**
   * @brief Open @p channel in the Raw Messages panel.
   *
   * @param channel Autolink channel name.
   */
  void openInRawMessagesRequested(const QString& channel);

  /**
   * @brief Add a numeric field as a Plot series.
   *
   * @param channel Source Autolink channel.
   * @param field_path Dot-separated protobuf field path.
   */
  void addToPlotRequested(const QString& channel, const QString& field_path);

 private:
  /** Type filter chip selection. */
  enum class TypeFilter {
    kAll = 0,
    kNumeric = 1,
    kImage = 2,
    kGeo = 3,
    kTf = 4,
  };

  /**
   * @brief Applies theme / frosted styles to filter, tree, and status widgets.
   */
  void applyChromeStyles();

  /**
   * @brief Clears and rebuilds top-level channel items from ChannelManager.
   *
   * Updates @c cached_channel_keys_ and toggles @c empty_hint_ visibility.
   */
  void rebuildTree();

  /**
   * @brief Shows/hides rows based on filter text + type chip.
   */
  void applyFilter();

  /**
   * @brief Lazily populates field children when a channel row is expanded.
   *
   * @param item Expanded tree item (channel leaf or group).
   */
  void onItemExpanded(QTreeWidgetItem* item);

  /**
   * @brief Returns whether the live channel key set differs from the cache.
   *
   * @return @c true when @ref rebuildTree() is required.
   */
  bool channelsStructureChanged() const;

  /**
   * @brief Writes Hz / count / last-value text into existing rows.
   */
  void updateChannelStatsColumns();

  /**
   * @brief Updates @c status_label_ with channel count / connection summary.
   */
  void updateStatusChip();

  /**
   * @brief Applies leaf vs. group styling (font, icon, columns) to @p item.
   *
   * @param item Tree item to style.
   * @param is_channel_leaf @c true for a concrete channel row.
   */
  void styleChannelItem(QTreeWidgetItem* item, bool is_channel_leaf) const;

  /**
   * @brief Context menu: Copy path / Open Raw / Add to Plot.
   *
   * @param pos Viewport-local position.
   */
  void onCustomContextMenu(const QPoint& pos);

  /**
   * @brief Double-click a channel leaf → Open in Raw Messages.
   *
   * @param item Clicked tree item.
   * @param column Unused.
   */
  void onItemDoubleClicked(QTreeWidgetItem* item, int column);

  /**
   * @brief Toggle lightweight stats probe subscriptions.
   *
   * @param enabled When @c true, short-subscribe visible channel leaves.
   */
  void setProbeEnabled(bool enabled);

  /**
   * @brief Apply type chip index without rebuilding the chip UI.
   *
   * @param index TypeFilter ordinal.
   */
  void setTypeFilterIndex(int index);

  /**
   * @brief Sync probe subscriptions to currently visible channel leaves.
   *
   * Caps concurrent probes to avoid bandwidth spikes.
   */
  void syncStatsProbes();

  /**
   * @brief Unsubscribe all probe subscriptions and clear last-payload cache.
   */
  void clearStatsProbes();

  /**
   * @brief Cache a probe payload (reader thread → mutex).
   *
   * @param channel Channel name.
   * @param message_type Schema type.
   * @param payload Wire payload.
   */
  void storeProbePayload(const std::string& channel,
                         const std::string& message_type,
                         const std::string& payload);

  /**
   * @brief Whether @p message_type matches the active type chip.
   *
   * @param message_type Schema type string.
   * @return @c true when the channel should stay visible.
   */
  bool matchesTypeFilter(const QString& message_type) const;

  /**
   * @brief Collects currently expanded channel leaf names.
   *
   * @return Fully-qualified channel names.
   */
  QStringList expandedChannels() const;

  /**
   * @brief Expands channel leaves matching @p channels (populates fields).
   *
   * @param channels Fully-qualified channel names.
   */
  void expandChannels(const QStringList& channels);

  /**
   * @brief Sync checkable type-chip buttons to @c type_filter_.
   */
  void syncTypeFilterChips() const;

  /** Non-owning; provides ChannelManager access. */
  common::VisualizationManager* manager_ = nullptr;

  /** Filter line edit (substring match on channel names). */
  QLineEdit* filter_edit_ = nullptr;

  /** Compact status chip (counts / health). */
  QLabel* status_label_ = nullptr;

  /** Shown when no channels are available. */
  QLabel* empty_hint_ = nullptr;

  /** Channel / field tree. */
  QTreeWidget* tree_ = nullptr;

  /** Toggle lightweight Hz probe subscriptions. */
  QToolButton* probe_button_ = nullptr;

  /** Type filter chip container (All/Numeric/…). */
  QWidget* type_filter_chips_ = nullptr;

  /** Periodic timer driving @ref refreshStats(). */
  QTimer* stats_timer_ = nullptr;

  /** Last known sorted channel keys; used by @ref channelsStructureChanged(). */
  QStringList cached_channel_keys_;

  /** Active type chip. */
  TypeFilter type_filter_ = TypeFilter::kAll;

  /** Whether stats probe subscriptions are active. */
  bool probe_enabled_ = true;

  /** Channel → ChannelReaderRegistry subscription id for probes. */
  std::unordered_map<std::string, std::uint64_t> probe_subscriptions_;

  /** Guards @c last_payloads_ / @c last_message_types_. */
  mutable std::mutex probe_mutex_;

  /** Latest probed payload per channel (for last-value column). */
  std::unordered_map<std::string, std::string> last_payloads_;

  /** Message type recorded with each probed payload. */
  std::unordered_map<std::string, std::string> last_message_types_;
};

}  // namespace autoviz
