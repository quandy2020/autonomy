/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file panel.hpp
 * @brief Foxglove-style raw message inspector for a selected Autolink channel.
 *
 * Subscribes via @ref integration::MessageQueue, parses protobuf payloads, and
 * renders them in @ref raw_messages::RawMessageTreeWidget. Supports channel
 * drag-and-drop and "add field to Plot" requests.
 *
 * @see raw_messages::RawMessageTreeWidget
 * @see ChannelsPanel
 * @see common::VisualizationManager
 */

#pragma once

#include <QWidget>

#include <cstdint>
#include <string>

#include "autoviz/common/session_config.hpp"
#include "autoviz/integration/message_queue.hpp"

class QComboBox;
class QDragEnterEvent;
class QDragMoveEvent;
class QDropEvent;
class QFrame;
class QLabel;
class QLineEdit;
class QMimeData;
class QTimer;
class QToolButton;

namespace google {
namespace protobuf {
class Message;
}  // namespace protobuf
}  // namespace google

namespace autoviz {
namespace raw_messages {
class RawMessageTreeWidget;
}  // namespace raw_messages

namespace common {
class VisualizationManager;
}

/**
 * @class RawMessagesPanel
 * @brief Channel picker + message-path filter + live protobuf tree.
 *
 * ## Layout
 *
 * @code
 * ┌──────────────────────────────────────────────┐
 * │ Channel [_________▾]  Path [/field…]  status │
 * │ schema badge / type label                    │
 * ├──────────────────────────────────────────────┤
 * │ ▾ MyMsg                                      │
 * │     header.stamp.sec        123              │
 * │     pose.position.x         1.2              │
 * └──────────────────────────────────────────────┘
 * @endcode
 *
 * ## Data flow
 *
 * - @ref refreshChannels() / @ref selectChannel() manage the combo and
 *   subscription (@ref resubscribe()).
 * - @c tick_timer_ drains @c payload_queue_ and calls @ref showPayload() /
 *   @ref renderMessage().
 * - @ref refreshFromVariables() re-renders @c last_payload_ after global
 *   variable changes (message-path substitution).
 *
 * Dropping a channel MIME onto the panel calls @ref selectChannel().
 *
 * @note Unsubscribes in the destructor; @c manager_ is non-owning.
 *
 * @see addToPlotRequested()
 * @see PopulateMessageTree()
 */
class RawMessagesPanel : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs the panel, applies chrome, and starts the tick timer.
   *
   * @param manager Non-owning visualization manager for channels / subscribe.
   * @param parent Qt parent widget.
   */
  explicit RawMessagesPanel(common::VisualizationManager* manager,
                            QWidget* parent = nullptr);

  /**
   * @brief Unsubscribes and stops the tick timer.
   */
  ~RawMessagesPanel() override;

  /**
   * @brief Rebuilds the channel combo when the live channel set changes.
   *
   * @see channelsStructureChanged()
   * @see rebuildChannelCombo()
   */
  void refreshChannels();

  /**
   * @brief Selects @p channel in the combo and resubscribes.
   *
   * @param channel Autolink channel name; no-op if empty / unknown.
   * @see resubscribe()
   */
  void selectChannel(const QString& channel);

  /**
   * @brief Captures channel / path / freeze / diff options for session persist.
   */
  common::RawMessagesPersistConfig config() const;

  /**
   * @brief Restores session persist state into the panel.
   */
  void setConfig(const common::RawMessagesPersistConfig& config);

  /**
   * @brief Re-render the last payload after global variables change.
   *
   * Re-evaluates @ref resolvedMessagePath() and refreshes the tree without
   * waiting for a new message.
   */
  void refreshFromVariables();

 signals:
  /**
   * @brief Request adding a numeric field as a Plot series (drag target /
   *        context menu).
   *
   * @param channel Source Autolink channel.
   * @param field_path Dot-separated protobuf field path.
   */
  void addToPlotRequested(const QString& channel, const QString& field_path);

  /**
   * @brief Emitted when toolbar options that belong in session config change.
   */
  void configChanged();

 protected:
  /**
   * @brief Accepts channel MIME drags over the panel.
   *
   * @param event Drag enter event.
   */
  void dragEnterEvent(QDragEnterEvent* event) override;

  /**
   * @brief Continues accepting valid channel MIME drags.
   *
   * @param event Drag move event.
   */
  void dragMoveEvent(QDragMoveEvent* event) override;

  /**
   * @brief Selects the dropped channel via @ref selectChannel().
   *
   * @param event Drop event.
   */
  void dropEvent(QDropEvent* event) override;

 private slots:
  /**
   * @brief Channel combo activated: clears selection state and resubscribes.
   *
   * @param index Combo index.
   */
  void onChannelChanged(int index);

  /**
   * @brief Message-path line edited: re-filters / re-renders the tree.
   */
  void onMessagePathEdited();

  /**
   * @brief Freeze toggle — pauses live tree updates.
   */
  void onFreezeToggled(bool frozen);

  /**
   * @brief Diff-highlight toggle for leaf value changes.
   */
  void onDiffToggled(bool enabled);

  /**
   * @brief Copies the latest parsed message as JSON to the clipboard.
   */
  void onCopyMessageJsonRequested();

  /**
   * @brief Tick timer: drains @c payload_queue_ and updates the tree.
   */
  void onTick();

 private:
  /**
   * @brief Applies theme / frosted styles to chrome widgets.
   */
  void applyChromeStyles();

  /**
   * @brief Unsubscribes any previous id and subscribes to @c active_channel_.
   */
  void resubscribe();

  /**
   * @brief Cancels the current subscription if any.
   */
  void unsubscribe();

  /**
   * @brief Clears active channel, payload cache, and tree contents.
   */
  void clearSelection();

  /**
   * @brief Re-subscribes if the channel exists again after a topology blip.
   */
  void tryResubscribeIfNeeded();

  /**
   * @brief Returns whether the live channel key set differs from the cache.
   *
   * @return @c true when @ref rebuildChannelCombo() is required.
   */
  bool channelsStructureChanged();

  /**
   * @brief Repopulates @c channel_combo_ from ChannelManager.
   */
  void rebuildChannelCombo();

  /**
   * @brief Updates schema badge / type label for the active channel.
   */
  void updateSchemaHeader();

  /**
   * @brief Sets @c status_label_ text (Hz, errors, waiting, …).
   *
   * @param text Status chip contents.
   */
  void updateStatusChip(const QString& text);

  /**
   * @brief Shows a schema-only tree before the first payload arrives.
   *
   * @see PopulateSchemaTree()
   */
  void showSchemaPlaceholder();

  /**
   * @brief Parses @p payload and renders via @ref renderMessage().
   *
   * @param payload Serialized protobuf bytes from the queue.
   */
  void showPayload(const std::string& payload);

  /**
   * @brief Populates or updates the message tree from a parsed message.
   *
   * Prefers @ref UpdateMessageTreeValues() when structure is unchanged;
   * otherwise @ref PopulateMessageTree().
   *
   * @param message Parsed protobuf message.
   */
  void renderMessage(const google::protobuf::Message& message);

  /**
   * @brief Looks up the protobuf type name for @p channel.
   *
   * @param channel Autolink channel name.
   * @return Fully-qualified message type, or empty if unknown.
   */
  std::string messageTypeForChannel(const std::string& channel) const;

  /**
   * @brief Returns whether @p mime carries a droppable channel reference.
   *
   * @param mime Drag MIME data.
   * @return @c true when the panel should accept the drop.
   */
  bool acceptChannelDrop(const QMimeData* mime) const;

  /**
   * @brief Updates status chip with Live/Frozen + Hz when available.
   */
  void refreshStatusChip();

  /**
   * @brief Message-path editor text after global-variable substitution.
   *
   * @return Resolved path filter applied to the tree.
   */
  QString resolvedMessagePath() const;

  /** Non-owning; channel list and subscribe APIs. */
  common::VisualizationManager* manager_ = nullptr;

  /** Channel picker combo. */
  QComboBox* channel_combo_ = nullptr;

  /** Optional message-path / field filter (supports variables). */
  QLineEdit* message_path_edit_ = nullptr;

  /** Freeze live updates. */
  QToolButton* freeze_button_ = nullptr;

  /** Highlight changed leaves vs previous frame. */
  QToolButton* diff_button_ = nullptr;

  /** Compact schema / encoding badge. */
  QLabel* schema_badge_ = nullptr;

  /** Human-readable message type label. */
  QLabel* schema_label_ = nullptr;

  /** Status chip (rate, waiting, error). */
  QLabel* status_label_ = nullptr;

  /** Empty-state hint when no channel is selected. */
  QFrame* empty_hint_ = nullptr;

  /** Collapsible protobuf field tree. */
  raw_messages::RawMessageTreeWidget* message_tree_ = nullptr;

  /** Periodic timer draining @c payload_queue_. */
  QTimer* tick_timer_ = nullptr;

  /** Thread-safe queue of incoming serialized payloads. */
  integration::MessageQueue payload_queue_;

  /** Last known channel keys; used by @ref channelsStructureChanged(). */
  QStringList cached_channel_keys_;

  /** Active subscription id from ChannelManager (0 = none). */
  std::uint64_t subscription_id_ = 0;

  /** Currently subscribed channel name (UTF-8 / std::string form). */
  std::string active_channel_;

  /** Most recent raw payload bytes. */
  std::string last_payload_;

  /** Payload last successfully rendered (dedupe identical frames). */
  std::string last_rendered_payload_;

  /** True after the tree has been seeded with schema or first message. */
  bool message_tree_seeded_ = false;

  /** Root label used for the last tree populate (structure fingerprint). */
  QString last_tree_root_label_;

  /** Path filter used for the last tree populate. */
  QString last_tree_path_filter_;

  /** When true, ignore new payloads for tree rendering. */
  bool freeze_ = false;

  /** When true, highlight leaf values that changed. */
  bool diff_highlight_ = true;
};

}  // namespace autoviz
