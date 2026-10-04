/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file publish_editor_widget.hpp
 * @brief Main Publish panel editor — channel / type / JSON / multi-publisher.
 *
 * Hosts the rqt-style publisher collection, field tree, JSON tabs, presets,
 * and once / loop publish timers. Owned by @ref PublishPanel.
 *
 * ## Data flow
 *
 * - **Out:** user edits mutate @c config_ and emit @ref configChanged();
 *   publish actions write via VisualizationManager / channel writers.
 * - **In:** @ref setConfig() restores UI; @ref refreshChannels() refreshes
 *   discovery lists.
 *
 * @see PublishFieldTreeWidget
 * @see PublishMessageCodec
 * @see PublishPanel
 */

#pragma once

#include <QHash>
#include <QWidget>

#include <QStringList>

#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/ui/publish/publish_types.hpp"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QPlainTextEdit;
class QPushButton;
class QSplitter;
class QTabWidget;
class QTimer;

namespace autoviz {
namespace common {
class VisualizationManager;
}

namespace publish_panel {

class PublishFieldTreeWidget;

/**
 * @class PublishEditorWidget
 * @brief Interactive editor for composing and publishing protobuf messages.
 *
 * ## Layout (editing mode)
 *
 * @code
 * ┌─ presets / channel / type / rate ─────────────────────────┐
 * │ [+][-] publishers tree (rqt)                              │
 * ├─ Message tabs: Fields | JSON | Metadata ──────────────────┤
 * │ result status + last publish summary                      │
 * └───────────────────────────────────────────────────────────┘
 * @endcode
 *
 * Supports a draft publisher (channel combo) and a multi-row collection with
 * per-entry loop timers keyed by @ref PublishEntry::id.
 *
 * @note Does not own @ref common::VisualizationManager; the panel / frame
 *       keeps that lifetime.
 */
class PublishEditorWidget : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Constructs the editor UI and wires signals.
   *
   * @param manager Non-owning visualization manager for channel discovery and
   *        publish; may be @c nullptr until later (publish stays disabled).
   * @param parent Qt parent (typically @ref PublishPanel content).
   */
  explicit PublishEditorWidget(common::VisualizationManager* manager,
                               QWidget* parent = nullptr);

  /**
   * @brief Returns the current panel configuration (draft + publishers).
   *
   * @return Copy of @c config_ synchronized from the UI.
   * @see setConfig()
   */
  PublishPanelConfig config() const;

  /**
   * @brief Replaces configuration and refreshes all editor widgets.
   *
   * Restores presets, publishers tree, timers, and draft fields.
   *
   * @param config Full @ref PublishPanelConfig to apply.
   * @see config()
   */
  void setConfig(const PublishPanelConfig& config);

  /**
   * @brief Rebuilds the channel combo from discovery + custom channels.
   *
   * Lightweight refresh when the topic list changes without a full
   * @ref setConfig().
   */
  void refreshChannels();

 signals:
  /**
   * @brief Emitted when any durable editor state changes.
   *
   * Listeners should persist @ref config() into session config.
   */
  void configChanged();

  /**
   * @brief Emitted when the user requests an immediate publish action.
   *
   * Used by hosting chrome; the editor also publishes directly via timers /
   * Publish Once.
   */
  void publishRequested();

 protected:
  /**
   * @brief Keyboard shortcuts (e.g. publish / focus helpers).
   *
   * @param event Key event from Qt.
   */
  void keyPressEvent(QKeyEvent* event) override;

 private slots:
  /**
   * @brief Editing-mode checkbox: shows/hides advanced editor chrome.
   *
   * @param enabled New checkbox state.
   */
  void onEditingModeToggled(bool enabled);

  /**
   * @brief Channel combo text changed: updates draft channel / type hints.
   *
   * @param text Channel name.
   */
  void onChannelChanged(const QString& text);

  /**
   * @brief Message-type combo changed: may fill a default JSON template.
   *
   * @param text Fully-qualified type name.
   */
  void onMessageTypeChanged(const QString& text);

  /** @brief Reloads the default JSON template for the current type. */
  void onResetTemplate();

  /** @brief Subscribes briefly to fill JSON from the latest channel message. */
  void onFillFromLatest();

  /** @brief Refreshes discovered topics into the channel combo. */
  void onRefreshTopics();

  /** @brief Publishes the draft / selected entry once (not from loop timer). */
  void onPublishOnceClicked();

  /** @brief Appends a new @ref PublishEntry and selects it. */
  void onAddPublisher();

  /** @brief Removes the selected publisher row and stops its timer. */
  void onRemovePublisher();

  /**
   * @brief Publishers tree edited: syncs JSON / config for @p index.
   *
   * @param index Publisher row that changed.
   */
  void onPublisherEdited(int index);

  /**
   * @brief Publishing flag toggled on publisher @p index.
   *
   * @param index Publisher row.
   * @param publishing New loop-enable state.
   */
  void onPublisherPublishingChanged(int index, bool publishing);

  /**
   * @brief Rate cell edited for publisher @p index.
   *
   * @param index Publisher row.
   * @param rate_hz New rate in Hz.
   */
  void onPublisherRateChanged(int index, double rate_hz);

  /** @brief Single-message field tree edited; marks fields dirty. */
  void onFieldEdited();

  /**
   * @brief Preset combo activated: applies the named preset.
   *
   * @param index Combo index.
   */
  void onPresetSelected(int index);

  /** @brief Saves the current draft as a named preset. */
  void onSavePreset();

  /** @brief Deletes the currently selected preset. */
  void onDeletePreset();

  /**
   * @brief Draft publish-rate spin changed.
   *
   * @param rate New rate in Hz.
   */
  void onPublishRateChanged(double rate);

  /**
   * @brief Message tab bar switched (Fields / JSON / Metadata).
   *
   * @param index Tab index.
   */
  void onMessageTabChanged(int index);

  /** @brief Field-tree edits that should sync JSON / emit config. */
  void onFieldsEdited();

 private:
  /**
   * @brief Factory for styled JSON plain-text editors.
   *
   * @param parent Parent widget for the editor.
   * @param placeholder Placeholder text when empty.
   * @param read_only When @c true, editor is not user-editable.
   * @return Owned @c QPlainTextEdit (parented to @p parent).
   */
  QPlainTextEdit* makeJsonEditor(QWidget* parent, const QString& placeholder,
                                 bool read_only = false);

  /** @brief Repopulates the message-type combo from the codec. */
  void rebuildMessageTypeList();

  /** @brief Rebuilds the preset combo from @c config_.saved_presets. */
  void rebuildPresetCombo();

  /** @brief Pushes @c config_.publishers into @c publishers_tree_. */
  void rebuildPublishersTree();

  /** @brief Shows/hides advanced chrome based on editing mode. */
  void applyEditingModeUi();

  /** @brief Enables/disables Publish Once based on selection / validity. */
  void updatePublishButtonState();

  /**
   * @brief Updates the result status label and details pane.
   *
   * @param success Whether the last operation succeeded.
   * @param summary Short status line.
   * @param details Longer diagnostic / payload text.
   */
  void showResult(bool success, const QString& summary, const QString& details);

  /**
   * @brief Publishes the draft entry (channel combo state).
   *
   * @param from_loop @c true when invoked by a timer tick.
   * @return @c true on successful encode + write.
   */
  bool publishDraft(bool from_loop);

  /**
   * @brief Publishes publishers[@p index].
   *
   * @param index Publisher row index.
   * @param from_loop @c true when invoked by that entry's timer.
   * @return @c true on successful encode + write.
   */
  bool publishEntry(int index, bool from_loop);

  /** @brief Loads the field tree from the JSON editor contents. */
  void syncFieldsFromJson();

  /** @brief Writes the field tree back into the JSON editor. */
  void syncJsonFromFields();

  /** @brief Refreshes the read-only metadata pane for the current type. */
  void updateMetadataPanel();

  /** @brief Copies current UI draft fields into @c config_. */
  void captureDraftFromUi();

  /**
   * @brief Selected row in the publishers tree (−1 if none).
   *
   * @return Publisher index.
   */
  int selectedPublisherRow() const;

  /**
   * @brief Whether the current selection is ready to publish.
   *
   * @return @c true when channel / type / JSON are usable.
   */
  bool selectedPublisherCanPublish() const;

  /**
   * @brief Finds a publisher by stable @p id.
   *
   * @param id @ref PublishEntry::id.
   * @return Index, or @c -1 if not found.
   */
  int findPublisherIndexById(const QString& id) const;

  /**
   * @brief Finds a publisher by channel name.
   *
   * @param channel Channel string.
   * @return Index of the first match, or @c -1.
   */
  int findPublisherIndexByChannel(const QString& channel) const;

  /**
   * @brief Resolves which publisher row is "active" for Publish Once.
   *
   * @return Index into @c config_.publishers, or @c -1 for draft-only.
   */
  int resolveActivePublisherRow() const;

  /**
   * @brief Starts or restarts the loop timer for @p entry.
   *
   * @param entry Publisher whose @c id keys @c publisher_timers_.
   */
  void startPublisherTimer(const PublishEntry& entry);

  /**
   * @brief Stops and deletes the timer for @p entry_id.
   *
   * @param entry_id @ref PublishEntry::id.
   */
  void stopPublisherTimer(const QString& entry_id);

  /** @brief Stops every per-publisher loop timer. */
  void stopAllPublisherTimers();

  /** @brief Recreates timers for all publishing entries after @ref setConfig(). */
  void restorePublisherTimers();

  /**
   * @brief Effective message type for the draft (combo or channel inference).
   *
   * @return Fully-qualified type name, may be empty.
   */
  QString resolvedMessageType() const;

  /**
   * @brief Looks up a discovered type for @p channel, if known.
   *
   * @param channel Channel name.
   * @return Type string, or empty.
   */
  QString messageTypeForChannel(const QString& channel) const;

  /**
   * @brief Remembers a user-typed channel in @c custom_channels_.
   *
   * @param channel Channel to keep in the combo across refreshes.
   */
  void rememberCustomChannel(const QString& channel);

  /**
   * @brief Sets the message-type combo without treating it as a user edit.
   *
   * @param message_type Type string to select / insert.
   */
  void setMessageTypeField(const QString& message_type);

  /**
   * @brief Replaces message JSON and updates the field tree.
   *
   * @param json New JSON body.
   * @param user_edited When @c true, marks @c message_user_edited_.
   */
  void fillMessageJson(const QString& json, bool user_edited);

  /**
   * @brief Fills a default template unless the user already edited the body.
   *
   * @param message_type Type whose template should be loaded.
   */
  void maybeFillTemplateForType(const QString& message_type);

  /**
   * @brief Starts an async fill-from-latest for @p channel / @p message_type.
   *
   * @param channel Channel to read.
   * @param message_type Expected payload type.
   */
  void requestAutoFillMessage(const QString& channel, const QString& message_type);

  /** @brief Cancels an in-flight latest-message fill subscription / timer. */
  void cancelLatestMessageFill();

  /**
   * @brief Applies a saved preset into draft UI + config.
   *
   * @param preset Preset to load.
   */
  void applyPreset(const PublishPreset& preset);

  /** @brief Ensures the draft has a non-empty JSON expression when needed. */
  void ensureDraftExpression();

  /**
   * @brief Validates / completes a draft @ref PublishEntry before publish.
   *
   * @param entry In/out entry to prepare.
   * @param error Out-parameter for a human-readable failure reason.
   * @return @c true if @p entry is ready to encode and write.
   */
  bool prepareDraftPublisher(PublishEntry* entry, QString* error);

  /** @brief Switches UI focus back to draft (non-collection) editing. */
  void enterDraftMode();

  /**
   * @brief Loads publishers[@p index] into the draft editors.
   *
   * @param index Publisher row index.
   */
  void loadPublisherAt(int index);

  /** @brief Pulls publisher rows from the tree into @c config_. */
  void syncConfigFromPublishersTree();

  /**
   * @brief Syncs JSON for publisher @p index from the tree subtree.
   *
   * @param index Publisher row index.
   */
  void syncPublisherJsonFromTree(int index);

  /** @brief Emits @ref configChanged() unless suppressed. */
  void emitConfigChanged();

  /** @brief Applies theme / chrome styles to child widgets. */
  void applyChromeStyles();

  /** @brief Updates Publish Once button label / color from config. */
  void refreshPublishButtonAppearance();

  /** Non-owning; channel discovery and publish path. */
  common::VisualizationManager* manager_ = nullptr;

  /** Canonical editor configuration. */
  PublishPanelConfig config_;

  /** Guard: ignore type-combo template side effects while rebuilding. */
  bool suppress_template_update_ = false;

  /** Guard: ignore tree edit signals while rebuilding publishers. */
  bool suppress_tree_update_ = false;

  /** Guard: ignore draft sync while programmatically loading a row. */
  bool suppress_draft_sync_ = false;

  /** @c true after the user manually edits message JSON. */
  bool message_user_edited_ = false;

  /** @c true when field-tree changes are not yet synced to JSON. */
  bool fields_dirty_ = false;

  /** User-typed channels retained across @ref refreshChannels(). */
  QStringList custom_channels_;

  /** Per-publisher loop timers keyed by @ref PublishEntry::id. */
  QHash<QString, QTimer*> publisher_timers_;

  /** Active ChannelReaderRegistry subscription for fill-from-latest. */
  integration::ChannelReaderRegistry::SubscriptionId latest_fill_subscription_ = 0;

  /** Channel currently targeted by fill-from-latest. */
  QString latest_fill_channel_;

  QCheckBox* editing_mode_check_ = nullptr;       /**< Show advanced editor. */
  QWidget* editor_body_ = nullptr;                /**< Advanced editor container. */
  QWidget* collection_bar_ = nullptr;             /**< Preset / collection toolbar. */
  QWidget* rqt_top_bar_ = nullptr;                /**< Channel / type / rate bar. */
  QWidget* expression_toolbar_ = nullptr;         /**< Reset / fill-latest tools. */
  QComboBox* preset_combo_ = nullptr;             /**< Saved preset selector. */
  QPushButton* save_preset_button_ = nullptr;     /**< Save current as preset. */
  QPushButton* delete_preset_button_ = nullptr;   /**< Delete selected preset. */
  QComboBox* channel_combo_ = nullptr;            /**< Draft channel. */
  QComboBox* message_type_combo_ = nullptr;       /**< Draft message type. */
  QDoubleSpinBox* publish_rate_spin_ = nullptr;   /**< Draft loop rate (Hz). */
  QPushButton* add_publisher_button_ = nullptr;   /**< Add publisher row. */
  QPushButton* remove_publisher_button_ = nullptr;/**< Remove selected row. */
  QPushButton* refresh_topics_button_ = nullptr;  /**< Refresh channel list. */
  QPushButton* publish_once_button_ = nullptr;    /**< Publish once action. */
  PublishFieldTreeWidget* publishers_tree_ = nullptr; /**< Rqt publishers table. */
  QPushButton* reset_template_button_ = nullptr;  /**< Reload default JSON. */
  QPushButton* fill_latest_button_ = nullptr;     /**< Fill from latest message. */
  QTabWidget* message_tabs_ = nullptr;            /**< Fields / JSON / Metadata. */
  QSplitter* payload_splitter_ = nullptr;         /**< Request vs result split. */
  QGroupBox* request_group_ = nullptr;            /**< Message payload group. */
  QGroupBox* result_group_ = nullptr;             /**< Last-result group. */
  QLabel* result_status_label_ = nullptr;         /**< Success / error status. */
  QPlainTextEdit* message_edit_ = nullptr;        /**< JSON editor tab. */
  PublishFieldTreeWidget* field_tree_ = nullptr;  /**< Fields editor tab. */
  QPlainTextEdit* metadata_edit_ = nullptr;       /**< Read-only metadata tab. */
  QPlainTextEdit* result_edit_ = nullptr;         /**< Last publish details. */
  QTimer* latest_fill_timer_ = nullptr;           /**< Timeout for fill-from-latest. */
};

}  // namespace publish_panel
}  // namespace autoviz
