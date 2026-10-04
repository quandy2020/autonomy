/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file publish_field_tree.hpp
 * @brief Tree widget for protobuf field editing and rqt-style publisher rows.
 *
 * Dual-mode @c QTreeWidget used by @ref PublishEditorWidget (and reused by
 * @ref service_panel::ServiceEditorWidget for request/response fields):
 * - @ref DisplayMode::kFieldsEditor — Name | Type | Value for one message;
 * - @ref DisplayMode::kRqtPublishers — channel | type | rate | expression
 *   with nested field subtrees per publisher.
 *
 * @see PublishEntry
 * @see PublishMessageCodec
 */

#pragma once

#include <QTreeWidget>

#include <memory>
#include <string>
#include <vector>

#include "autoviz/ui/publish/publish_types.hpp"

class QMenu;

namespace google {
namespace protobuf {
class Message;
}  // namespace protobuf
}  // namespace google

namespace autoviz {
namespace publish_panel {

/**
 * @class PublishFieldTreeWidget
 * @brief Editable protobuf field tree or multi-publisher table (rqt layout).
 *
 * ## Modes
 *
 * @code
 * kFieldsEditor:          kRqtPublishers:
 * ┌────────┬──────┬────┐  ┌─────────┬────────┬──────┬────────────┐
 * │ Name   │ Type │Val │  │ Channel │ Type   │ Rate │ Expression │
 * ├────────┼──────┼────┤  ├─────────┼────────┼──────┼────────────┤
 * │ linear │ …    │ …  │  │ /cmd_vel│ Twist  │ 1.0  │ {…}        │
 * └────────┴──────┴────┘  │   └ field subtree…                   │
 *                         └──────────────────────────────────────┘
 * @endcode
 *
 * Edits emit @ref messageEdited() (single-message) or
 * @ref publisherEdited() / rate / publishing signals (multi-publisher).
 *
 * @note Owns protobuf @c Message instances mirrored from JSON; callers sync
 *       via @ref toJson() / @ref publishers().
 */
class PublishFieldTreeWidget : public QTreeWidget {
  Q_OBJECT

 public:
  /**
   * @enum DisplayMode
   * @brief Selects single-message field editor vs rqt multi-publisher table.
   */
  enum class DisplayMode {
    kFieldsEditor,   /**< Name | Type | Value for one protobuf message. */
    kRqtPublishers,  /**< Channel | type | rate | expression + field subtrees. */
  };

  /**
   * @brief Constructs an empty field tree in @ref DisplayMode::kFieldsEditor.
   *
   * @param parent Qt parent widget.
   */
  explicit PublishFieldTreeWidget(QWidget* parent = nullptr);

  /**
   * @brief Switches between fields-editor and rqt-publishers layouts.
   *
   * Rebuilds columns and clears/rebuilds content for the new mode.
   *
   * @param mode Target display mode.
   */
  void setDisplayMode(DisplayMode mode);

  /**
   * @brief Makes value cells non-editable when @p read_only is @c true.
   *
   * @param read_only Pass @c true for response trees / locked views.
   */
  void setReadOnly(bool read_only);

  /**
   * @brief Overrides the header title of the editable value / expression column.
   *
   * @param title Column header text.
   */
  void setValueColumnTitle(const QString& title);

  /**
   * @brief Loads a single message from JSON into fields-editor mode.
   *
   * @param message_type Fully-qualified protobuf type name.
   * @param json Message body as JSON.
   * @return @c true if parse and tree rebuild succeeded.
   * @see loadTemplate()
   * @see toJson()
   */
  bool loadFromJson(const std::string& message_type, const QString& json);

  /**
   * @brief Loads the default empty template for @p message_type.
   *
   * @param message_type Fully-qualified protobuf type name.
   * @return @c true if a template was built and applied.
   * @see PublishMessageCodec::defaultJsonTemplate()
   */
  bool loadTemplate(const std::string& message_type);

  /**
   * @brief Serializes the current single-message tree back to JSON.
   *
   * @return JSON string, or empty if no message is loaded.
   */
  QString toJson() const;

  /**
   * @brief Whether a single-message protobuf instance is currently loaded.
   *
   * @return @c true if @c message_ is non-null.
   */
  bool hasMessage() const;

  /**
   * @brief Clears the loaded single-message instance and empties the tree.
   */
  void clearMessage();

  /**
   * @brief Replaces the multi-publisher table from @p entries.
   *
   * @param entries Publisher rows (id, channel, type, rate, JSON, publishing).
   * @see publishers()
   */
  void setPublishers(const QVector<PublishEntry>& entries);

  /**
   * @brief Returns the current multi-publisher rows (JSON synced from subtrees).
   *
   * @return Copy of @c publisher_entries_ with up-to-date expression JSON.
   */
  QVector<PublishEntry> publishers() const;

  /**
   * @brief Index of the currently selected publisher root row.
   *
   * @return Index into @c publisher_entries_, or @c -1 if none.
   */
  int selectedPublisherIndex() const;

  /**
   * @brief Stable id of the selected publisher, or empty if none.
   *
   * @return @ref PublishEntry::id for the selection.
   */
  QString selectedPublisherId() const;

  /**
   * @brief Programmatically selects the publisher root at @p index.
   *
   * @param index Index into the publishers vector (−1 clears selection).
   */
  void selectPublisher(int index);

  /**
   * @brief Whether the current selection is exactly the publisher root at
   *        @p index (not a nested field).
   *
   * @param index Publisher index to test.
   * @return @c true when the root item for @p index is current.
   */
  bool isPublisherRootSelected(int index) const;

  /**
   * @brief Sets the publishing checkbox / state for publisher @p index.
   *
   * @param index Publisher row index.
   * @param publishing When @c true, that entry may be loop-published.
   */
  void setPublisherPublishing(int index, bool publishing);

  /**
   * @brief Returns the expression JSON for publisher @p index.
   *
   * @param index Publisher row index.
   * @return JSON string, or empty if out of range.
   */
  QString publisherJsonAt(int index) const;

  /**
   * @brief Column index used for editable values in the current mode.
   *
   * @return Value column for fields editor, or expression column for rqt mode.
   */
  int editorValueColumn() const;

 public slots:
  /** @brief Expands every tree node. */
  void expandAllFields();

  /** @brief Collapses every tree node. */
  void collapseAllFields();

  /**
   * @brief Adds an element to the array under the current / context item.
   *
   * @see removeArrayElement()
   */
  void addArrayElement();

  /**
   * @brief Removes the selected repeated-field element.
   *
   * @see addArrayElement()
   */
  void removeArrayElement();

 signals:
  /**
   * @brief Emitted when any field value changes in single-message mode.
   */
  void messageEdited();

  /**
   * @brief Emitted when publisher @p index expression / fields change.
   *
   * @param index Publisher row that was edited.
   */
  void publisherEdited(int index);

  /**
   * @brief Emitted when the publishing flag for @p index changes.
   *
   * @param index Publisher row.
   * @param publishing New publishing state.
   */
  void publisherPublishingChanged(int index, bool publishing);

  /**
   * @brief Emitted when the rate cell for @p index is edited.
   *
   * @param index Publisher row.
   * @param rate_hz New publish rate in Hz.
   */
  void publisherRateChanged(int index, double rate_hz);

  /**
   * @brief Emitted when the selected publisher root changes.
   *
   * @param index New selected index, or @c -1 if cleared.
   */
  void publisherSelectionChanged(int index);

 private slots:
  /**
   * @brief Applies cell edits to the underlying protobuf / publisher entry.
   *
   * @param item Changed tree item.
   * @param column Edited column index.
   */
  void onItemChanged(QTreeWidgetItem* item, int column);

  /**
   * @brief Tracks selection and emits @ref publisherSelectionChanged().
   *
   * @param current Newly current item.
   * @param previous Previously current item.
   */
  void onCurrentItemChanged(QTreeWidgetItem* current, QTreeWidgetItem* previous);

  /**
   * @brief Shows the context menu for repeated-field add/remove/duplicate.
   *
   * @param pos Local position of the right-click.
   */
  void showContextMenu(const QPoint& pos);

  /** @brief Context-menu: append an element to the remembered array item. */
  void addRepeatedElement();

  /** @brief Context-menu: remove the remembered array element. */
  void removeRepeatedElement();

  /** @brief Context-menu: remove the currently selected array element. */
  void removeSelectedElement();

  /** @brief Context-menu: duplicate the remembered array element. */
  void duplicateSelectedElement();

 private:
  /** @brief Rebuilds the tree for the active @ref DisplayMode. */
  void rebuildTree();

  /** @brief Rebuilds the multi-publisher root rows and field subtrees. */
  void rebuildPublishersTree();

  /** @brief Rebuilds the single-message Name|Type|Value tree. */
  void rebuildSingleMessageTree();

  /**
   * @brief Resolves the protobuf message owning @p item's field path.
   *
   * @param item Tree item under a message root.
   * @return Non-owning message pointer, or @c nullptr.
   */
  google::protobuf::Message* messageForItem(QTreeWidgetItem* item) const;

  /**
   * @brief Maps a tree item up to its publisher root index.
   *
   * @param item Any item under a publisher row.
   * @return Publisher index, or @c -1 if not under a publisher.
   */
  int publisherIndexForItem(QTreeWidgetItem* item) const;

  /**
   * @brief Returns the top-level publisher root item for @p index.
   *
   * @param index Publisher row index.
   * @return Root item, or @c nullptr if out of range.
   */
  QTreeWidgetItem* publisherRootItem(int index) const;

  /**
   * @brief Finds the publisher root by walking parents / scanning tops.
   *
   * @param index Publisher row index.
   * @return Matching root item, or @c nullptr.
   */
  QTreeWidgetItem* findPublisherRootItem(int index) const;

  /**
   * @brief Writes the subtree for publisher @p index back into entry JSON.
   *
   * @param index Publisher row index.
   */
  void syncPublisherJson(int index);

  /**
   * @brief Column index of the expression / JSON cell in rqt mode.
   *
   * @return Expression column index.
   */
  int expressionColumn() const;

  /**
   * @brief Finds an array (repeated) item at viewport @p pos.
   *
   * @param pos Local widget coordinates.
   * @return Array item, or @c nullptr.
   */
  QTreeWidgetItem* arrayItemAt(const QPoint& pos) const;

  /**
   * @brief Finds a repeated-element item at viewport @p pos.
   *
   * @param pos Local widget coordinates.
   * @return Element item, or @c nullptr.
   */
  QTreeWidgetItem* arrayElementItemAt(const QPoint& pos) const;

  /**
   * @brief Appends a repeated element at @p array_path on @p message.
   *
   * @param array_path Dot path to the repeated field.
   * @param message Message to mutate.
   * @return @c true on success.
   */
  bool addRepeatedElementAtPath(const QString& array_path,
                                google::protobuf::Message* message);

  /**
   * @brief Removes the repeated element at @p element_path on @p message.
   *
   * @param element_path Dot path including element index.
   * @param message Message to mutate.
   * @return @c true on success.
   */
  bool removeRepeatedElementAtPath(const QString& element_path,
                                   google::protobuf::Message* message);

  /**
   * @brief Duplicates the repeated element at @p element_path on @p message.
   *
   * @param element_path Dot path including element index.
   * @param message Message to mutate.
   * @return @c true on success.
   */
  bool duplicateRepeatedElementAtPath(const QString& element_path,
                                      google::protobuf::Message* message);

  /**
   * @brief Rebuilds only the field subtree under publisher @p index.
   *
   * @param index Publisher row index.
   */
  void rebuildPublisherSubtree(int index);

  /**
   * @brief Applies read-only flags recursively starting at @p item.
   *
   * @param item Subtree root.
   */
  void applyReadOnlyFlags(QTreeWidgetItem* item);

  /** @brief Applies read-only flags to the entire tree. */
  void applyReadOnlyFlags();

  /** Current layout: fields editor vs rqt publishers table. */
  DisplayMode display_mode_ = DisplayMode::kFieldsEditor;

  /** When @c true, value cells are not user-editable. */
  bool read_only_ = false;

  /** Owned single-message protobuf for fields-editor mode. */
  std::unique_ptr<google::protobuf::Message> message_;

  /** Type name for @c message_ (fields-editor mode). */
  std::string message_type_;

  /** Multi-publisher row data (rqt mode). */
  QVector<PublishEntry> publisher_entries_;

  /** Owned protobuf instances parallel to @c publisher_entries_. */
  std::vector<std::unique_ptr<google::protobuf::Message>> publisher_messages_;

  /**
   * @brief Re-entrancy guard while programmatically updating cells.
   *
   * Prevents @ref onItemChanged() from writing back during rebuilds.
   */
  bool suppress_updates_ = false;

  /** Context-menu target: repeated field (array) item. */
  QTreeWidgetItem* context_array_item_ = nullptr;

  /** Context-menu target: repeated element item. */
  QTreeWidgetItem* context_element_item_ = nullptr;

  /** Context-menu target: protobuf message owning the array path. */
  google::protobuf::Message* context_message_ = nullptr;
};

}  // namespace publish_panel
}  // namespace autoviz
