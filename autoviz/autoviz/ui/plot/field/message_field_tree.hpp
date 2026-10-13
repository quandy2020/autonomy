/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file message_field_tree.hpp
 * @brief Build Topics-panel field trees and list numeric paths for Plot pickers.
 *
 * Populates @c QTreeWidgetItem hierarchies with drag roles used by Plot,
 * Table, Image, and Map drop targets. Also enumerates field paths for value
 * editors.
 *
 * @see plot_drag_mime.hpp
 * @see PlotPanel
 * @see message_path_navigation.hpp
 */

#pragma once

#include <Qt>
#include <QStringList>

#include <string>

class QTreeWidgetItem;

namespace autoviz {
namespace plot {

/**
 * @brief Qt::ItemDataRole storing the channel name on a topic tree node.
 */
constexpr int kTopicChannelRole = Qt::UserRole + 10;

/**
 * @brief Qt::ItemDataRole storing a protobuf field path on a leaf / branch.
 */
constexpr int kTopicFieldPathRole = Qt::UserRole + 11;

/**
 * @brief Qt::ItemDataRole: item is draggable as a plot series field.
 */
constexpr int kTopicDraggableRole = Qt::UserRole + 12;

/**
 * @brief Qt::ItemDataRole: item is draggable onto Table panels.
 */
constexpr int kTopicTableDraggableRole = Qt::UserRole + 13;

/**
 * @brief Channel node can be dropped on Raw Messages and similar panels.
 */
constexpr int kTopicChannelDraggableRole = Qt::UserRole + 14;

/**
 * @brief Channel node carries an image-compatible message type.
 */
constexpr int kTopicImageDraggableRole = Qt::UserRole + 15;

/**
 * @brief Channel node carries geo/map-compatible message type.
 */
constexpr int kTopicMapDraggableRole = Qt::UserRole + 16;

/**
 * @brief Populate numeric protobuf field leaves under @p parent for drag-to-plot.
 *
 * Sets @ref kTopicFieldPathRole / @ref kTopicDraggableRole on numeric leaves.
 *
 * @param parent Parent tree item (typically a channel node).
 * @param message_type Fully-qualified protobuf message type.
 * @param path_prefix Field path prefix already walked (empty at root).
 */
void PopulateMessageFieldTree(QTreeWidgetItem* parent,
                              const std::string& message_type,
                              const QString& path_prefix);

/**
 * @brief Mark repeated array fields as draggable for Table panels.
 *
 * @param parent Parent tree item.
 * @param message_type Fully-qualified protobuf message type.
 * @param path_prefix Field path prefix already walked.
 */
void PopulateTableArrayFieldTree(QTreeWidgetItem* parent,
                                 const std::string& message_type,
                                 const QString& path_prefix);

/**
 * @brief Numeric protobuf leaf paths for plot Y/X field pickers.
 *
 * @param message_type Fully-qualified protobuf message type.
 * @return Dotted field paths that resolve to numeric scalars.
 */
QStringList NumericFieldPathsForMessageType(const std::string& message_type);

/**
 * @brief Repeated (array) protobuf field paths suitable for the Table panel.
 *
 * @param message_type Fully-qualified protobuf message type.
 * @return Dotted paths to repeated fields (depth-first).
 */
QStringList TableArrayFieldPathsForMessageType(const std::string& message_type);

/**
 * @brief All protobuf field paths for plot browsing (includes message prefixes).
 *
 * @param message_type Fully-qualified protobuf message type.
 * @return Paths suitable for Foxglove-style value browsing.
 */
QStringList PlotBrowsePathsForMessageType(const std::string& message_type);

/**
 * @brief Every nested field path relative to the message root (includes repeated
 *        branches).
 *
 * @param message_type Fully-qualified protobuf message type.
 * @return Exhaustive field path list.
 */
QStringList PlotAllFieldPathsForMessageType(const std::string& message_type);

/**
 * @brief Immediate protobuf field name segments under @p parent_field_path.
 *
 * Example: parent @c "linear" → @c x, @c y, @c z.
 *
 * @param message_type Fully-qualified protobuf message type.
 * @param parent_field_path Dotted path to the parent message / field.
 * @return Child field name segments (not full paths).
 */
QStringList PlotNextLevelFieldPaths(const std::string& message_type,
                                    const QString& parent_field_path);

}  // namespace plot
}  // namespace autoviz
