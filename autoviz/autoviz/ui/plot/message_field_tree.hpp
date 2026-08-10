/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <Qt>
#include <QStringList>

#include <string>

class QTreeWidgetItem;

namespace autoviz {
namespace plot {

constexpr int kTopicChannelRole = Qt::UserRole + 10;
constexpr int kTopicFieldPathRole = Qt::UserRole + 11;
constexpr int kTopicDraggableRole = Qt::UserRole + 12;

constexpr int kTopicTableDraggableRole = Qt::UserRole + 13;

/** Channel node can be dropped on Raw Messages and similar panels. */
constexpr int kTopicChannelDraggableRole = Qt::UserRole + 14;

/** Channel node carries an image-compatible message type. */
constexpr int kTopicImageDraggableRole = Qt::UserRole + 15;

/** Channel node carries geo/map-compatible message type. */
constexpr int kTopicMapDraggableRole = Qt::UserRole + 16;

/** Populate numeric protobuf field leaves under parent for drag-to-plot. */
void PopulateMessageFieldTree(QTreeWidgetItem* parent,
                              const std::string& message_type,
                              const QString& path_prefix);

/** Mark repeated array fields as draggable for Table panels. */
void PopulateTableArrayFieldTree(QTreeWidgetItem* parent,
                                 const std::string& message_type,
                                 const QString& path_prefix);

/** Numeric protobuf leaf paths for plot Y/X field pickers. */
QStringList NumericFieldPathsForMessageType(const std::string& message_type);

/** All protobuf field paths for plot browsing (includes message prefixes). */
QStringList PlotBrowsePathsForMessageType(const std::string& message_type);

/** Every nested field path relative to the message root (includes repeated branches). */
QStringList PlotAllFieldPathsForMessageType(const std::string& message_type);

/** Immediate protobuf field name segments under parent_field_path (e.g. "linear" → x,y,z). */
QStringList PlotNextLevelFieldPaths(const std::string& message_type,
                                    const QString& parent_field_path);

}  // namespace plot
}  // namespace autoviz
