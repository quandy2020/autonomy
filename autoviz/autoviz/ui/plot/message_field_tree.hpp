/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <Qt>

#include <string>

class QTreeWidgetItem;

namespace autoviz {
namespace plot {

constexpr int kTopicChannelRole = Qt::UserRole + 10;
constexpr int kTopicFieldPathRole = Qt::UserRole + 11;
constexpr int kTopicDraggableRole = Qt::UserRole + 12;

constexpr int kTopicTableDraggableRole = Qt::UserRole + 13;

/** Populate numeric protobuf field leaves under parent for drag-to-plot. */
void PopulateMessageFieldTree(QTreeWidgetItem* parent,
                              const std::string& message_type,
                              const QString& path_prefix);

/** Mark repeated array fields as draggable for Table panels. */
void PopulateTableArrayFieldTree(QTreeWidgetItem* parent,
                                 const std::string& message_type,
                                 const QString& path_prefix);

}  // namespace plot
}  // namespace autoviz
