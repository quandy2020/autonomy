/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include "autoviz/ui/behavior_tree/bt_node_models.hpp"

#include <QHash>
#include <QSizeF>
#include <QString>

#include <optional>

namespace autoviz {
namespace behavior_tree {

std::optional<BtDocument> LoadBtDocument(const QString& path);
bool SaveBtDocument(const BtDocument& doc, const QString& path);

std::optional<QHash<QString, BtNodeModel>> LoadTreeNodesModelFile(const QString& path);
/** Custom models only (no built-ins), for Groot-style import merge. */
std::optional<QHash<QString, BtNodeModel>> LoadImportedCustomModels(const QString& path);
bool SaveTreeNodesModelFile(const QHash<QString, BtNodeModel>& models, const QString& path);

void ApplyVerticalTreeLayout(BtAbsTree& tree,
                             const QHash<int, QSizeF>& node_sizes = {});
void ApplyHorizontalTreeLayout(BtAbsTree& tree,
                               const QHash<int, QSizeF>& node_sizes = {});

}  // namespace behavior_tree
}  // namespace autoviz
