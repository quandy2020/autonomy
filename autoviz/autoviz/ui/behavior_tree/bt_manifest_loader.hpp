/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include "autoviz/ui/behavior_tree/bt_node_models.hpp"

#include <QString>
#include <QStringList>

#include <optional>

namespace autoviz {
namespace behavior_tree {

std::optional<QString> DiscoverConfigRoot();

QStringList ListBtXmlFiles(const QString& config_bt_root);

bool LoadTreeNodesModelFile(const QString& path, QHash<QString, BtNodeModel>& models);

QHash<QString, BtNodeModel> LoadDefaultManifests();

}  // namespace behavior_tree
}  // namespace autoviz
