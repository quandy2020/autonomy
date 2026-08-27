/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/behavior_tree/bt_manifest_loader.hpp"

#include "autoviz/common/path_env_utils.hpp"
#include "autoviz/ui/behavior_tree/bt_xml_io.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QDirIterator>
#include <QFileInfo>
#include <QProcessEnvironment>

namespace autoviz {
namespace behavior_tree {
namespace {

bool IsExcludedBtXml(const QFileInfo& info) {
  const QString base = info.fileName();
  if (base.compare(QStringLiteral("tree_nodes_model.xml"), Qt::CaseInsensitive) == 0) {
    return true;
  }
  if (base.startsWith(QStringLiteral("README"), Qt::CaseInsensitive)) {
    return true;
  }
  return false;
}

QString ResolveBehaviorTreeRoot(const QString& config_root) {
  const QString bt_root =
      QDir(config_root).filePath(QStringLiteral("task/behavior_tree"));
  if (QFileInfo(bt_root).isDir()) {
    return QFileInfo(bt_root).absoluteFilePath();
  }
  return QFileInfo(config_root).absoluteFilePath();
}

bool AppendCandidate(QStringList& candidates, const QString& path) {
  const QString absolute = QFileInfo(path).absoluteFilePath();
  if (absolute.isEmpty() || candidates.contains(absolute)) {
    return false;
  }
  if (!QFileInfo(absolute).isDir()) {
    return false;
  }
  candidates.push_back(absolute);
  return true;
}

}  // namespace

std::optional<QString> DiscoverConfigRoot() {
  QStringList candidates;

  const QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
  const QString env_config = env.value(QStringLiteral("AUTONOMY_CONFIG_DIR"));
  if (!env_config.isEmpty()) {
    AppendCandidate(candidates, env_config);
    AppendCandidate(candidates, ResolveBehaviorTreeRoot(env_config));
  }

  const QString app_dir = QCoreApplication::applicationDirPath();
  const QStringList relative_config_paths = {
      QStringLiteral("../config"),
      QStringLiteral("../../config"),
      QStringLiteral("../../../config"),
  };
  for (const QString& relative : relative_config_paths) {
    AppendCandidate(candidates, QDir(app_dir).filePath(relative));
    AppendCandidate(candidates,
                     ResolveBehaviorTreeRoot(QDir(app_dir).filePath(relative)));
  }

  AppendCandidate(candidates, QStringLiteral("/workspace/autonomy/src/autonomy/config"));
  AppendCandidate(candidates, ResolveBehaviorTreeRoot(
                                   QStringLiteral("/workspace/autonomy/src/autonomy/config")));

  for (const QString& prefix : common::resourceSearchPaths({})) {
    AppendCandidate(candidates, QDir(prefix).filePath(QStringLiteral("config")));
    AppendCandidate(candidates, ResolveBehaviorTreeRoot(QDir(prefix).filePath(QStringLiteral("config"))));
    AppendCandidate(candidates, QDir(prefix).filePath(QStringLiteral("share/autonomy/config")));
    AppendCandidate(candidates, ResolveBehaviorTreeRoot(
                                     QDir(prefix).filePath(QStringLiteral("share/autonomy/config"))));
  }

  for (const QString& candidate : candidates) {
    if (candidate.endsWith(QStringLiteral("/task/behavior_tree")) ||
        candidate.endsWith(QStringLiteral("\\task\\behavior_tree"))) {
      return candidate;
    }
  }

  for (const QString& candidate : candidates) {
    if (QFileInfo(QDir(candidate).filePath(QStringLiteral("task/behavior_tree"))).isDir()) {
      return ResolveBehaviorTreeRoot(candidate);
    }
  }

  if (!candidates.isEmpty()) {
    return candidates.first();
  }
  return std::nullopt;
}

QStringList ListBtXmlFiles(const QString& config_bt_root) {
  QStringList files;
  QDirIterator iterator(config_bt_root, {QStringLiteral("*.xml")},
                        QDir::Files, QDirIterator::Subdirectories);
  while (iterator.hasNext()) {
    const QFileInfo info(iterator.next());
    if (IsExcludedBtXml(info)) {
      continue;
    }
    files.push_back(info.absoluteFilePath());
  }
  files.sort();
  return files;
}

bool LoadTreeNodesModelFile(const QString& path, QHash<QString, BtNodeModel>& models) {
  const std::optional<BtDocument> doc = LoadBtDocument(path);
  if (!doc.has_value()) {
    return false;
  }
  for (auto it = doc->models.constBegin(); it != doc->models.constEnd(); ++it) {
    models.insert(it.key(), it.value());
  }
  return true;
}

QHash<QString, BtNodeModel> LoadDefaultManifests() {
  QHash<QString, BtNodeModel> models = BuiltinNodeModels();

  const std::optional<QString> config_root = DiscoverConfigRoot();
  if (!config_root.has_value()) {
    return models;
  }

  const QString model_path =
      QDir(config_root.value()).filePath(QStringLiteral("tree_nodes_model.xml"));
  if (QFileInfo(model_path).isFile()) {
    LoadTreeNodesModelFile(model_path, models);
  }

  return models;
}

}  // namespace behavior_tree
}  // namespace autoviz
