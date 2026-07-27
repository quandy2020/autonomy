/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/path_env_utils.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QProcessEnvironment>

namespace autoviz {
namespace common {

QStringList splitPathList(const QString& value) {
  if (value.isEmpty()) {
    return {};
  }
#if defined(Q_OS_WIN)
  return value.split(QLatin1Char(';'), Qt::SkipEmptyParts);
#else
  return value.split(QLatin1Char(':'), Qt::SkipEmptyParts);
#endif
}

QStringList pluginSearchPaths() {
  const QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
  return splitPathList(env.value(QStringLiteral("AUTOVIZ_PLUGIN_PATH")));
}

QStringList resourceSearchPaths(const std::string& base_directory) {
  QStringList prefixes;
  const QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
  for (const QString& part :
       splitPathList(env.value(QStringLiteral("AUTOVIZ_RESOURCE_PATH")))) {
    if (!part.isEmpty() && !prefixes.contains(part)) {
      prefixes.push_back(part);
    }
  }

  const QString install_share =
      QCoreApplication::applicationDirPath() +
      QStringLiteral("/../share/autonomy");
  if (QFileInfo(install_share).isDir() && !prefixes.contains(install_share)) {
    prefixes.push_back(QFileInfo(install_share).absoluteFilePath());
  }

  if (!base_directory.empty()) {
    QDir dir(QString::fromStdString(base_directory));
    for (int depth = 0; depth < 6 && dir.exists(); ++depth) {
      const QString install_prefix = dir.absolutePath();
      if (QFileInfo(dir.filePath(QStringLiteral("share"))).isDir() &&
          !prefixes.contains(install_prefix)) {
        prefixes.push_back(install_prefix);
      }
      if (!dir.cdUp()) {
        break;
      }
    }
  }
  return prefixes;
}

}  // namespace common
}  // namespace autoviz
