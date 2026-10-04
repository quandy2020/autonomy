/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/ogre_resource_config.hpp"

#include <QByteArray>
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QProcessEnvironment>
#include <QString>

namespace autoviz {
namespace rendering {
namespace {

std::string g_resource_directory;
std::string g_plugin_directory;

QStringList SplitEnvPath(const QString& value) {
#if defined(Q_OS_WIN)
  return value.split(QLatin1Char(';'), Qt::SkipEmptyParts);
#else
  return value.split(QLatin1Char(':'), Qt::SkipEmptyParts);
#endif
}

QString FirstExistingDir(const QStringList& candidates) {
  for (const QString& candidate : candidates) {
    if (QFileInfo(candidate).isDir()) {
      return QFileInfo(candidate).absoluteFilePath();
    }
  }
  return QString();
}

QString DefaultResourceDirectory() {
  const QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
  for (const QString& part :
       SplitEnvPath(env.value(QStringLiteral("AUTOVIZ_OGRE_MEDIA_PATH")))) {
    if (QFileInfo(part).isDir()) {
      return QFileInfo(part).absoluteFilePath();
    }
  }

#ifdef AUTOVIZ_OGRE_MEDIA_DIR
  if (QFileInfo(QStringLiteral(AUTOVIZ_OGRE_MEDIA_DIR)).isDir()) {
    return QFileInfo(QStringLiteral(AUTOVIZ_OGRE_MEDIA_DIR)).absoluteFilePath();
  }
#endif

  const QString app_dir = QCoreApplication::applicationDirPath();
  const QStringList candidates = {
      app_dir + QStringLiteral("/../share/autonomy/autoviz/ogre_media"),
      app_dir + QStringLiteral("/../../autoviz/resources/ogre_media"),
      app_dir + QStringLiteral("/../../../autoviz/resources/ogre_media"),
  };
  return FirstExistingDir(candidates);
}

QString DefaultPluginDirectory() {
  const QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
  const QString from_env = env.value(QStringLiteral("AUTOVIZ_OGRE_PLUGIN_DIR"));
  if (!from_env.isEmpty() && QFileInfo(from_env).isDir()) {
    return QFileInfo(from_env).absoluteFilePath();
  }

#ifdef AUTOVIZ_OGRE_PLUGIN_DIR
  if (QFileInfo(QStringLiteral(AUTOVIZ_OGRE_PLUGIN_DIR)).isDir()) {
    return QFileInfo(QStringLiteral(AUTOVIZ_OGRE_PLUGIN_DIR)).absoluteFilePath();
  }
#endif

  const QString app_dir = QCoreApplication::applicationDirPath();
  const QStringList candidates = {
      // Build-tree / bundle layouts next to the binary.
      app_dir + QStringLiteral("/../lib/macosx"),
      app_dir + QStringLiteral("/../lib/OGRE"),
      app_dir + QStringLiteral("/../lib"),
      QStringLiteral("/usr/local/lib/OGRE"),
      QStringLiteral("/usr/lib/x86_64-linux-gnu/OGRE"),
      QStringLiteral("/usr/lib/aarch64-linux-gnu/OGRE"),
      QStringLiteral("/usr/lib/OGRE"),
  };
  return FirstExistingDir(candidates);
}

}  // namespace

std::string ogreResourceDirectory() {
  if (g_resource_directory.empty()) {
    g_resource_directory = DefaultResourceDirectory().toStdString();
  }
  return g_resource_directory;
}

std::string ogrePluginDirectory() {
  if (g_plugin_directory.empty()) {
    g_plugin_directory = DefaultPluginDirectory().toStdString();
  }
  return g_plugin_directory;
}

void setOgreResourceDirectory(const std::string& path) {
  g_resource_directory = path;
}

void setOgrePluginDirectory(const std::string& path) {
  g_plugin_directory = path;
}

std::string loadOgreMediaText(const std::string& relative_path) {
  const QString root = QString::fromStdString(ogreResourceDirectory());
  if (root.isEmpty() || relative_path.empty()) {
    return {};
  }
  const QString path =
      QDir(root).filePath(QString::fromStdString(relative_path));
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return {};
  }
  return QString::fromUtf8(file.readAll()).toStdString();
}

}  // namespace rendering
}  // namespace autoviz

