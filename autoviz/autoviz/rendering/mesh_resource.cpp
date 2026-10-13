/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/mesh_resource.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QEventLoop>
#include <QFile>
#include <QFileInfo>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QObject>
#include <QProcessEnvironment>
#include <QRegularExpression>
#include <QString>
#include <QUrl>

#include "autoviz/common/path_env_utils.hpp"

namespace autoviz {
namespace rendering {
namespace {

/**
 * @brief Rewrites @c file://$(find pkg)/rel to @c package://pkg/rel (RViz / ROS).
 */
QString NormalizeFindUri(QString uri) {
  // file://$(find ocs2_robotic_assets)/resources/foo.obj
  static const QRegularExpression kFind(
      QStringLiteral(R"(^file://\$\(find\s+([^)]+)\)/?(.*)$)"));
  const QRegularExpressionMatch match = kFind.match(uri);
  if (match.hasMatch()) {
    const QString pkg = match.captured(1).trimmed();
    const QString rest = match.captured(2);
    return QStringLiteral("package://%1/%2").arg(pkg, rest);
  }
  return uri;
}

QStringList CandidateAssetRoots() {
  QStringList roots;
  const QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
  for (const char* key : {"BOLZA_ROBOTIC_ASSETS", "AUTOMANIP_ROBOTIC_ASSETS",
                          "OCS2_ROBOTIC_ASSETS"}) {
    const QString value = env.value(QString::fromLatin1(key));
    if (!value.isEmpty() && QFileInfo(value).isDir()) {
      roots.push_back(QFileInfo(value).absoluteFilePath());
    }
  }
  for (const QString& part :
       common::splitPathList(env.value(QStringLiteral("AUTOVIZ_RESOURCE_PATH")))) {
    if (!part.isEmpty() && QFileInfo(part).isDir()) {
      const QString abs = QFileInfo(part).absoluteFilePath();
      if (!roots.contains(abs)) {
        roots.push_back(abs);
      }
    }
  }

  const QString app = QCoreApplication::applicationDirPath();
  const QStringList relative = {
      app + QStringLiteral("/../share/ocs2_robotic_assets/resources"),
      app + QStringLiteral("/../../share/ocs2_robotic_assets/resources"),
      // Source-tree / docker bind-mount layouts (build/.../bin and install/.../bin).
      app + QStringLiteral(
                "/../../src/autonomy/automanip/bolza/examples/assets"),
      app + QStringLiteral(
                "/../../../src/autonomy/automanip/bolza/examples/assets"),
      app + QStringLiteral(
                "/../../../../src/autonomy/automanip/bolza/examples/assets"),
  };
  for (const QString& candidate : relative) {
    const QFileInfo info(candidate);
    if (info.isDir()) {
      const QString abs = info.absoluteFilePath();
      if (!roots.contains(abs)) {
        roots.push_back(abs);
      }
    }
  }
  return roots;
}

}  // namespace

MeshResourceResolver& MeshResourceResolver::instance() {
  static MeshResourceResolver resolver;
  return resolver;
}

void MeshResourceResolver::addPackageSharePath(const std::string& package_name,
                                               const std::string& share_root) {
  package_share_paths_[package_name] = share_root;
}

void MeshResourceResolver::addPackageContentRoot(
    const std::string& package_name, const std::string& content_root,
    const std::string& uri_prefix_to_strip) {
  PackageContentRoot entry;
  entry.content_root = content_root;
  entry.uri_prefix_to_strip = uri_prefix_to_strip;
  package_content_roots_[package_name] = std::move(entry);
}

void MeshResourceResolver::clearPackagePaths() {
  package_share_paths_.clear();
  package_content_roots_.clear();
  defaults_registered_ = false;
}

void MeshResourceResolver::ensureDefaultPackagesRegistered() {
  if (defaults_registered_) {
    return;
  }
  defaults_registered_ = true;

  const QString app = QCoreApplication::applicationDirPath();
  const QString installed_share =
      QFileInfo(app + QStringLiteral("/../share/ocs2_robotic_assets"))
          .absoluteFilePath();
  if (QFileInfo(installed_share).isDir()) {
    addPackageSharePath("ocs2_robotic_assets", installed_share.toStdString());
  }

  // Source / env assets live at …/assets/<robot>/… while URDF URIs use
  // package://ocs2_robotic_assets/resources/<robot>/… (RViz / OCS2 layout).
  auto looks_like_assets_root = [](const QString& root) {
    return QFileInfo(root + QStringLiteral("/quadrotor")).isDir() ||
           QFileInfo(root + QStringLiteral("/anymal_c")).isDir() ||
           QFileInfo(root + QStringLiteral("/mobile_manipulator")).isDir() ||
           QFileInfo(root + QStringLiteral("/ballbot")).isDir() ||
           QFileInfo(root + QStringLiteral("/cartpole")).isDir();
  };
  for (const QString& root : CandidateAssetRoots()) {
    if (looks_like_assets_root(root)) {
      addPackageContentRoot("ocs2_robotic_assets", root.toStdString(),
                            "resources/");
      break;
    }
    // Already the ament share root that contains resources/.
    if (QFileInfo(root + QStringLiteral("/resources")).isDir()) {
      const QString share = QFileInfo(root).absoluteFilePath();
      // root may be …/resources — use parent as share.
      if (QFileInfo(root).fileName() == QLatin1String("resources")) {
        addPackageSharePath(
            "ocs2_robotic_assets",
            QFileInfo(root + QStringLiteral("/..")).absoluteFilePath().toStdString());
      } else {
        addPackageSharePath("ocs2_robotic_assets", share.toStdString());
      }
      break;
    }
  }
}

std::optional<std::string> MeshResourceResolver::resolveToPath(
    const std::string& uri) const {
  if (uri.empty()) {
    return std::nullopt;
  }
  // Const-cast for lazy default registration (singleton mutates once).
  const_cast<MeshResourceResolver*>(this)->ensureDefaultPackagesRegistered();

  QString quri = NormalizeFindUri(QString::fromStdString(uri));
  if (quri.startsWith(QLatin1String("file://"))) {
    const QString path = quri.mid(7);
    if (QFileInfo(path).exists()) {
      return path.toStdString();
    }
    return path.toStdString();
  }
  if (quri.startsWith(QLatin1String("package://"))) {
    const QString rest = quri.mid(10);
    const int slash = rest.indexOf(QLatin1Char('/'));
    if (slash <= 0) {
      return std::nullopt;
    }
    const std::string package = rest.left(slash).toStdString();
    QString subpath = rest.mid(slash + 1);

    // 1) Explicit content-root mapping (source-tree assets).
    const auto content_it = package_content_roots_.find(package);
    if (content_it != package_content_roots_.end()) {
      QString relative = subpath;
      const QString prefix =
          QString::fromStdString(content_it->second.uri_prefix_to_strip);
      if (!prefix.isEmpty() && relative.startsWith(prefix)) {
        relative = relative.mid(prefix.size());
      }
      const QString candidate =
          QDir(QString::fromStdString(content_it->second.content_root))
              .filePath(relative);
      if (QFileInfo(candidate).exists()) {
        return QFileInfo(candidate).absoluteFilePath().toStdString();
      }
    }

    // 2) Registered ament-style share/<pkg>/<subpath>.
    const auto share_it = package_share_paths_.find(package);
    if (share_it != package_share_paths_.end()) {
      const QString candidate =
          QDir(QString::fromStdString(share_it->second)).filePath(subpath);
      if (QFileInfo(candidate).exists()) {
        return QFileInfo(candidate).absoluteFilePath().toStdString();
      }
    }

    // 3) AUTOVIZ_RESOURCE_PATH / install prefixes (share/pkg/…).
    for (const QString& prefix : common::resourceSearchPaths()) {
      const QString candidate =
          QDir(prefix).filePath(QStringLiteral("share/%1/%2")
                                    .arg(QString::fromStdString(package),
                                         subpath));
      if (QFileInfo(candidate).exists()) {
        return QFileInfo(candidate).absoluteFilePath().toStdString();
      }
      // Prefix may already be the package share directory.
      const QString direct = QDir(prefix).filePath(subpath);
      if (QFileInfo(direct).exists()) {
        return QFileInfo(direct).absoluteFilePath().toStdString();
      }
    }

    return std::nullopt;
  }
  // Absolute filesystem paths (after demo_loop rewrites package:// → abs).
  if (QFileInfo(quri).isAbsolute()) {
    if (QFileInfo(quri).exists()) {
      return QFileInfo(quri).absoluteFilePath().toStdString();
    }
    return uri;
  }
  return std::nullopt;
}

std::shared_ptr<MeshResource> MeshResourceResolver::fetchHttp(
    const std::string& uri) const {
  QNetworkAccessManager manager;
  QEventLoop loop;
  QNetworkRequest request(QUrl(QString::fromStdString(uri)));
  QNetworkReply* reply = manager.get(request);
  QObject::connect(reply, &QNetworkReply::finished, &loop, &QEventLoop::quit);
  loop.exec();
  if (reply->error() != QNetworkReply::NoError) {
    const std::string err = reply->errorString().toStdString();
    reply->deleteLater();
    throw MeshResourceError("HTTP fetch failed for " + uri + ": " + err);
  }
  const QByteArray bytes = reply->readAll();
  reply->deleteLater();
  auto resource = std::make_shared<MeshResource>();
  resource->data.assign(bytes.begin(), bytes.end());
  resource->resolved_uri = uri;
  return resource;
}

bool MeshResourceResolver::exists(const std::string& uri) const {
  const auto path = resolveToPath(uri);
  return path.has_value() &&
         QFileInfo(QString::fromStdString(*path)).exists();
}

std::optional<std::string> MeshResourceResolver::resolvePath(
    const std::string& uri) const {
  const auto path = resolveToPath(uri);
  if (!path.has_value()) {
    return std::nullopt;
  }
  if (!QFileInfo(QString::fromStdString(*path)).exists()) {
    return std::nullopt;
  }
  return path;
}

std::shared_ptr<MeshResource> MeshResourceResolver::fetch(
    const std::string& uri) const {
  const QString quri = QString::fromStdString(uri);
  if (quri.startsWith(QLatin1String("http://")) ||
      quri.startsWith(QLatin1String("https://"))) {
    return fetchHttp(uri);
  }
  const auto path = resolvePath(uri);
  if (!path.has_value()) {
    throw MeshResourceError("Could not resolve resource URI: " + uri);
  }
  QFile file(QString::fromStdString(*path));
  if (!file.open(QIODevice::ReadOnly)) {
    throw MeshResourceError("Could not open resource: " + uri);
  }
  const QByteArray bytes = file.readAll();
  auto resource = std::make_shared<MeshResource>();
  resource->data.assign(bytes.begin(), bytes.end());
  resource->resolved_uri = *path;
  return resource;
}

}  // namespace rendering
}  // namespace autoviz
