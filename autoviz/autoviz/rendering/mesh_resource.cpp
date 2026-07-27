/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/mesh_resource.hpp"

#ifdef AUTOVIZ_USE_OGRE

#include <QEventLoop>
#include <QFile>
#include <QFileInfo>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QObject>
#include <QString>
#include <QUrl>

namespace autoviz {
namespace rendering {

MeshResourceResolver& MeshResourceResolver::instance() {
  static MeshResourceResolver resolver;
  return resolver;
}

void MeshResourceResolver::addPackageSharePath(const std::string& package_name,
                                               const std::string& share_root) {
  package_share_paths_[package_name] = share_root;
}

void MeshResourceResolver::clearPackagePaths() { package_share_paths_.clear(); }

std::optional<std::string> MeshResourceResolver::resolveToPath(
    const std::string& uri) const {
  if (uri.empty()) {
    return std::nullopt;
  }
  QString quri = QString::fromStdString(uri);
  if (quri.startsWith(QLatin1String("file://"))) {
    return quri.mid(7).toStdString();
  }
  if (quri.startsWith(QLatin1String("package://"))) {
    const QString rest = quri.mid(10);
    const int slash = rest.indexOf(QLatin1Char('/'));
    if (slash <= 0) {
      return std::nullopt;
    }
    const std::string package = rest.left(slash).toStdString();
    const std::string subpath = rest.mid(slash + 1).toStdString();
    const auto it = package_share_paths_.find(package);
    if (it == package_share_paths_.end()) {
      return std::nullopt;
    }
    return (QString::fromStdString(it->second) + QLatin1Char('/') +
            QString::fromStdString(subpath))
        .toStdString();
  }
  if (QFileInfo(quri).isAbsolute()) {
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
  return resolveToPath(uri);
}

std::shared_ptr<MeshResource> MeshResourceResolver::fetch(
    const std::string& uri) const {
  const QString quri = QString::fromStdString(uri);
  if (quri.startsWith(QLatin1String("http://")) ||
      quri.startsWith(QLatin1String("https://"))) {
    return fetchHttp(uri);
  }
  const auto path = resolveToPath(uri);
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
  resource->resolved_uri = uri;
  return resource;
}

}  // namespace rendering
}  // namespace autoviz

#endif
