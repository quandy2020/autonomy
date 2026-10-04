/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/map/map_tile_cache.hpp"

#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QStandardPaths>

#include <algorithm>

namespace autoviz {
namespace map {

MapTileCache::MapTileCache(QObject* parent) : QObject(parent) {
  network_ = new QNetworkAccessManager(this);
}

MapTileCache::~MapTileCache() { clear(); }

void MapTileCache::setUserAgent(const QString& user_agent) {
  user_agent_ = user_agent;
}

void MapTileCache::setMaxCacheEntries(int max_entries) {
  max_cache_entries_ = std::max(32, max_entries);
  evictIfNeeded();
}

QImage MapTileCache::cachedTile(const MapTileCoord& coord) const {
  return cache_.value(coord);
}

bool MapTileCache::isUnavailable(const MapTileCoord& coord) const {
  return unavailable_.contains(coord);
}

QString MapTileCache::buildTileUrl(const QString& url_template,
                                   const MapTileCoord& coord) const {
  QString url = url_template;
  url.replace(QStringLiteral("{z}"), QString::number(coord.z));
  url.replace(QStringLiteral("{x}"), QString::number(coord.x));
  url.replace(QStringLiteral("{y}"), QString::number(coord.y));
  return url;
}

void MapTileCache::requestTiles(const QString& url_template,
                                const QSet<MapTileCoord>& coords) {
  if (url_template.trimmed().isEmpty() || network_ == nullptr) {
    return;
  }
  for (const MapTileCoord& coord : coords) {
    if (cache_.contains(coord) || pending_.contains(coord) ||
        unavailable_.contains(coord)) {
      continue;
    }
    QImage disk_image;
    if (loadDiskTile(url_template, coord, &disk_image)) {
      insertCacheEntry(coord, disk_image);
      emit tileReady(coord);
      continue;
    }
    pending_.insert(coord);
    QNetworkRequest request(buildTileUrl(url_template, coord));
    request.setAttribute(QNetworkRequest::CacheLoadControlAttribute,
                         QNetworkRequest::PreferCache);
    if (!user_agent_.isEmpty()) {
      request.setHeader(QNetworkRequest::UserAgentHeader, user_agent_);
    }
    QNetworkReply* reply = network_->get(request);
    reply->setProperty("tileUrlTemplate", url_template);
    active_replies_.insert(reply, coord);
    connect(reply, &QNetworkReply::finished, this, &MapTileCache::onTileFinished);
  }
}

void MapTileCache::clear() {
  // Copy keys first: abort() can emit finished synchronously, and that slot
  // mutates active_replies_. Disconnect before abort so onTileFinished does not
  // double-delete the same reply.
  const QList<QNetworkReply*> replies = active_replies_.keys();
  active_replies_.clear();
  pending_.clear();
  unavailable_.clear();
  cache_.clear();
  lru_order_.clear();
  for (QNetworkReply* reply : replies) {
    if (reply == nullptr) {
      continue;
    }
    QObject::disconnect(reply, nullptr, this, nullptr);
    reply->abort();
    reply->deleteLater();
  }
}

void MapTileCache::insertCacheEntry(const MapTileCoord& coord, QImage image) {
  if (image.isNull()) {
    return;
  }
  cache_.insert(coord, image);
  lru_order_.remove(coord);
  lru_order_.push_front(coord);
  evictIfNeeded();
}

void MapTileCache::evictIfNeeded() {
  while (cache_.size() > static_cast<std::size_t>(max_cache_entries_) &&
         !lru_order_.empty()) {
    const MapTileCoord oldest = lru_order_.back();
    lru_order_.pop_back();
    cache_.remove(oldest);
  }
}

void MapTileCache::onTileFinished() {
  auto* reply = qobject_cast<QNetworkReply*>(sender());
  if (reply == nullptr) {
    return;
  }
  if (!active_replies_.contains(reply)) {
    reply->deleteLater();
    return;
  }
  const MapTileCoord coord = active_replies_.take(reply);
  pending_.remove(coord);
  reply->deleteLater();

  if (reply->error() != QNetworkReply::NoError) {
    const int status =
        reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt();
    if (status == 404 || status == 204) {
      unavailable_.insert(coord);
      emit tileUnavailable(coord);
    }
    return;
  }
  QImage image;
  if (!image.loadFromData(reply->readAll())) {
    return;
  }
  insertCacheEntry(coord, image);
  const QString url_template = reply->property("tileUrlTemplate").toString();
  saveDiskTile(url_template, coord, image);
  emit tileReady(coord);
}

QString MapTileCache::diskPath(const QString& url_template,
                               const MapTileCoord& coord) const {
  const QByteArray hash = QCryptographicHash::hash(url_template.toUtf8(),
                                                   QCryptographicHash::Sha1)
                              .toHex();
  const QString root =
      QStandardPaths::writableLocation(QStandardPaths::CacheLocation) +
      QStringLiteral("/autoviz/map-tiles/") + QString::fromLatin1(hash);
  return root + QStringLiteral("/%1/%2/%3.png")
                    .arg(coord.z)
                    .arg(coord.x)
                    .arg(coord.y);
}

bool MapTileCache::loadDiskTile(const QString& url_template,
                                const MapTileCoord& coord, QImage* image) const {
  if (image == nullptr || url_template.isEmpty()) {
    return false;
  }
  const QString path = diskPath(url_template, coord);
  if (!QFile::exists(path)) {
    return false;
  }
  return image->load(path);
}

void MapTileCache::saveDiskTile(const QString& url_template,
                                const MapTileCoord& coord,
                                const QImage& image) const {
  if (url_template.isEmpty() || image.isNull()) {
    return;
  }
  const QString path = diskPath(url_template, coord);
  QDir().mkpath(QFileInfo(path).absolutePath());
  image.save(path, "PNG");
}

}  // namespace map
}  // namespace autoviz
