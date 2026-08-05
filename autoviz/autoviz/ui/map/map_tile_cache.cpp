/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/map/map_tile_cache.hpp"

#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QNetworkRequest>

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
    if (cache_.contains(coord) || pending_.contains(coord)) {
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
    active_replies_.insert(reply, coord);
    connect(reply, &QNetworkReply::finished, this, &MapTileCache::onTileFinished);
  }
}

void MapTileCache::clear() {
  for (auto it = active_replies_.cbegin(); it != active_replies_.cend(); ++it) {
    if (it.key() != nullptr) {
      it.key()->abort();
      it.key()->deleteLater();
    }
  }
  active_replies_.clear();
  pending_.clear();
  cache_.clear();
  lru_order_.clear();
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
  const MapTileCoord coord = active_replies_.take(reply);
  pending_.remove(coord);
  reply->deleteLater();

  if (reply->error() != QNetworkReply::NoError) {
    return;
  }
  QImage image;
  if (!image.loadFromData(reply->readAll())) {
    return;
  }
  insertCacheEntry(coord, image);
  emit tileReady(coord);
}

}  // namespace map
}  // namespace autoviz
