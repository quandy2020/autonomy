/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QHash>
#include <QImage>
#include <QObject>
#include <QSet>
#include <QString>

#include <list>

#include "autoviz/ui/map/map_projection.hpp"

class QNetworkAccessManager;
class QNetworkReply;

namespace autoviz {
namespace map {

/** LRU tile cache with asynchronous HTTP fetch for map basemap layers. */
class MapTileCache : public QObject {
  Q_OBJECT

 public:
  explicit MapTileCache(QObject* parent = nullptr);
  ~MapTileCache() override;

  void setUserAgent(const QString& user_agent);
  void setMaxCacheEntries(int max_entries);

  QImage cachedTile(const MapTileCoord& coord) const;
  void requestTiles(const QString& url_template, const QSet<MapTileCoord>& coords);
  void clear();

 signals:
  void tileReady(const MapTileCoord& coord);

 private slots:
  void onTileFinished();

 private:
  struct CacheEntry {
    MapTileCoord coord;
    QImage image;
  };

  QString buildTileUrl(const QString& url_template, const MapTileCoord& coord) const;
  void insertCacheEntry(const MapTileCoord& coord, QImage image);
  void evictIfNeeded();

  QNetworkAccessManager* network_ = nullptr;
  QString user_agent_;
  int max_cache_entries_ = 512;
  QHash<MapTileCoord, QImage> cache_;
  std::list<MapTileCoord> lru_order_;
  QSet<MapTileCoord> pending_;
  QHash<QNetworkReply*, MapTileCoord> active_replies_;
};

}  // namespace map
}  // namespace autoviz
