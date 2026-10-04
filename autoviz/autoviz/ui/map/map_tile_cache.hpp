/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file map_tile_cache.hpp
 * @brief LRU tile cache with asynchronous HTTP fetch for map basemap layers.
 *
 * Fetches slippy-map tiles from a URL template (`{z}/{x}/{y}`), stores decoded
 * images in an LRU cache, and emits @ref tileReady() so @ref MapViewportWidget
 * can repaint.
 *
 * @see MapProjection
 * @see MapViewportWidget
 */

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

/**
 * @class MapTileCache
 * @brief Asynchronous HTTP tile fetcher with an in-memory LRU image cache.
 *
 * ## Flow
 *
 * 1. Viewport calls @ref requestTiles() with a URL template and tile set.
 * 2. Cache hits are immediate; misses enqueue HTTP GETs.
 * 3. Completed downloads insert into the LRU and emit @ref tileReady().
 *
 * @note Pending requests for the same coord are coalesced via @c pending_.
 */
class MapTileCache : public QObject {
  Q_OBJECT

 public:
  /**
   * @brief Constructs the cache and network access manager.
   *
   * @param parent Qt parent object.
   */
  explicit MapTileCache(QObject* parent = nullptr);

  /**
   * @brief Aborts in-flight replies and clears the cache.
   */
  ~MapTileCache() override;

  /**
   * @brief Sets the HTTP @c User-Agent header for tile requests.
   *
   * @param user_agent User-Agent string (required by many tile providers).
   */
  void setUserAgent(const QString& user_agent);

  /**
   * @brief Caps the number of cached tile images.
   *
   * @param max_entries Maximum LRU entries (evicts oldest when exceeded).
   */
  void setMaxCacheEntries(int max_entries);

  /**
   * @brief Returns a cached tile image if present.
   *
   * @param coord Tile coordinate.
   * @return Cached image, or a null @c QImage on miss.
   */
  QImage cachedTile(const MapTileCoord& coord) const;

  /**
   * @brief Whether the server already reported this tile missing.
   *
   * @param coord Tile coordinate.
   * @return @c true after HTTP 404 or 204.
   */
  bool isUnavailable(const MapTileCoord& coord) const;

  /**
   * @brief Requests any missing tiles in @p coords from @p url_template.
   *
   * @param url_template Template containing @c {z}, @c {x}, @c {y} placeholders.
   * @param coords Set of tiles needed for the current viewport.
   */
  void requestTiles(const QString& url_template, const QSet<MapTileCoord>& coords);

  /**
   * @brief Clears cached images and cancels pending bookkeeping.
   */
  void clear();

 signals:
  /**
   * @brief Emitted when a tile image becomes available in the cache.
   *
   * @param coord Tile that finished loading (or was refreshed).
   */
  void tileReady(const MapTileCoord& coord);

  /**
   * @brief Emitted when a tile request fails with HTTP 404 or 204.
   *
   * @param coord Tile the server does not have.
   */
  void tileUnavailable(const MapTileCoord& coord);

 private slots:
  /**
   * @brief Handles a finished @c QNetworkReply: decode, insert, emit.
   */
  void onTileFinished();

 private:
  /**
   * @struct CacheEntry
   * @brief One LRU node pairing a coordinate with its image.
   */
  struct CacheEntry {
    MapTileCoord coord;  /**< Tile key. */
    QImage image;        /**< Decoded raster. */
  };

  /**
   * @brief Substitutes z/x/y into the URL template.
   *
   * @param url_template Template with placeholders.
   * @param coord Tile coordinate.
   * @return Absolute HTTP(S) URL.
   */
  QString buildTileUrl(const QString& url_template, const MapTileCoord& coord) const;

  /**
   * @brief Inserts or refreshes a cache entry and updates LRU order.
   *
   * @param coord Tile key.
   * @param image Decoded tile image.
   */
  void insertCacheEntry(const MapTileCoord& coord, QImage image);

  /** Evicts least-recently-used entries when over @c max_cache_entries_. */
  void evictIfNeeded();

  /**
   * @brief On-disk path for one tile of @p url_template.
   *
   * @param url_template Provider template (part of the cache key).
   * @param coord Tile coordinate.
   * @return Absolute PNG path under the user cache directory.
   */
  QString diskPath(const QString& url_template, const MapTileCoord& coord) const;

  /**
   * @brief Loads a previously saved tile, if the file exists.
   *
   * @param url_template Provider template.
   * @param coord Tile coordinate.
   * @param image Output image.
   * @return @c true when a decodable file was read.
   */
  bool loadDiskTile(const QString& url_template, const MapTileCoord& coord,
                    QImage* image) const;

  /**
   * @brief Writes a downloaded tile so later sessions can draw it offline.
   *
   * @param url_template Provider template.
   * @param coord Tile coordinate.
   * @param image Decoded tile.
   */
  void saveDiskTile(const QString& url_template, const MapTileCoord& coord,
                    const QImage& image) const;

  /** Network manager for HTTP tile GETs. */
  QNetworkAccessManager* network_ = nullptr;

  /** HTTP User-Agent. */
  QString user_agent_;

  /** Maximum number of images retained in @c cache_. */
  int max_cache_entries_ = 512;

  /** Tile coordinates the server reported missing (HTTP 404). */
  QSet<MapTileCoord> unavailable_;

  /** Coordinate → image lookup. */
  QHash<MapTileCoord, QImage> cache_;

  /** LRU order (front = most recent). */
  std::list<MapTileCoord> lru_order_;

  /** Coords with an in-flight request. */
  QSet<MapTileCoord> pending_;

  /** Active reply → coord mapping. */
  QHash<QNetworkReply*, MapTileCoord> active_replies_;
};

}  // namespace map
}  // namespace autoviz
