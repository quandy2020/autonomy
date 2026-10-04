/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file map_message_ingest.hpp
 * @brief Decode map-compatible message payloads into @ref MapIngestResult.
 *
 * Dispatches by message type (GeoJSON text, NavSatFix, Pose, Path, …) for
 * @ref MapPanel subscription handlers.
 *
 * @see MapGeoJsonParser
 * @see MapLayerStore
 */

#pragma once

#include <QString>

#include "autoviz/ui/map/map_geojson_parser.hpp"

namespace autoviz {
namespace map {

/**
 * @class MapMessageIngest
 * @brief Stateless facade: message type + payload → geographic features.
 */
class MapMessageIngest {
 public:
  /**
   * @brief Whether Autoviz can ingest the given schema as map geometry.
   *
   * @param message_type Fully-qualified message type name.
   * @return @c true when @ref Ingest() can produce features.
   */
  static bool SupportsMessageType(const QString& message_type);

  /**
   * @brief Parses a payload into points / lines / polygons.
   *
   * @param message_type Schema type (selects decoder).
   * @param payload Serialized message bytes.
   * @param error Optional human-readable failure reason.
   * @return Ingest batch (empty on failure).
   * @see SupportsMessageType()
   */
  static MapIngestResult Ingest(const QString& message_type,
                                const std::string& payload, QString* error);
};

}  // namespace map
}  // namespace autoviz
