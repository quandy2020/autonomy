/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QString>

#include "autoviz/ui/map/map_geojson_parser.hpp"

namespace autoviz {
namespace map {

class MapMessageIngest {
 public:
  static bool SupportsMessageType(const QString& message_type);
  static MapIngestResult Ingest(const QString& message_type,
                                const std::string& payload, QString* error);
};

}  // namespace map
}  // namespace autoviz
