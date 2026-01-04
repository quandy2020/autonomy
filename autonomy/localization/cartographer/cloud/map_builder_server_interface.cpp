#include "autonomy/localization/cartographer/cloud/map_builder_server_interface.hpp"

#include "absl/memory/memory.h"
#include "autonomy/localization/cartographer/cloud/internal/map_builder_server.hpp"

namespace cartographer {
namespace cloud {

void RegisterMapBuilderServerMetrics(metrics::FamilyFactory* factory) {
    MapBuilderServer::RegisterMetrics(factory);
}

std::unique_ptr<MapBuilderServerInterface> CreateMapBuilderServer(
    const proto::MapBuilderServerOptions& map_builder_server_options,
    std::unique_ptr<mapping::MapBuilderInterface> map_builder) {
    return absl::make_unique<MapBuilderServer>(map_builder_server_options,
                                               std::move(map_builder));
}

}  // namespace cloud
}  // namespace cartographer
