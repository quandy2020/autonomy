/*
 * Copyright 2026 The Openbot Authors
 */
#include "argus/system/voxel/voxel_module.hpp"
#include "argus/common/component_factory.hpp"

namespace argus {
namespace system {
namespace voxel {

void RegisterVoxelPlugin(common::ComponentFactory* factory) {
    if (factory == nullptr) {
        return;
    }
    factory->RegisterCreator(
        "voxel",
        [](const common::Component::Config& cfg) -> std::unique_ptr<common::Component> {
            auto m = std::make_unique<VoxelModule>(
                cfg.name.empty() ? "voxel" : cfg.name);
            m->Configure(cfg);
            return m;
        });
}

}  // namespace voxel
}  // namespace system
}  // namespace argus
