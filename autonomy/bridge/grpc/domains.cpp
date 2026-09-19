/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file domains.cpp
 * @brief Implementation of DomainBundle construction and RegisterCancels.
 */

#include "autonomy/bridge/grpc/domains.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {

DomainBundle::DomainBundle(std::shared_ptr<autolink::Node> node,
                           TaskMuxer::SharedPtr muxer,
                           WorkScheduler* scheduler,
                           CommandIdempotencyCache* idempotency)
    : navigator_(clients::NavigatorStub::make_unique(node, muxer)),
      teleop_(clients::TeleopStub::make_unique(node, muxer, scheduler,
                                               idempotency)),
      follow_(clients::FollowStub::make_unique(node, muxer)),
      charge_(clients::ChargeStub::make_unique(node, muxer)),
      mapping_(clients::MappingStub::make_unique(node, muxer)),
      map_service_(
          clients::MapServiceStub::make_unique(node, mapping_.get())),
      localization_(clients::LocalizationStub::make_unique(node, muxer)),
      exploration_(clients::ExplorationStub::make_unique(node, muxer)),
      voice_(clients::VoiceStub::make_unique(node, muxer)),
      sensor_(clients::SensorStub::make_unique(node, scheduler)),
      system_monitor_(
          clients::SystemMonitorStub::make_unique(node, muxer)) {
    RegisterCancels();
}

void DomainBundle::RegisterCancels() {
    RegisterDomainCancel(registry_, "navigator", navigator_.get(),
                         [](auto& stub) {
                             if (stub.IsNavigating()) {
                                 stub.CancelGoal();
                             }
                         });
    RegisterDomainCancel(registry_, "teleop", teleop_.get(),
                         [](auto& stub) { stub.ResetSession(); });
    RegisterDomainCancel(registry_, "follow", follow_.get(),
                         [](auto& stub) { stub.CancelSession(); });
    RegisterDomainCancel(registry_, "charge", charge_.get(),
                         [](auto& stub) { stub.CancelSession(); });
    RegisterDomainCancel(registry_, "mapping", mapping_.get(),
                         [](auto& stub) { stub.CancelSession(); });
    RegisterDomainCancel(registry_, "exploration", exploration_.get(),
                         [](auto& stub) { stub.CancelSession(); });
    RegisterDomainCancel(registry_, "voice", voice_.get(),
                         [](auto& stub) { stub.CancelSession(); });
}

}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
