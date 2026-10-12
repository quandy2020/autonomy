/*
 * Copyright 2026 The Openbot Authors
 *
 * move_group capability plugin base.
 */

#pragma once

#include <string>

#include "autonomy/common/macros.hpp"
#include "automanip/proto/capability.pb.h"

namespace automanip {

class ManipulationServer;

namespace dispatch {

/** @brief Result of a single-state validity query. */
using StateValidationResult =
    ::automanip::proto::StateValidationResult;

/** @brief Planner interface descriptor (MoveIt PlannerInterface analogue). */
using PlannerInterfaceInfo =
    ::automanip::proto::PlannerInterfaceInfo;

/**
 * @brief Base interface for move_group-style capability plugins.
 */
class Capability {
 public:
  /**
   * @brief Define Capability::SharedPtr type
   */
  AUTONOMY_SMART_PTR_DEFINITIONS(Capability)

  virtual ~Capability() = default;

  /** @brief Stable capability name used for lookup / plugin registration. */
  virtual std::string Name() const = 0;

  /**
   * @brief Bind to the owning ManipulationServer.
   * @param[in] server Runtime server (not owned).
   * @return true on successful initialization.
   */
  virtual bool Init(ManipulationServer* server) = 0;
};

}  // namespace dispatch
}  // namespace automanip
