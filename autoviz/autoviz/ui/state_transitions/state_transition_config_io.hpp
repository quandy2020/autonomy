/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QString>

#include "autoviz/common/session_config.hpp"
#include "autoviz/ui/state_transitions/state_transition_types.hpp"

namespace autoviz {
namespace state_transitions {

common::StateTransitionPanelPersistConfig ToPersistConfig(
    const QString& object_name, const StateTransitionPanelConfig& config);
StateTransitionPanelConfig FromPersistConfig(
    const common::StateTransitionPanelPersistConfig& persist);

}  // namespace state_transitions
}  // namespace autoviz
