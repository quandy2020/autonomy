/*
 * Copyright 2026 The Openbot Authors
 */

#include "automanip/constraints/union_constraint_sampler_allocator.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"

namespace automanip {
namespace constraints {

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(UnionConstraintSamplerAllocator, ConstraintSamplerAllocator);

}  // namespace constraints
}  // namespace automanip
