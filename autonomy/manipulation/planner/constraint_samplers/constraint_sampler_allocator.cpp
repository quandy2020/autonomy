/*
 * Copyright 2026 The Openbot Authors
 *
 * Register ConstraintSamplerAllocator concrete classes as Autolink plugins.
 */

#include "autonomy/manipulation/planner/constraint_samplers/constraint_sampler_allocator.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"

namespace autonomy {
namespace manipulation {
namespace constraint_samplers {

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(UnionConstraintSamplerAllocator,
                                        ConstraintSamplerAllocator);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(IkConstraintSamplerAllocator,
                                        ConstraintSamplerAllocator);
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(JointConstraintSamplerAllocator,
                                        ConstraintSamplerAllocator);

}  // namespace constraint_samplers
}  // namespace manipulation
}  // namespace autonomy
