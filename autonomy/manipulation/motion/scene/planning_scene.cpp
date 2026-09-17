/*
 * Copyright 2026 The Openbot Authors
 *
 * Helpers for PlanningScene live in collision_object_util.hpp (header-only).
 * This translation unit exists for CMake target linkage stability.
 */

#include "autonomy/manipulation/motion/scene/planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace scene {

// Intentionally empty: UpdateMeshAabb / TransformAttached are inline in
// collision_object_util.hpp.

}  // namespace scene
}  // namespace manipulation
}  // namespace autonomy
