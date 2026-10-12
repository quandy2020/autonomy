/*
 * Copyright 2026 The Openbot Authors
 *
 * Helpers for PlanningScene live in collision_object_helpers.hpp (header-only).
 * This translation unit exists for CMake target linkage stability.
 */

#include "automanip/motion/scene/planning_scene.hpp"

namespace automanip {
namespace scene {

// Intentionally empty: UpdateMeshAabb / TransformAttached are inline in
// collision_object_helpers.hpp.

}  // namespace scene
}  // namespace automanip
