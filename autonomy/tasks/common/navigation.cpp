/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/common/navigation.hpp"

#include "autonomy/tasks/behavior_tree/navigator/behavior_tree_navigation_engine.hpp"

namespace autonomy {
namespace tasks {

std::shared_ptr<NavigationEngine> CreateBehaviorTreeNavigationEngine() {
    return std::make_shared<behavior_tree::BehaviorTreeNavigationEngine>();
}

}  // namespace tasks
}  // namespace autonomy
