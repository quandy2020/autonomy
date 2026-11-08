/* Copyright 2026 The Openbot Authors (duyongquan)
 * Licensed under the Apache License, Version 2.0.
 */
/** @file navigation_component_test.cpp
 * @brief Component configuration and disabled-model startup regression.
 */
#include "navigation/navigation_component.hpp"

int main(int argc, char** argv) {
    if (argc != 2)
        return 1;
    char validate[] = "--validate";
    char* args[] = {argv[0], argv[1], validate, nullptr};
    using autonomy::navigation::NavigationComponent;
    if (NavigationComponent::RunNavigation(3, args) != 0)
        return 1;
    return NavigationComponent::RunNavigation(2, args) == 3 ? 0 : 1;
}
