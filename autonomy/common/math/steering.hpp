/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file steering.hpp
 * @brief Dubins, Reeds–Shepp, and clothoid (HC/CC) steering.
 *
 * Ported from steering_functions (Apache-2.0) with OMPL Dubins/
 * Reeds–Shepp and Inria CC-Steer (BSD); see steering/3rd-party-licenses.txt.
 * Covariance uses Eigen and autonomy::common::kalman_filter.
 */

#pragma once

#include "autonomy/common/math/steering/types.hpp"
#include "autonomy/common/math/steering/utilities.hpp"
#include "autonomy/common/math/steering/dubins.hpp"
#include "autonomy/common/math/steering/reeds_shepp.hpp"
#include "autonomy/common/math/steering/clothoid/dubins.hpp"
#include "autonomy/common/math/steering/clothoid/reeds_shepp.hpp"
#include "autonomy/common/math/steering/clothoid/hybrid.hpp"
