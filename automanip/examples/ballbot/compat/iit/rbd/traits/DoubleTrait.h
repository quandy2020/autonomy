/*
 * Copyright 2026 Automanip contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file DoubleTrait.h
 * @brief Name used by the RobCoGen ballbot sources for the double scalar trait.
 */

#ifndef AUTOMANIP_BALLBOT_IIT_RBD_TRAITS_DOUBLE_TRAIT_H_
#define AUTOMANIP_BALLBOT_IIT_RBD_TRAITS_DOUBLE_TRAIT_H_

#include <iit/rbd/scalar_traits.h>

namespace iit {
namespace rbd {

/** RobCoGen emits `DoubleTrait`; current iit_rbd spells it `DoubleTraits`. */
using DoubleTrait = DoubleTraits;

}  // namespace rbd
}  // namespace iit

#endif
