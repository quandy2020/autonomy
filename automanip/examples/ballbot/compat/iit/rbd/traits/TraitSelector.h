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
 * @file TraitSelector.h
 * @brief Maps a scalar type onto the iit_rbd trait the generated dynamics call.
 */

#ifndef AUTOMANIP_BALLBOT_IIT_RBD_TRAITS_TRAIT_SELECTOR_H_
#define AUTOMANIP_BALLBOT_IIT_RBD_TRAITS_TRAIT_SELECTOR_H_

#include <cppad/cg.hpp>

#include <iit/rbd/scalar_traits.h>
#include <iit/robcogen/scalar/cppad.h>

namespace iit {
namespace rbd {
namespace tpl {

/**
 * RobCoGen looks up `TraitSelector<Scalar>::Trait` for `sin`/`cos`.
 * `ScalarTraits` covers `double`. The ballbot dynamics also differentiate
 * through `CppAD::AD<CppAD::cg::CG<double>>`, which `std::sin` cannot accept.
 */
template <typename SCALAR>
struct TraitSelector {
  using Trait = ScalarTraits<SCALAR>;
};

template <>
struct TraitSelector<CppAD::AD<CppAD::cg::CG<double>>> {
  struct Trait {
    using Scalar = CppAD::AD<CppAD::cg::CG<double>>;
    static Scalar sin(const Scalar& x) { return CppAD::sin(x); }
    static Scalar cos(const Scalar& x) { return CppAD::cos(x); }
  };
};

}  // namespace tpl
}  // namespace rbd
}  // namespace iit

#endif
