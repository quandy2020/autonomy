/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <automanip/core/loopshaping/augmented_lagrangian/loopshaping_augmented_lagrangian.hpp>
#include <automanip/core/loopshaping/augmented_lagrangian/loopshaping_augmented_lagrangian_eliminate_pattern.hpp>
#include <automanip/core/loopshaping/augmented_lagrangian/loopshaping_augmented_lagrangian_output_pattern.hpp>
#include <automanip/core/loopshaping/augmented_lagrangian/loopshaping_state_augmented_lagrangian.hpp>
#include <automanip/core/loopshaping/augmented_lagrangian/loopshaping_state_input_augmented_lagrangian.hpp>

namespace automanip {
namespace LoopshapingAugmentedLagrangian {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateAugmentedLagrangianCollection> create(const StateAugmentedLagrangianCollection& systemAugmentedLagrangian,
                                                           std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) {
  return std::make_unique<LoopshapingStateAugmentedLagrangian>(systemAugmentedLagrangian, std::move(loopshapingDefinition));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputAugmentedLagrangianCollection> create(const StateInputAugmentedLagrangianCollection& systemAugmentedLagrangian,
                                                                std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) {
  switch (loopshapingDefinition->getType()) {
    case LoopshapingType::outputpattern:
      return std::make_unique<LoopshapingAugmentedLagrangianOutputPattern>(systemAugmentedLagrangian, std::move(loopshapingDefinition));
    case LoopshapingType::eliminatepattern:
      return std::make_unique<LoopshapingAugmentedLagrangianEliminatePattern>(systemAugmentedLagrangian, std::move(loopshapingDefinition));
    default:
      throw std::runtime_error("[LoopshapingAugmentedLagrangian::create] invalid loopshaping type");
  }
}

}  // namespace LoopshapingAugmentedLagrangian
}  // namespace automanip
