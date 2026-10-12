/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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

#pragma once

/**
 * @file loopshaping_initializer.hpp
 * @brief loopshaping initializer (bolza).
 */


#include <memory>

#include <bolza/core/types.hpp>
#include <bolza/core/initialization/initializer.hpp>
#include <bolza/core/loopshaping/loopshaping_definition.hpp>

namespace bolza {

class LoopshapingInitializer final : public Initializer {
 public:
  /**
   * @brief Constructs a LoopshapingInitializer.
   * @param [in] systembase: Systembase.
   * @param [in] loopshapingDefinition: Loopshaping Definition.
   */
  LoopshapingInitializer(const Initializer& systembase, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);

  /**
   * @brief Destructor.
   */
  ~LoopshapingInitializer() override = default;

  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  LoopshapingInitializer* Clone() const override;

  /**
   * @brief Compute.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] nextTime: Next Time.
   * @param [in,out] input: Input.
   * @param [in,out] nextState: Next State.
   */
  void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) override;

 private:
  /**
   * @brief Constructs a LoopshapingInitializer.
   * @param [in] obj: Obj.
   */
  LoopshapingInitializer(const LoopshapingInitializer& obj);

  std::unique_ptr<Initializer> systembase_;
  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
};

}  // namespace bolza
