/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#pragma once

// OCS2
#include <automanip/core/types.hpp>
#include <automanip/core/initialization/initializer.hpp>
#include <automanip/ddp/ddp_settings.hpp>
#include <automanip/mpc/mpc_settings.hpp>
#include <automanip/oc/rollout/time_triggered_rollout.hpp>
#include <automanip/tools/common/robot_interface.hpp>

// CartPole
#include "examples/cartpole/cart_pole_parameters.hpp"
#include "examples/cartpole/definitions.hpp"

namespace automanip {
namespace cartpole {

class CartPoleInterface final : public RobotInterface {
 public:
  /**
   * Constructor
   *
   * @note Creates directory for generated library into if it does not exist.
   * @throw Invalid argument error if input task file does not exist.
   *
   * @param [in] taskFile: The absolute path to the configuration file for the MPC.
   * @param [in] libraryFolder: The absolute path to the directory to generate CppAD library into.
   * @param [in] verbose: Flag to determine to print out the loaded settings and status of complied libraries.
   */
  CartPoleInterface(const std::string& taskFile, const std::string& libraryFolder, bool verbose);

  /**
   * Destructor
   */
  ~CartPoleInterface() override = default;

  const vector_t& getInitialState() { return initialState_; }

  const vector_t& getInitialTarget() { return xFinal_; }

  ddp::Settings& ddpSettings() { return ddpSettings_; }

  mpc::Settings& mpcSettings() { return mpcSettings_; }

  OptimalControlProblem& optimalControlProblem() { return problem_; }
  const OptimalControlProblem& getOptimalControlProblem() const override { return problem_; }

  const RolloutBase& getRollout() const { return *rolloutPtr_; }

  const Initializer& getInitializer() const override { return *cartPoleInitializerPtr_; }

 private:
  ddp::Settings ddpSettings_;
  mpc::Settings mpcSettings_;

  OptimalControlProblem problem_;

  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::unique_ptr<Initializer> cartPoleInitializerPtr_;

  vector_t initialState_{STATE_DIM};
  vector_t xFinal_{STATE_DIM};
};

}  // namespace cartpole
}  // namespace automanip
