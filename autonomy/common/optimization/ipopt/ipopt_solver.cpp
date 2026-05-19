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

#include "autonomy/common/optimization/ipopt/ipopt_solver.hpp"

#include "autonomy/common/log.hpp"
#include "autonomy/common/optimization/ipopt/ipopt_adapter.hpp"

namespace autonomy {
namespace common {
namespace optimization {

IpoptSolver::IpoptSolver(bool rethrow_non_ipopt_exceptions) {
    ipopt_app_ = std::make_shared<Ipopt::IpoptApplication>();
    status_ = Ipopt::Solve_Succeeded;

    /* Which linear solver to use. Mumps is default because it comes with the
     * precompiled ubuntu binaries. However, the coin-hsl solvers can be
     * significantly faster and are free for academic purposes. They can be
     * downloaded here: http://www.hsl.rl.ac.uk/ipopt/ and must be compiled
     * into your IPOPT libraries. Then you can use the additional strings:
     * "ma27, ma57, ma77, ma86, ma97" here.
     */
    SetOption("linear_solver", "mumps");

    /* whether to use the analytical derivatives "exact" coded in ifopt, or let
     * IPOPT approximate these through "finite difference-values". This is
     * usually significantly slower.
     */
    SetOption("jacobian_approximation", "exact");
    SetOption("hessian_approximation", "limited-memory");
    SetOption("max_cpu_time", 40.0);
    SetOption("tol", 0.001);
    SetOption("print_timing_statistics", "no");
    SetOption("print_user_options", "no");
    SetOption("print_level", 4);

    // SetOption("max_iter", 1);
    // SetOption("derivative_test", "first-order");
    // SetOption("derivative_test_tol", 1e-3);

    // Enable or Disable throwing original exceptions for catching errors
    ipopt_app_->RethrowNonIpoptException(rethrow_non_ipopt_exceptions);
}

void IpoptSolver::Solve(Problem& nlp) {
    using namespace Ipopt;

    status_ = ipopt_app_->Initialize();
    if (status_ != Solve_Succeeded) {
        AERROR << "*** Error during initialization!";
        throw std::length_error("Ipopt could not initialize correctly");
    }

    // check the jacobian_approximation method
    std::string jac_type = "";
    ipopt_app_->Options()->GetStringValue("jacobian_approximation", jac_type,
                                          "");
    bool finite_diff = jac_type == "finite-difference-values";

    // convert the NLP problem to Ipopt
    SmartPtr<TNLP> nlp_ptr = new IpoptAdapter(nlp, finite_diff);
    status_ = ipopt_app_->OptimizeTNLP(nlp_ptr);

    if (status_ != Solve_Succeeded) {
        AERROR << "Ipopt failed to find a solution. Return Code: " << status_;
    }
}

void IpoptSolver::SetOption(const std::string& name, const std::string& value) {
    ipopt_app_->Options()->SetStringValue(name, value);
}

void IpoptSolver::SetOption(const std::string& name, int value) {
    ipopt_app_->Options()->SetIntegerValue(name, value);
}

void IpoptSolver::SetOption(const std::string& name, double value) {
    ipopt_app_->Options()->SetNumericValue(name, value);
}

double IpoptSolver::GetTotalWallclockTime() {
    return ipopt_app_->Statistics()->TotalWallclockTime();
}

}  // namespace optimization
}  // namespace common
}  // namespace autonomy