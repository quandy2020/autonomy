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

#pragma once

#include "autonomy/common/optimization/core/problem.hpp"
#include "autonomy/common/optimization/core/solver.hpp"

namespace Ipopt {
class IpoptApplication;
}

namespace autonomy {
namespace common {
namespace optimization {

/**
 * @brief An interface to IPOPT, fully hiding its implementation.
 *
 * To set specific options, see:
 * https://www.coin-or.org/Ipopt/documentation/node40.html
 *
 * @ingroup Solvers
 */
class IpoptSolver : public Solver
{
public:
    using Ptr = std::shared_ptr<IpoptSolver>;

    IpoptSolver(bool rethrow_non_ipopt_exceptions = false);
    virtual ~IpoptSolver() = default;

    /** @brief  Creates an IpoptAdapter and solves the NLP.
     * @param [in/out]  nlp  The specific problem.
     */
    void Solve(Problem& nlp) override;

    /** Set options for the IPOPT solver. A complete list can be found here:
     * https://www.coin-or.org/Ipopt/documentation/node40.html
     */
    void SetOption(const std::string& name, const std::string& value);
    void SetOption(const std::string& name, int value);
    void SetOption(const std::string& name, double value);

    /** @brief  Get the total wall clock time for the optimization, including
     * function evaluations.
     */
    double GetTotalWallclockTime();

private:
    std::shared_ptr<Ipopt::IpoptApplication> ipopt_app_;
};

}  // namespace optimization
}  // namespace common
}  // namespace autonomy