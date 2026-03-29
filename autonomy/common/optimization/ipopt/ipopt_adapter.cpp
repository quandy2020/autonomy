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

#include "autonomy/common/optimization/ipopt/ipopt_adapter.hpp"

namespace autonomy {
namespace common {
namespace optimization {

IpoptAdapter::IpoptAdapter(Problem& nlp, bool finite_diff) {
    nlp_ = &nlp;
    finite_diff_ = finite_diff;
}

bool IpoptAdapter::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                                Index& nnz_h_lag, IndexStyleEnum& index_style) {
    n = nlp_->GetNumberOfOptimizationVariables();
    m = nlp_->GetNumberOfConstraints();

    if (finite_diff_)
        nnz_jac_g = m * n;
    else
        nnz_jac_g = nlp_->GetJacobianOfConstraints().nonZeros();

    nnz_h_lag = n * n;

    // start index at 0 for row/col entries
    index_style = C_STYLE;

    return true;
}

bool IpoptAdapter::get_bounds_info(Index n, double* x_lower, double* x_upper,
                                   Index m, double* g_l, double* g_u) {
    auto bounds_x = nlp_->GetBoundsOnOptimizationVariables();
    for (std::size_t c = 0; c < bounds_x.size(); ++c) {
        x_lower[c] = bounds_x.at(c).lower_;
        x_upper[c] = bounds_x.at(c).upper_;
    }

    // specific bounds depending on equality and inequality constraints
    auto bounds_g = nlp_->GetBoundsOnConstraints();
    for (std::size_t c = 0; c < bounds_g.size(); ++c) {
        g_l[c] = bounds_g.at(c).lower_;
        g_u[c] = bounds_g.at(c).upper_;
    }

    return true;
}

bool IpoptAdapter::get_starting_point(Index n, bool init_x, double* x,
                                      bool init_z, double* z_L, double* z_U,
                                      Index m, bool init_lambda,
                                      double* lambda) {
    // Here, we assume we only have starting values for x
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);

    VectorXd x_all = nlp_->GetVariableValues();
    Eigen::Map<VectorXd>(&x[0], x_all.rows()) = x_all;

    return true;
}

bool IpoptAdapter::eval_f(Index n, const double* x, bool new_x,
                          double& obj_value) {
    obj_value = nlp_->EvaluateCostFunction(x);
    return true;
}

bool IpoptAdapter::eval_grad_f(Index n, const double* x, bool new_x,
                               double* grad_f) {
    Eigen::VectorXd grad = nlp_->EvaluateCostFunctionGradient(x, finite_diff_);
    Eigen::Map<Eigen::MatrixXd>(grad_f, n, 1) = grad;
    return true;
}

bool IpoptAdapter::eval_g(Index n, const double* x, bool new_x, Index m,
                          double* g) {
    VectorXd g_eig = nlp_->EvaluateConstraints(x);
    Eigen::Map<VectorXd>(g, m) = g_eig;
    return true;
}

bool IpoptAdapter::eval_jac_g(Index n, const double* x, bool new_x, Index m,
                              Index nele_jac, Index* iRow, Index* jCol,
                              double* values) {
    // defines the positions of the nonzero elements of the jacobian
    if (values == NULL) {
        // If "jacobian_approximation" option is set as
        // "finite-difference-values", the Jacobian is dense!
        Index nele = 0;
        if (finite_diff_) {  // dense jacobian
            for (Index row = 0; row < m; row++) {
                for (Index col = 0; col < n; col++) {
                    iRow[nele] = row;
                    jCol[nele] = col;
                    nele++;
                }
            }
        } else {  // sparse jacobian
            auto jac = nlp_->GetJacobianOfConstraints();
            for (int k = 0; k < jac.outerSize(); ++k) {
                for (Jacobian::InnerIterator it(jac, k); it; ++it) {
                    iRow[nele] = it.row();
                    jCol[nele] = it.col();
                    nele++;
                }
            }
        }

        assert(
            nele ==
            nele_jac);  // initial sparsity structure is never allowed to change
    } else {
        // only gets used if "jacobian_approximation finite-difference-values"
        // is not set
        nlp_->EvalNonzerosOfJacobian(x, values);
    }

    return true;
}

bool IpoptAdapter::intermediate_callback(
    AlgorithmMode mode, Index iter, double obj_value, double inf_pr,
    double inf_du, double mu, double d_norm, double regularization_size,
    double alpha_du, double alpha_pr, Index ls_trials, const IpoptData* ip_data,
    IpoptCalculatedQuantities* ip_cq) {
    nlp_->SaveCurrent();
    return true;
}

void IpoptAdapter::finalize_solution(SolverReturn status, Index n,
                                     const double* x, const double* z_L,
                                     const double* z_U, Index m,
                                     const double* g, const double* lambda,
                                     double obj_value, const IpoptData* ip_data,
                                     IpoptCalculatedQuantities* ip_cq) {
    nlp_->SetVariables(x);
    nlp_->SaveCurrent();
}

}  // namespace optimization
}  // namespace common
}  // namespace autonomy