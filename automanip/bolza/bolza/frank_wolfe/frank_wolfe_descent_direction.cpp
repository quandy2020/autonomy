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

#include <bolza/frank_wolfe/frank_wolfe_descent_direction.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FrankWolfeDescentDirection::FrankWolfeDescentDirection(bool display)
    : lpPtr_(glp_create_prob(), glp_delete_prob), lpOptionsPtr_(new glp_smcp) {
  // set LP options
  glp_init_smcp(lpOptionsPtr_.get());
  if (!display) lpOptionsPtr_->msg_lev = GLP_MSG_ERR;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FrankWolfeDescentDirection::InstantiateGLPK() {
  // erase the solver
  glp_erase_prob(lpPtr_.get());

  // name
  glp_set_prob_name(lpPtr_.get(), "FrankWolfe");

  // set it as a minimization problem
  glp_set_obj_dir(lpPtr_.get(), GLP_MIN);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FrankWolfeDescentDirection::SetupLP(const vector_t& parameter, const vector_t& gradient, const vector_t& maxGradientInverse,
                                         NLPConstraints* nlpConstraintsPtr) {
  const size_t parameterDim = parameter.size();

  // return if there is no parameter
  if (parameterDim == 0) return;

  // set parameters limits
  glp_add_cols(lpPtr_.get(), parameterDim);

  // set the LP cost function of Frank-Wolfe algorithm
  for (size_t i = 0; i < parameterDim; i++) glp_set_obj_coef(lpPtr_.get(), i + 1, gradient(i));

  // set descent directions reciprocal element-wise max
  const vector_t Ev = maxGradientInverse.cwiseAbs();
  for (size_t i = 0; i < parameterDim; i++) {
    // if the gradient is zero in one direction
    if (numerics::almost_eq(gradient(i), 0.0)) {
      glp_set_col_bnds(lpPtr_.get(), i + 1, GLP_FX, 0.0, 0.0);

      // if the gradient should be limited
    } else if (!numerics::almost_eq(Ev(i), 0.0)) {
      glp_set_col_bnds(lpPtr_.get(), i + 1, GLP_DB, -1.0 / Ev(i), 1.0 / Ev(i));

      // if free
    } else {
      glp_set_col_bnds(lpPtr_.get(), i + 1, GLP_FR, 0.0, 0.0);
    }

  }  // end of i loop

  // set the current parameter vector.
  nlpConstraintsPtr->setCurrentParameter(parameter);

  // get domain equality constraints
  vector_t g;
  nlpConstraintsPtr->getLinearEqualityConstraint(g);
  matrix_t dgdx;
  nlpConstraintsPtr->getLinearEqualityConstraintDerivative(dgdx);
  if (g.size() > 0 && dgdx.cols() != parameterDim)
    throw std::runtime_error(
        "calculateLinearEqualityConstraint: The number of columns of Jacobian matrix "
        "should be equal to the number of parameters.");
  if (g.size() > 0 && dgdx.rows() != g.rows())
    throw std::runtime_error(
        "calculateLinearEqualityConstraint: The number of rows of Jacobian matrix "
        "should be equal to the number of equality constraints.");

  // get domain inequality constraints
  vector_t h;
  nlpConstraintsPtr->getLinearInequalityConstraint(h);
  matrix_t dhdx;
  nlpConstraintsPtr->getLinearInequalityConstraintDerivative(dhdx);
  if (h.size() > 0 && dhdx.cols() != parameterDim)
    throw std::runtime_error(
        "calculateLinearInequalityConstraint: The number of columns of Jacobian matrix "
        "should be equal to the number of parameters.");
  if (h.size() > 0 && dhdx.rows() != h.rows())
    throw std::runtime_error(
        "calculateLinearInequalityConstraint: The number of rows of Jacobian matrix "
        "should be equal to the number of inequality constraints.");

  // set the total number of constraint limits
  glp_add_rows(lpPtr_.get(), g.size() + h.size());
  scalar_array_t values{0.1};     // 0 index is not used!
  std::vector<int> xIndices{-1};  // 0 index is not used!
  std::vector<int> yIndices{-1};  // 0 index is not used!

  // domain equality constraints
  for (size_t i = 0; i < g.size(); i++) {
    for (size_t j = 0; j < parameterDim; j++) {
      if (!numerics::almost_eq(dgdx(i, j), 0.0)) {
        values.push_back(dgdx(i, j));
        xIndices.push_back(i + 1);
        yIndices.push_back(j + 1);
      }
    }
    glp_set_row_bnds(lpPtr_.get(), i + 1, GLP_FX, -g(i), -g(i));
  }

  // domain inequality constraints
  for (size_t i = 0; i < h.size(); i++) {
    for (size_t j = 0; j < parameterDim; j++) {
      if (!numerics::almost_eq(dhdx(i, j), 0.0)) {
        values.push_back(dhdx(i, j));
        xIndices.push_back(i + 1);
        yIndices.push_back(j + 1);
      }
    }
    glp_set_row_bnds(lpPtr_.get(), i + 1, GLP_LO, -h(i), 0.0);
  }

  // set the constraint coefficients
  glp_load_matrix(lpPtr_.get(), g.size() + h.size(), xIndices.data(), yIndices.data(), values.data());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FrankWolfeDescentDirection::Run(const vector_t& parameter, const vector_t& gradient, const vector_t& maxGradientInverse,
                                     NLPConstraints* nlpConstraintsPtr, vector_t& fwDescentDirection) {
  if (gradient.size() != parameter.size()) throw std::runtime_error("The gradient vector size is incompatible to the parameter size.");
  if (maxGradientInverse.size() != gradient.size())
    throw std::runtime_error("The gradient limit size is incompatible to the gradient size.");

  // instantiate GLPK
  InstantiateGLPK();

  // setup LP
  SetupLP(parameter, gradient, maxGradientInverse, nlpConstraintsPtr);

  // solve LP
  glp_simplex(lpPtr_.get(), lpOptionsPtr_.get());

  // get the solution
  fwDescentDirection.resize(parameter.size());
  for (size_t i = 0; i < parameter.size(); i++) fwDescentDirection(i) = glp_get_col_prim(lpPtr_.get(), i + 1);

  // test
  if (gradient.dot(fwDescentDirection) > 0) throw std::runtime_error("Frank-Wolfe does not produce a descent direction.");
}

}  // namespace bolza
