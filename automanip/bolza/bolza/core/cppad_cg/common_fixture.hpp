#pragma once

#include <bolza/core/types.hpp>
#include <bolza/core/automatic_differentiation/cpp_ad_interface.hpp>
#include <bolza/core/automatic_differentiation/cpp_ad_sparsity.hpp>
#include <bolza/core/automatic_differentiation/types.hpp>

namespace bolza {

class CommonCppAdNoParameterFixture : public ::testing::Test {
 public:
  using ad_fun_t = CppAdInterface::ad_fun_t;
  using ad_vector_t = bolza::ad_vector_t;

  const size_t variableDim_ = 3;
  const size_t rangeDim_ = 1;
  const size_t parameterDim_ = 0;

  CommonCppAdNoParameterFixture() {
    Create();
    Init();
  }

  virtual ~CommonCppAdNoParameterFixture() = default;

  void Create(){};

  static void FunImpl(const ad_vector_t& x, ad_vector_t& y) {
    // the model equation
    y.resize(1);
    y(0) = x(0) + 0.5 * x(0) * x(1) + 2.0 * x(0) * x(1) * x(2);
  }

  vector_t TestFun(const vector_t& x) {
    vector_t y(rangeDim_);
    y(0) = x(0) + 0.5 * x(0) * x(1) + 2.0 * x(0) * x(1) * x(2);
    return y;
  }

  matrix_t TestJacobian(const vector_t& x) {
    matrix_t Jacobian(rangeDim_, variableDim_);
    // y(0):
    Jacobian(0, 0) = 1.0 + 0.5 * x(1) + 2.0 * x(1) * x(2);
    Jacobian(0, 1) = 0.5 * x(0) + 2.0 * x(0) * x(2);
    Jacobian(0, 2) = 2.0 * x(0) * x(1);
    return jacobian;
  }

  matrix_t TestHessian(const vector_t& x) {
    matrix_t Hessian(variableDim_, variableDim_);
    hessian.setZero();

    Hessian(0, 0) = 0.0;
    Hessian(0, 1) = 0.5 + 2.0 * x(2);
    Hessian(1, 0) = Hessian(0, 1);
    Hessian(0, 2) = 2.0 * x(1);
    Hessian(2, 0) = Hessian(0, 2);
    Hessian(1, 1) = 0.0;
    Hessian(1, 2) = 2.0 * x(0);
    Hessian(2, 1) = Hessian(1, 2);
    Hessian(2, 2) = 0.0;

    return hessian;
  }

  void Init() {}
};

class CommonCppAdParameterizedFixture : public ::testing::Test {
 public:
  using ad_fun_t = CppAdInterface::ad_fun_t;
  using ad_vector_t = bolza::ad_vector_t;

  const size_t variableDim_ = 2;
  const size_t rangeDim_ = 2;
  const size_t parameterDim_ = 1;

  CommonCppAdParameterizedFixture() {
    Create();
    Init();
  }

  virtual ~CommonCppAdParameterizedFixture() = default;

  void Create() {
    // set and declare independent variables and start tape recording
    ad_vector_t xp(variableDim_ + parameterDim_);
    xp.setOnes();
    CppAD::Independent(xp);

    ad_vector_t x = xp.segment(0, variableDim_);
    ad_vector_t p = xp.segment(variableDim_, parameterDim_);
    ad_vector_t y;
    FunImpl(x, p, y);

    // create f: x -> y and stop tape recording
    fun_.Reset(new ad_fun_t(xp, y));
    fun_->optimize();

    jacobianSparsity_.resize(rangeDim_);
    jacobianSparsity_[0] = {0, 1, 2};
    jacobianSparsity_[1] = {0, 1};

    hessianSparsity_.resize(variableDim_ + parameterDim_);
    hessianSparsity_[0] = {0, 1};
    hessianSparsity_[1] = {0, 1, 2};
    hessianSparsity_[2] = {1};
  }

  static void FunImpl(const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) {
    // the model equation
    y.resize(2);
    y(0) = x(0) + 0.5 * x(0) * x(1) + 2.0 * p(0) * x(1) * x(1);
    y(1) = x(0) * x(0) * x(1) / 2.0;
  }

  vector_t TestFun(const vector_t& x, const vector_t& p) {
    vector_t y(rangeDim_);
    y(0) = x(0) + 0.5 * x(0) * x(1) + 2.0 * p(0) * x(1) * x(1);
    y(1) = x(0) * x(0) * x(1) / 2.0;
    return y;
  }

  matrix_t TestJacobian(const vector_t& x, const vector_t& p) {
    matrix_t Jacobian(rangeDim_, variableDim_);
    // y(0):
    Jacobian(0, 0) = 1.0 + 0.5 * x(1);
    Jacobian(0, 1) = 0.5 * x(0) + 4.0 * p(0) * x(1);
    // y(1):
    Jacobian(1, 0) = x(0) * x(1);
    Jacobian(1, 1) = x(0) * x(0) / 2.0;
    return jacobian;
  }

  matrix_t TestHessian(int outputIndex, const vector_t& x, const vector_t& p) {
    matrix_t Hessian(variableDim_, variableDim_);

    switch (outputIndex) {
      case 0:
        Hessian(0, 0) = 0.0;
        Hessian(0, 1) = 0.5;
        Hessian(1, 0) = Hessian(0, 1);
        Hessian(1, 1) = 4.0 * p(0);
        break;
      case 1:
        Hessian(0, 0) = x(1);
        Hessian(0, 1) = x(0);
        Hessian(1, 0) = Hessian(0, 1);
        Hessian(1, 1) = 0.0;
        break;
    }

    return hessian;
  }

  void Init() {}

  std::unique_ptr<ad_fun_t> fun_;
  cppad_sparsity::SparsityPattern jacobianSparsity_;
  cppad_sparsity::SparsityPattern hessianSparsity_;
};

}  // namespace bolza
