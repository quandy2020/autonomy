

#include <gtest/gtest.h>

#include "commonFixture.h"

using namespace bolza;

class CppAdInterfaceNoParameterFixture : public CommonCppAdNoParameterFixture {};
class CppAdInterfaceParameterizedFixture : public CommonCppAdParameterizedFixture {};

TEST_F(CppAdInterfaceNoParameterFixture, testModelGeneration) {
  bolza::CppAdInterface adInterface(funImpl, variableDim_, "testModelWithoutParameters");

  adInterface.CreateModels(bolza::CppAdInterface::ApproximationOrder::Second, true);
  vector_t x = vector_t::Random(variableDim_);

  ASSERT_TRUE(adInterface.GetFunctionValue(x).isApprox(TestFun(x)));
  ASSERT_TRUE(adInterface.GetJacobian(x).isApprox(TestJacobian(x)));
  ASSERT_TRUE(adInterface.GetHessian(0, x).isApprox(TestHessian(x)));

  const auto gnApproximation = adInterface.GetGaussNewtonApproximation(x);
  ASSERT_DOUBLE_EQ(gnApproximation.f, 0.5 * TestFun(x).squaredNorm());
  ASSERT_TRUE(gnApproximation.dfdx.isApprox(TestJacobian(x).transpose() * TestFun(x)));
  ASSERT_TRUE(gnApproximation.dfdxx.isApprox(TestJacobian(x).transpose() * TestJacobian(x)));
}

TEST_F(CppAdInterfaceParameterizedFixture, testModelGeneration) {
  bolza::CppAdInterface adInterface(funImpl, variableDim_, parameterDim_, "testModelWithParameters");

  adInterface.CreateModels(bolza::CppAdInterface::ApproximationOrder::Second, true);
  vector_t x = vector_t::Random(variableDim_);
  vector_t p = vector_t::Random(parameterDim_);

  ASSERT_TRUE(adInterface.GetFunctionValue(x, p).isApprox(TestFun(x, p)));
  ASSERT_TRUE(adInterface.GetJacobian(x, p).isApprox(TestJacobian(x, p)));
  ASSERT_TRUE(adInterface.GetHessian(0, x, p).isApprox(TestHessian(0, x, p)));
  ASSERT_TRUE(adInterface.GetHessian(1, x, p).isApprox(TestHessian(1, x, p)));

  const auto gnApproximation = adInterface.GetGaussNewtonApproximation(x, p);
  ASSERT_DOUBLE_EQ(gnApproximation.f, 0.5 * TestFun(x, p).squaredNorm());
  ASSERT_TRUE(gnApproximation.dfdx.isApprox(TestJacobian(x, p).transpose() * TestFun(x, p)));
  ASSERT_TRUE(gnApproximation.dfdxx.isApprox(TestJacobian(x, p).transpose() * TestJacobian(x, p)));
}

TEST_F(CppAdInterfaceParameterizedFixture, loadIfAvailable) {
  bolza::CppAdInterface adInterface(funImpl, variableDim_, parameterDim_, "testModelLoadIfAvailable");

  adInterface.LoadModelsIfAvailable(bolza::CppAdInterface::ApproximationOrder::Second, true);
  vector_t x = vector_t::Random(variableDim_);
  vector_t p = vector_t::Random(parameterDim_);

  ASSERT_TRUE(adInterface.GetFunctionValue(x, p).isApprox(TestFun(x, p)));
  ASSERT_TRUE(adInterface.GetJacobian(x, p).isApprox(TestJacobian(x, p)));
  ASSERT_TRUE(adInterface.GetHessian(0, x, p).isApprox(TestHessian(0, x, p)));
  ASSERT_TRUE(adInterface.GetHessian(1, x, p).isApprox(TestHessian(1, x, p)));

  const auto gnApproximation = adInterface.GetGaussNewtonApproximation(x, p);
  ASSERT_DOUBLE_EQ(gnApproximation.f, 0.5 * TestFun(x, p).squaredNorm());
  ASSERT_TRUE(gnApproximation.dfdx.isApprox(TestJacobian(x, p).transpose() * TestFun(x, p)));
  ASSERT_TRUE(gnApproximation.dfdxx.isApprox(TestJacobian(x, p).transpose() * TestJacobian(x, p)));
}
