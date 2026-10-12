#include <gtest/gtest.h>

#include <memory>

#include <bolza/core/types.hpp>
#include <bolza/core/dynamics/linear_system_dynamics.hpp>
#include <bolza/core/dynamics/system_dynamics_base.hpp>

using namespace bolza;

class DummyPreComputation final : public PreComputation {
 public:
  DummyPreComputation() = default;
  PreComputation* Clone() const override { return new DummyPreComputation(*this); }

  void Request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override { lastRequest = request; }

  void RequestPreJump(RequestSet request, scalar_t t, const vector_t& x) override { lastRequest = request; }

  static void Reset() { lastRequest = RequestSet(static_cast<Request>(0)); }

  static RequestSet lastRequest;
};

RequestSet DummyPreComputation::lastRequest = RequestSet(static_cast<Request>(0));

class DummySystem final : public SystemDynamicsBase {
 public:
  DummySystem() : SystemDynamicsBase(DummyPreComputation()) {}
  ~DummySystem() override = default;
  DummySystem* Clone() const override { return new DummySystem(*this); }

  vector_t ComputeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) override {
    vector_t dfdt(2);
    dfdt << x(1), u(0);
    return dfdt;
  }

  VectorFunctionLinearApproximation LinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                        const PreComputation& preComp) override {
    VectorFunctionLinearApproximation linearDynamics;
    linearDynamics.f = ComputeFlowMap(t, x, u, preComp);
    linearDynamics.dfdx.resize(2, 2);
    linearDynamics.dfdx << 0, 1,  // clang-format off
                           0, 0;  // clang-format on
    linearDynamics.dfdu.resize(2, 1);
    linearDynamics.dfdu << 0, 1;
    return linearDynamics;
  }

  using ControlledSystemBase::ComputeFlowMap;
  using SystemDynamicsBase::LinearApproximation;
};

TEST(testSystemDynamicsPreComputation, testIntermediateCallback) {
  DummySystem system;

  const scalar_t t = 0.0;
  const vector_t x = vector_t::Zero(2);
  const vector_t u = vector_t::Zero(1);

  DummyPreComputation::Reset();
  EXPECT_FALSE(DummyPreComputation::lastRequest.Contains(Request::Dynamics));
  EXPECT_FALSE(DummyPreComputation::lastRequest.Contains(Request::Approximation));

  const auto flowMap = system.ControlledSystemBase::ComputeFlowMap(t, x, u);
  EXPECT_TRUE(DummyPreComputation::lastRequest.Contains(Request::Dynamics));
  EXPECT_FALSE(DummyPreComputation::lastRequest.Contains(Request::Approximation));

  DummyPreComputation::Reset();
  EXPECT_FALSE(DummyPreComputation::lastRequest.Contains(Request::Dynamics));
  EXPECT_FALSE(DummyPreComputation::lastRequest.Contains(Request::Approximation));
  const auto flowMapApproximation = system.SystemDynamicsBase::LinearApproximation(t, x, u);
  EXPECT_TRUE(DummyPreComputation::lastRequest.Contains(Request::Dynamics));
  EXPECT_TRUE(DummyPreComputation::lastRequest.Contains(Request::Approximation));
}

TEST(testSystemDynamicsPreComputation, testPreJumpCallback) {
  DummySystem system;

  const scalar_t t = 0.0;
  const vector_t x = vector_t::Zero(2);

  DummyPreComputation::Reset();
  EXPECT_FALSE(DummyPreComputation::lastRequest.Contains(Request::Dynamics));
  EXPECT_FALSE(DummyPreComputation::lastRequest.Contains(Request::Approximation));

  const auto jumpMap = system.ComputeJumpMap(t, x);
  EXPECT_TRUE(DummyPreComputation::lastRequest.Contains(Request::Dynamics));
  EXPECT_FALSE(DummyPreComputation::lastRequest.Contains(Request::Approximation));

  DummyPreComputation::Reset();
  EXPECT_FALSE(DummyPreComputation::lastRequest.Contains(Request::Dynamics));
  EXPECT_FALSE(DummyPreComputation::lastRequest.Contains(Request::Approximation));

  const auto jumpMapApproximation = system.JumpMapLinearApproximation(t, x);
  EXPECT_TRUE(DummyPreComputation::lastRequest.Contains(Request::Dynamics));
  EXPECT_TRUE(DummyPreComputation::lastRequest.Contains(Request::Approximation));
}

TEST(testSystemDynamicsPreComputation, testPreComputationRequestLogic) {
  constexpr RequestSet a = Request::Cost + Request::Constraint;

  // Test for individual flags
  EXPECT_TRUE(a.Contains(Request::Cost));
  EXPECT_TRUE(a.Contains(Request::Constraint));
  EXPECT_FALSE(a.Contains(Request::Approximation));
  EXPECT_FALSE(a.Contains(Request::SoftConstraint));
  EXPECT_FALSE(a.Contains(Request::Dynamics));
  // a.Contains(int(42)); // this should not compile
  // a.Contains(Request(Request::Cost)); // this should not compile

  constexpr RequestSet b = Request::Cost + Request::Approximation;
  constexpr RequestSet c = a + b;  // union

  // Test for subset
  EXPECT_TRUE(c.ContainsAll(a));
  EXPECT_TRUE(c.ContainsAll(b));
  EXPECT_TRUE(c.ContainsAll(c));
  EXPECT_FALSE(a.ContainsAll(b));
  EXPECT_FALSE(b.ContainsAll(a));

  // Test for non-empty intersection
  EXPECT_TRUE(c.ContainsAny(a));
  EXPECT_TRUE(c.ContainsAny(b));
  EXPECT_TRUE(c.ContainsAny(c));
  EXPECT_TRUE(a.ContainsAny(b));
  EXPECT_TRUE(b.ContainsAny(a));

  EXPECT_FALSE(c.Contains(Request::Dynamics));
  EXPECT_FALSE(c.Contains(Request::SoftConstraint));

  // Add same flag twice
  EXPECT_TRUE((Request::Dynamics + Request::Dynamics).Contains(Request::Dynamics));
}
