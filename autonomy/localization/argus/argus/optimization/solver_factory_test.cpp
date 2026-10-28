#include "optimization/solver_factory.hpp"

#include <gtest/gtest.h>

namespace argus {
namespace optimization {
namespace {

TEST(SolverFactory, EigenIsAlwaysAvailable) {
  EXPECT_TRUE(SolverAvailable(SolverKind::kEigen));
  const std::unique_ptr<GraphSolver> solver = CreateSolver(SolverKind::kEigen);
  ASSERT_NE(solver, nullptr);
  EXPECT_STREQ(solver->name(), "eigen");
}

TEST(SolverFactory, GtsamIsInstalled) {
  EXPECT_TRUE(SolverAvailable(SolverKind::kGtsam));
  const std::unique_ptr<GraphSolver> solver = CreateSolver(SolverKind::kGtsam);
  ASSERT_NE(solver, nullptr);
  EXPECT_STREQ(solver->name(), "gtsam");
}

TEST(SolverFactory, MissingG2oDoesNotYieldASolver) {
  if (!SolverAvailable(SolverKind::kG2o)) {
    EXPECT_EQ(CreateSolver(SolverKind::kG2o), nullptr);
  }
}

}  // namespace
}  // namespace optimization
}  // namespace argus
