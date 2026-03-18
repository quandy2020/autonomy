#include "openbot/common/math/eigen_alignment.hpp"
#include "openbot/common/utils/logging.hpp"

#include <benchmark/benchmark.h>
#include <ceres/ceres.h>

class BM_ReprojErrorConstantPoint3DCostFunction : public benchmark::Fixture {
 public:
  void SetUp(::benchmark::State& state) 
  {
 
  }
};

BENCHMARK_F(BM_ReprojErrorConstantPoint3DCostFunction, Run)
(benchmark::State& state) {
  for (auto _ : state) {
    cost_function->Evaluate(parameters, residuals, jacobians);
  }
}

BENCHMARK_MAIN();
