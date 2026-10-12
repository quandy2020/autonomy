/*
 * Copyright 2026 Bolza contributors duyongquan (quandy2020@126.com)
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

/**
 * @file solve_robot.cpp
 * @brief Terminal OCP demos for examples/assets, solved with libbolza only.
 *
 * Physics demos: double integrator, cart-pole, quadrotor.
 * Every URDF with nv > 0 also gets a joint-velocity tracking solve (qdot = u).
 */

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>

#include <assets/package_path.hpp>
#include <bolza/core/dynamics/controlled_system_base.hpp>
#include <bolza/core/dynamics/system_dynamics_linearizer.hpp>
#include <bolza/pinocchio/interface/urdf.hpp>
#include <bolza/solver.hpp>
#include <bolza/tools/common/angular_velocity_mapping.hpp>

namespace {

using bolza::matrix_t;
using bolza::scalar_t;
using bolza::vector_t;

class FlowMapDynamics final : public bolza::ControlledSystemBase {
 public:
  using Flow = std::function<vector_t(const vector_t&, const vector_t&)>;

  explicit FlowMapDynamics(Flow flow) : flow_(std::move(flow)) {}

  FlowMapDynamics* Clone() const override { return new FlowMapDynamics(*this); }

  vector_t ComputeFlowMap(scalar_t, const vector_t& state, const vector_t& input, const bolza::PreComputation&) override {
    return flow_(state, input);
  }

 private:
  Flow flow_;
};

struct PrintOptions {
  int state_cols = 6;
  int input_cols = 4;
};

bool SolveAndPrint(const std::string& title, bolza::Problem* problem, const bolza::Solver::Options& options,
                   const vector_t& x_star, PrintOptions columns = {}, double tolerance = 5e-2) {
  bolza::Solver::Summary summary;
  bolza::Solve(options, problem, &summary);
  const double error = summary.ok ? (summary.x_final - x_star).norm() : -1.0;

  std::cout << std::fixed << std::setprecision(4);
  std::cout << title << "\n";
  std::cout << "  solve ok=" << std::boolalpha << summary.ok << std::noboolalpha << "  " << summary.message
            << "  samples=" << summary.time_trajectory.size() << "  |x(tf)-x*|=" << error << '\n';
  if (!summary.ok || summary.time_trajectory.empty()) {
    return false;
  }

  const int nx = static_cast<int>(summary.x_final.size());
  const int nu = summary.input_trajectory.empty() ? 0 : static_cast<int>(summary.input_trajectory.front().size());
  const int sx = std::min(nx, columns.state_cols);
  const int su = std::min(nu, columns.input_cols);
  const auto samples = summary.time_trajectory.size();
  const std::size_t stride = std::max<std::size_t>(1, samples / 4);

  auto print_row = [&](std::size_t k) {
    std::cout << "  t=" << std::setw(6) << summary.time_trajectory[k] << "  x";
    for (int i = 0; i < sx; ++i) {
      std::cout << ' ' << std::setw(8) << summary.state_trajectory[k](i);
    }
    if (sx < nx) {
      std::cout << " ...";
    }
    std::cout << "  u";
    for (int i = 0; i < su; ++i) {
      std::cout << ' ' << std::setw(8) << summary.input_trajectory[k](i);
    }
    if (su < nu) {
      std::cout << " ...";
    }
    std::cout << '\n';
  };
  for (std::size_t k = 0; k < samples; k += stride) {
    print_row(k);
  }
  if (samples > 1 && (samples - 1) % stride != 0) {
    print_row(samples - 1);
  }
  return error < tolerance;
}

void AddQuadratic(bolza::Problem* problem, int nx, int nu, double r_weight, double qf_scale) {
  problem->AddQuadraticCost(matrix_t::Identity(nx, nx), matrix_t::Identity(nu, nu) * r_weight);
  problem->AddQuadraticFinalCost(qf_scale * matrix_t::Identity(nx, nx));
}

bool SolveDoubleIntegrator() {
  std::cout << "\n==== double_integrator ====\n";
  std::cout << "  asset " << bolza::robotic_assets::GetPath() << "/double_integrator/urdf/double_integrator.urdf\n";
  std::cout << "  x=[p, pdot]  u=pddot  target p=1\n";

  bolza::Problem problem;
  const matrix_t A = (matrix_t(2, 2) << 0, 1, 0, 0).finished();
  const matrix_t B = (matrix_t(2, 1) << 0, 1).finished();
  problem.SetLinearDynamics(A, B);
  AddQuadratic(&problem, 2, 1, 0.1, 10.0);
  const vector_t x_star = (vector_t(2) << 1.0, 0.0).finished();
  problem.SetInitialState(vector_t::Zero(2));
  problem.SetTarget(x_star);

  bolza::Solver::Options options;
  options.tf = 3.0;
  options.max_num_iterations = 20;
  return SolveAndPrint("  LTI", &problem, options, x_star, {2, 1});
}

vector_t CartPoleFlow(const vector_t& state, const vector_t& input) {
  constexpr double cart_mass = 1.0;
  constexpr double pole_mass = 1.0;
  constexpr double pole_length = 1.0;
  constexpr double gravity = 9.81;
  const double half = pole_length / 2.0;
  const double steiner = pole_mass * pole_length * pole_length / 12.0 + pole_mass * half * half;
  const double cosine = std::cos(state(0));
  const double sine = std::sin(state(0));

  Eigen::Matrix2d inertia;
  inertia << steiner, pole_mass * half * cosine, pole_mass * half * cosine, cart_mass + pole_mass;
  Eigen::Vector2d rhs;
  rhs << pole_mass * half * gravity * sine, input(0) + pole_mass * half * state(2) * state(2) * sine;
  const Eigen::Vector2d acceleration = inertia.ldlt().solve(rhs);

  vector_t derivative(4);
  derivative << state(2), state(3), acceleration(0), acceleration(1);
  return derivative;
}

bool SolveCartPole() {
  std::cout << "\n==== cartpole ====\n";
  std::cout << "  asset " << bolza::robotic_assets::GetPath() << "/cartpole/urdf/cartpole.urdf\n";
  std::cout << "  x=[theta, cart, theta_dot, cart_dot]  u=force  upright theta=0, cart=0.5\n";

  bolza::Problem problem;
  problem.SetDynamics(std::make_unique<bolza::SystemDynamicsLinearizer>(
      std::make_unique<FlowMapDynamics>(CartPoleFlow), true, false, 1e-5));
  AddQuadratic(&problem, 4, 1, 0.1, 20.0);

  vector_t x0(4);
  x0 << 0.35, 0.0, 0.0, 0.0;
  const vector_t x_star = (vector_t(4) << 0.0, 0.5, 0.0, 0.0).finished();
  problem.SetInitialState(x0);
  problem.SetTarget(x_star, vector_t::Zero(1));

  bolza::Solver::Options options;
  options.tf = 3.0;
  options.max_num_iterations = 30;
  return SolveAndPrint("  nonlinear", &problem, options, x_star, {4, 1}, 0.1);
}

vector_t QuadrotorFlow(const vector_t& state, const vector_t& input) {
  constexpr double mass = 0.5;
  constexpr double thxxyy = 0.0023;
  constexpr double thzz = 0.004;
  constexpr double gravity = 9.81;

  const Eigen::Matrix<scalar_t, 3, 1> euler = state.segment<3>(3);
  const Eigen::Matrix<scalar_t, 3, 3> mapping =
      bolza::GetMappingFromLocalAngularVelocityToEulerAnglesXyzDerivative<scalar_t>(euler);
  const Eigen::Matrix<scalar_t, 3, 1> euler_dot = mapping * state.segment<3>(9);

  const double qph = state(3);
  const double qth = state(4);
  const double qps = state(5);
  const double dqph = state(9);
  const double dqth = state(10);
  const double dqps = state(11);
  const double fz = input(0);
  const double mx = input(1);
  const double my = input(2);
  const double mz = input(3);

  const double t2 = 1.0 / mass;
  const double t3 = std::cos(qth);
  const double t4 = std::sin(qth);
  const double t5 = 1.0 / thxxyy;
  const double t6 = std::cos(qps);
  const double t7 = std::sin(qps);
  const double t8 = dqph * dqph;
  const double t10 = std::sin(2.0 * qth);
  const double t11 = 1.0 / t3;
  const double t12 = thzz * thzz;
  const double t13 = t3 * t3;

  vector_t derivative(12);
  derivative(0) = state(6);
  derivative(1) = state(7);
  derivative(2) = state(8);
  derivative(3) = euler_dot(0);
  derivative(4) = euler_dot(1);
  derivative(5) = euler_dot(2);
  derivative(6) = fz * t2 * t4;
  derivative(7) = -fz * t2 * t3 * std::sin(qph);
  derivative(8) = t2 * (mass * gravity - fz * t3 * std::cos(qph)) * (-1.0);
  derivative(9) = -t5 * t11 *
                   (-mx * t6 + my * t7 + thzz * dqps * dqth - thxxyy * dqph * dqth * t4 * 2.0 + thzz * dqph * dqth * t4);
  derivative(10) = t5 * (mx * t7 + my * t6 - thxxyy * t8 * t10 * 0.5 + thzz * t8 * t10 * 0.5 + thzz * dqph * dqps * t3);
  derivative(11) = (t5 * t11 *
                    (mz * thxxyy * t3 + dqph * dqth * t12 - dqph * dqth * t12 * t13 + dqps * dqth * t4 * t12 -
                     thxxyy * thzz * dqph * dqth * 2.0 - mx * thzz * t4 * t6 + my * thzz * t4 * t7 +
                     thxxyy * thzz * dqph * dqth * t13)) /
                   thzz;
  return derivative;
}

void Linearize(const std::function<vector_t(const vector_t&, const vector_t&)>& flow, const vector_t& x0, const vector_t& u0,
               matrix_t* A, matrix_t* B) {
  constexpr double eps = 1e-6;
  const int nx = static_cast<int>(x0.size());
  const int nu = static_cast<int>(u0.size());
  *A = matrix_t::Zero(nx, nx);
  *B = matrix_t::Zero(nx, nu);
  for (int i = 0; i < nx; ++i) {
    vector_t plus = x0;
    vector_t minus = x0;
    plus(i) += eps;
    minus(i) -= eps;
    A->col(i) = (flow(plus, u0) - flow(minus, u0)) / (2.0 * eps);
  }
  for (int i = 0; i < nu; ++i) {
    vector_t plus = u0;
    vector_t minus = u0;
    plus(i) += eps;
    minus(i) -= eps;
    B->col(i) = (flow(x0, plus) - flow(x0, minus)) / (2.0 * eps);
  }
}

bool SolveQuadrotor() {
  std::cout << "\n==== quadrotor ====\n";
  std::cout << "  asset " << bolza::robotic_assets::GetPath() << "/quadrotor/urdf/quadrotor.urdf\n";
  std::cout << "  deviation from hover  x=[p, euler_xyz, v, w]  u=[Fz, Mx, My, Mz]-hover  mass=0.5\n";

  constexpr double mass = 0.5;
  constexpr double gravity = 9.81;
  vector_t hover_state = vector_t::Zero(12);
  hover_state(2) = 1.0;
  vector_t hover_input(4);
  hover_input << mass * gravity, 0.0, 0.0, 0.0;
  const double hover_residual = QuadrotorFlow(hover_state, hover_input).norm();
  std::cout << "  |f(hover)|=" << hover_residual << '\n';

  matrix_t A;
  matrix_t B;
  Linearize(QuadrotorFlow, hover_state, hover_input, &A, &B);

  bolza::Problem problem;
  problem.SetLinearDynamics(A, B);
  matrix_t Q = matrix_t::Identity(12, 12);
  Q.diagonal().head<3>() *= 5.0;
  problem.AddQuadraticCost(Q, matrix_t::Identity(4, 4) * 0.2);
  problem.AddQuadraticFinalCost(8.0 * Q);

  vector_t x_star = vector_t::Zero(12);
  x_star(0) = 0.3;
  x_star(2) = 0.2;
  problem.SetInitialState(vector_t::Zero(12));
  problem.SetTarget(x_star);

  bolza::Solver::Options options;
  options.tf = 2.0;
  options.max_num_iterations = 20;
  const bool reached = SolveAndPrint("  hover linearization", &problem, options, x_star, {6, 4});
  return reached && hover_residual < 1e-6;
}

std::string ReadText(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot read " + path.string());
  }
  return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

std::string SanitizeEffortLimits(std::string xml) {
  const std::pair<const char*, const char*> replacements[] = {
      {"effort=\"-1.0\"", "effort=\"10\""},
      {"velocity=\"-1.0\"", "velocity=\"10\""},
  };
  for (const auto& [from, to] : replacements) {
    const std::string source(from);
    for (std::size_t at = xml.find(source); at != std::string::npos; at = xml.find(source, at + 1)) {
      xml.replace(at, source.size(), to);
    }
  }
  return xml;
}

bool SolveUrdfKinematics(const std::filesystem::path& urdf) {
  const std::string relative = std::filesystem::relative(urdf, bolza::robotic_assets::GetPath()).generic_string();
  std::cout << "\n==== urdf " << relative << " ====\n";

  bolza::PinocchioInterface interface = [&] {
    try {
      return bolza::GetPinocchioInterfaceFromUrdfFile(urdf.string());
    } catch (const std::exception& error) {
      std::cout << "  urdf parse retry (" << error.what() << ")\n";
      return bolza::GetPinocchioInterfaceFromUrdfString(SanitizeEffortLimits(ReadText(urdf)));
    }
  }();
  const auto& model = interface.GetModel();
  const int n = static_cast<int>(model.nv);
  if (n <= 0) {
    std::cout << "  model " << model.name << " has nv=0, skipped (no velocity coordinate)\n";
    return true;
  }

  std::vector<std::string> joints;
  for (std::size_t i = 1; i < model.names.size(); ++i) {
    if (model.joints[i].nv() > 0) {
      joints.push_back(model.names[i]);
    }
  }
  std::cout << "  model " << model.name << "  nq=" << model.nq << "  nv=" << n << "  joints:";
  const int shown = std::min<int>(static_cast<int>(joints.size()), 8);
  for (int i = 0; i < shown; ++i) {
    std::cout << ' ' << joints[static_cast<size_t>(i)];
  }
  if (static_cast<int>(joints.size()) > shown) {
    std::cout << " ...";
  }
  std::cout << "\n  x=q  u=qdot\n";

  bolza::Problem problem;
  problem.SetLinearDynamics(matrix_t::Zero(n, n), matrix_t::Identity(n, n));
  AddQuadratic(&problem, n, n, 0.05, 10.0);

  vector_t x_star = vector_t::Zero(n);
  for (int i = 0; i < n; ++i) {
    x_star(i) = 0.15 * ((i % 2 == 0) ? 1.0 : -1.0);
  }
  problem.SetInitialState(vector_t::Zero(n));
  problem.SetTarget(x_star);

  bolza::Solver::Options options;
  options.tf = 1.0;
  options.max_num_iterations = 15;
  return SolveAndPrint("  kinematic", &problem, options, x_star, {4, 4});
}

}  // namespace

int main() {
  const auto root = std::filesystem::path(bolza::robotic_assets::GetPath());
  std::cout << "assets " << root.generic_string() << '\n';

  bool ok = true;
  ok = SolveDoubleIntegrator() && ok;
  ok = SolveCartPole() && ok;
  ok = SolveQuadrotor() && ok;

  std::vector<std::filesystem::path> urdfs;
  for (const auto& entry : std::filesystem::recursive_directory_iterator(root)) {
    if (entry.is_regular_file() && entry.path().extension() == ".urdf") {
      urdfs.push_back(entry.path());
    }
  }
  std::sort(urdfs.begin(), urdfs.end());
  for (const auto& urdf : urdfs) {
    try {
      ok = SolveUrdfKinematics(urdf) && ok;
    } catch (const std::exception& error) {
      std::cout << "  failed: " << error.what() << '\n';
      ok = false;
    }
  }

  std::cout << "\n" << (ok ? "all demos reached their targets" : "one or more demos missed the target") << '\n';
  return ok ? 0 : 1;
}
