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

#pragma once

/**
 * @file cpp_ad_interface.hpp
 * @brief Constructor for parameterized functions.
 */


// Eigen
#include <Eigen/Core>

// STL
#include <string>

// CppAD
#include <cppad/cg.hpp>

// CppAD helpers
#include <bolza/core/types.hpp>
#include <bolza/core/automatic_differentiation/cpp_ad_sparsity.hpp>
#include <bolza/core/automatic_differentiation/types.hpp>

namespace bolza {

class CppAdInterface {
 public:
  enum class ApproximationOrder { Zero, First, Second };

  using ad_base_t = bolza::ad_base_t;
  using ad_scalar_t = bolza::ad_scalar_t;
  using ad_vector_t = bolza::ad_vector_t;
  using ad_function_t = std::function<void(const ad_vector_t&, ad_vector_t&)>;
  using ad_parameterized_function_t = std::function<void(const ad_vector_t&, const ad_vector_t&, ad_vector_t&)>;
  using ad_fun_t = CppAD::ADFun<ad_base_t>;

  /**
   * Constructor for parameterized functions
   *
   * @param [in] adFunction: parameterized function y = f(x,p)
   * @param [in] variableDim: size of x
   * @param [in] parameterDim: size of p
   * @param [in] modelName: Name of the library to be generated.
   * @param [in] folderName: Folder to save library files to, either absolute of relative
   * @param [in] compileFlags: Compilation flags for the model library.
   */
  CppAdInterface(ad_parameterized_function_t adFunction, size_t variableDim, size_t parameterDim, std::string modelName,
                 std::string folderName = "/tmp/ocs2",
                 std::vector<std::string> compileFlags = {"-O3", "-g", "-march=native", "-mtune=native", "-ffast-math"});

  /**
   * Constructor for functions without parameters
   *
   * @param [in] adFunction: function y = f(x)
   * @param [in] variableDim: size of x
   * @param [in] modelName: Name of the library to be generated.
   * @param [in] folderName: Folder to save library files to, either absolute of relative
   * @param [in] compileFlags: Compilation flags for the model library.
   */
  CppAdInterface(ad_function_t adFunction, size_t variableDim, std::string modelName, std::string folderName = "/tmp/ocs2",
                 std::vector<std::string> compileFlags = {"-O3", "-g", "-march=native", "-mtune=native", "-ffast-math"});

  /**
   * @brief Destructor.
   */
  ~CppAdInterface() = default;

    /**
   * @brief.
   *
   * Copy constructor. Models are reloaded if available.
   *
   * /
   * @param [in] rhs: Rhs.
   */
  CppAdInterface(const CppAdInterface& rhs);

  //! Not implemented
  CppAdInterface& operator=(const CppAdInterface& rhs) = delete;
  /**
   * @brief Constructs a CppAdInterface.
   * @param [in,out] rhs: Rhs.
   */
  CppAdInterface(CppAdInterface&& rhs) = delete;
  CppAdInterface& operator=(CppAdInterface&& rhs) = delete;

    /**
   * @brief.
   *
   * Loads earlier created model from disk
   *
   * /
   * @param [in] verbose: Verbose.
   */
  void LoadModels(bool verbose = true);

  /**
   * Creates models, compiles them, and saves them to disk
   *
   * @param [in] approximationOrder: Order of derivatives to generate
   * @param [in] verbose: Print out extra information
   */
  void CreateModels(ApproximationOrder approximationOrder = ApproximationOrder::Second, bool verbose = true);

  /**
   * Load models if they are available on disk. Creates a new library otherwise.
   *
   * @param [in] approximationOrder: Order of derivatives to generate
   * @param [in] verbose: Print out extra information
   */
  void LoadModelsIfAvailable(ApproximationOrder approximationOrder = ApproximationOrder::Second, bool verbose = true);

  /**
   * @brief Get Function Value.
   * @param [in] x: input vector of size variableDim
   * @param [in] p: parameter vector of size parameterDim
   * @return y = f(x,p)
   */
  vector_t GetFunctionValue(const vector_t& x, const vector_t& p = vector_t(0)) const;

  /**
   * Jacobian with gradient of each output w.r.t the variables x in the rows.
   *
   * @param [in] x: input vector of size variableDim
   * @param [in] p: parameter vector of size parameterDim
   * @return d/dx( f(x,p) )
   */
  matrix_t GetJacobian(const vector_t& x, const vector_t& p = vector_t(0)) const;

  /**
   * Returns the full Gauss-Newton approximation of the function.
   * With auto differentiated function y = f(x,p), the following approximation is made:
   *
   * Value (scalar) : 0.5 * |y|^2
   * first derivative : dy/dx' * y
   * second derivative : dy/dx' * dy/dx
   *
   * @param [in] x: input vector of size variableDim
   * @param [in] p: parameter vector of size parameterDim
   * @return Quadratic approximation with the values stored in f, dfdx, dfdxx.
   */
  ScalarFunctionQuadraticApproximation GetGaussNewtonApproximation(const vector_t& x, const vector_t& p = vector_t(0)) const;

  /**
   * Hessian, available per output.
   *
   * @param [in] outputIndex: Output to get the hessian for.
   * @param [in] x: input vector of size variableDim
   * @param [in] p: parameter vector of size parameterDim
   * @return dd/dxdx( f_i(x,p) )
   */
  matrix_t GetHessian(size_t outputIndex, const vector_t& x, const vector_t& p = vector_t(0)) const;

  /**
   * Weighted hessian
   *
   * @param [in] w: vector of weights of size rangeDim
   * @param [in] x: input vector of size variableDim
   * @param [in] p: parameter vector of size parameterDim
   * @return dd/dxdx(sum_i  w_i*f_i(x,p) )
   */
  matrix_t GetHessian(const vector_t& w, const vector_t& x, const vector_t& p = vector_t(0)) const;

 private:
  /**
   * Defines library folder names
   */
  void SetFolderNames();

  /**
   * Creates folders on disk
   */
  void CreateFolderStructure() const;

  /**
   * Checks if library can already be found on disk.
   * @return isLibraryAvailable
   */
  bool IsLibraryAvailable() const;

  /**
   * Creates a random temporary folder name
   * @return folder name
   */
  std::string GetUniqueTemporaryName() const;

  /**
   * Configures the compiler that compiles the model library
   * @param [in] compiler: compiler to be configured
   */
  void SetCompilerOptions(CppAD::cg::GccCompiler<scalar_t>& compiler) const;

    /**
   * @brief.
   *
   * Configure the approximation order for the source generator
   *
   * /
   * @param [in] approximationOrder: Approximation Order.
   * @param [in] sourceGen: Source Gen.
   * @param [in] fun: Fun.
   */
  void SetApproximationOrder(ApproximationOrder approximationOrder, CppAD::cg::ModelCSourceGen<scalar_t>& sourceGen, ad_fun_t& fun) const;

  /**
   * Stores the sparisty nonzeros
   */
  void SetSparsityNonzeros();

  /**
   * Creates sparsity pattern for the Jacobian that will be generated
   * @param [in] fun: taped ad function
   * @return Sparsity pattern that contains entries for variables only, not for parameters
   */
  cppad_sparsity::SparsityPattern CreateJacobianSparsity(ad_fun_t& fun) const;

  /**
   * Creates sparsity pattern for the Hessian that will be generated
   * @param [in] fun: taped ad function
   * @return Sparsity pattern that contains entries for variables only, not for parameters
   */
  cppad_sparsity::SparsityPattern CreateHessianSparsity(ad_fun_t& fun) const;

  std::unique_ptr<CppAD::cg::DynamicLib<scalar_t>> dynamicLib_;
  std::unique_ptr<CppAD::cg::GenericModel<scalar_t>> model_;
  ad_parameterized_function_t adFunction_;
  std::vector<std::string> compileFlags_;

  // Sizes
  size_t variableDim_;
  size_t parameterDim_;
  size_t rangeDim_ = 0;
  size_t nnzJacobian_ = 0;
  size_t nnzHessian_ = 0;

  // Names
  std::string modelName_;
  std::string folderName_;
  std::string libraryFolder_;
  std::string tmpName_;
  std::string tmpFolder_;
  std::string libraryName_;
};

}  // namespace bolza
