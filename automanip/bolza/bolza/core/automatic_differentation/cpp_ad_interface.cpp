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

#include <bolza/core/automatic_differentiation/cpp_ad_interface.hpp>
#include <filesystem>


namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CppAdInterface::CppAdInterface(ad_parameterized_function_t adFunction, size_t variableDim, size_t parameterDim, std::string modelName,
                               std::string folderName, std::vector<std::string> compileFlags)
    : adFunction_(std::move(adFunction)),
      variableDim_(variableDim),
      parameterDim_(parameterDim),
      modelName_(std::move(modelName)),
      folderName_(std::move(folderName)),
      compileFlags_(std::move(compileFlags)) {
  SetFolderNames();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CppAdInterface::CppAdInterface(ad_function_t adFunction, size_t variableDim, std::string modelName, std::string folderName,
                               std::vector<std::string> compileFlags)
    : CppAdInterface([adFunction](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) { adFunction(x, y); }, variableDim, 0,
                     std::move(modelName), std::move(folderName), std::move(compileFlags)){};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CppAdInterface::CppAdInterface(const CppAdInterface& rhs)
    : CppAdInterface(rhs.adFunction_, rhs.variableDim_, rhs.parameterDim_, rhs.modelName_, rhs.folderName_, rhs.compileFlags_) {
  if (IsLibraryAvailable()) {
    LoadModels(false);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CppAdInterface::CreateModels(ApproximationOrder approximationOrder, bool verbose) {
  CreateFolderStructure();

  // set and declare independent variables and start tape recording
  ad_vector_t xp(variableDim_ + parameterDim_);
  xp.setOnes();  // Ones are better than zero, to prevent devision by zero in taping
  CppAD::Independent(xp);

  // Split in variables and parameters
  ad_vector_t x = xp.segment(0, variableDim_);
  ad_vector_t p = xp.segment(variableDim_, parameterDim_);
  // dependent variable vector
  ad_vector_t y;
  // the model equation
  adFunction_(x, p, y);
  rangeDim_ = y.rows();
  // create f: xp -> y and stop tape recording
  ad_fun_t fun(xp, y);
  // Optimize the operation sequence
  fun.optimize();

  // generates source code
  CppAD::cg::ModelCSourceGen<scalar_t> sourceGen(fun, modelName_);
  SetApproximationOrder(approximationOrder, sourceGen, fun);

  // Compiler objects, compile to temporary shared library file to avoid interference between processes
  CppAD::cg::ModelLibraryCSourceGen<scalar_t> libraryCSourceGen(sourceGen);
  CppAD::cg::GccCompiler<scalar_t> gccCompiler;
  CppAD::cg::DynamicModelLibraryProcessor<scalar_t> libraryProcessor(libraryCSourceGen, libraryName_ + tmpName_);
  SetCompilerOptions(gccCompiler);

  if (verbose) {
    std::cerr << "[CppAdInterface] Compiling Shared Library: "
              << libraryName_ + tmpName_ + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION << std::endl;
  }

  // Compile and store the library
  dynamicLib_ = libraryProcessor.createDynamicLibrary(gccCompiler);
  model_ = dynamicLib_->model(modelName_);

  SetSparsityNonzeros();

  // Rename generated library after loading
  if (verbose) {
    std::cerr << "[CppAdInterface] Renaming " << libraryName_ + tmpName_ + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION << " to "
              << libraryName_ + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION << std::endl;
  }
  std::filesystem::rename(libraryName_ + tmpName_ + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION,
                            libraryName_ + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CppAdInterface::LoadModels(bool verbose) {
  if (verbose) {
    std::cerr << "[CppAdInterface] Loading Shared Library: " << libraryName_ + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION
              << std::endl;
  }
  dynamicLib_.reset(new CppAD::cg::LinuxDynamicLib<scalar_t>(libraryName_ + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION));
  model_ = dynamicLib_->model(modelName_);
  rangeDim_ = model_->Range();

  SetSparsityNonzeros();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CppAdInterface::LoadModelsIfAvailable(ApproximationOrder approximationOrder, bool verbose) {
  if (IsLibraryAvailable()) {
    LoadModels(verbose);
  } else {
    CreateModels(approximationOrder, verbose);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t CppAdInterface::GetFunctionValue(const vector_t& x, const vector_t& p) const {
  vector_t xp(variableDim_ + parameterDim_);
  xp << x, p;

  vector_t functionValue(model_->Range());

  model_->ForwardZero(xp, functionValue);
  assert(functionValue.allFinite());
  return functionValue;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t CppAdInterface::GetJacobian(const vector_t& x, const vector_t& p) const {
  // Concatenate input
  vector_t xp(variableDim_ + parameterDim_);
  xp << x, p;
  CppAD::cg::ArrayView<scalar_t> xpArrayView(xp.data(), xp.size());

  std::vector<scalar_t> sparseJacobian(nnzJacobian_);
  CppAD::cg::ArrayView<scalar_t> sparseJacobianArrayView(sparseJacobian);
  size_t const* rows;
  size_t const* cols;
  // Call this particular SparseJacobian. Other CppAd functions allocate internal vectors that are incompatible with multithreading.
  model_->SparseJacobian(xpArrayView, sparseJacobianArrayView, &rows, &cols);

  // Write sparse elements into Eigen type. Only jacobian w.r.t. variables was requested, so cols should not contain elements corresponding
  // to parameters.
  matrix_t jacobian = matrix_t::Zero(model_->Range(), variableDim_);
  for (size_t i = 0; i < nnzJacobian_; i++) {
    jacobian(rows[i], cols[i]) = sparseJacobian[i];
  }

  assert(jacobian.allFinite());
  return jacobian;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation CppAdInterface::GetGaussNewtonApproximation(const vector_t& x, const vector_t& p) const {
  // Concatenate input
  vector_t xp(variableDim_ + parameterDim_);
  xp << x, p;
  CppAD::cg::ArrayView<scalar_t> xpArrayView(xp.data(), xp.size());

  ScalarFunctionQuadraticApproximation gnApprox;

  // Zero order
  vector_t valueVector(model_->Range());
  model_->ForwardZero(xp, valueVector);
  gnApprox.f = 0.5 * valueVector.squaredNorm();

  // Jacobian
  std::vector<scalar_t> sparseJacobian(nnzJacobian_);
  CppAD::cg::ArrayView<scalar_t> sparseJacobianArrayView(sparseJacobian);
  size_t const* rows;
  size_t const* cols;
  model_->SparseJacobian(xpArrayView, sparseJacobianArrayView, &rows, &cols);

  // Sparse evaluation of J' * f
  gnApprox.dfdx.setZero(variableDim_);
  for (size_t i = 0; i < nnzJacobian_; i++) {
    gnApprox.dfdx(cols[i]) += sparseJacobian[i] * valueVector(rows[i]);
  }

  /*
   * Sparse construction of the GN matrix, H = J' * J.
   * H(i, j) = sum_rows { J(row, i) * J(row, j) }
   * Because the sparse elements are ordered first by row, then by column, we process J row-by-row.
   * For each row of J, we add the non-zero pairs (i, j) to H(i, j).
   */
  gnApprox.dfdxx.setZero(variableDim_, variableDim_);
  for (size_t i = 0; i < nnzJacobian_; ++i) {
    const size_t row_i = rows[i];
    const size_t col_i = cols[i];
    const scalar_t v_i = sparseJacobian[i];
    // Diagonal element always exists:
    gnApprox.dfdxx(col_i, col_i) += v_i * v_i;
    // Process off-diagonals
    size_t j = i + 1;
    while (rows[j] == row_i) {
      const size_t col_j = cols[j];
      gnApprox.dfdxx(col_j, col_i) += v_i * sparseJacobian[j];
      gnApprox.dfdxx(col_i, col_j) = gnApprox.dfdxx(col_j, col_i);  // Maintain symmetry as we go.
      ++j;
    }
  }

  assert(gnApprox.dfdx.allFinite());
  assert(gnApprox.dfdxx.allFinite());
  return gnApprox;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t CppAdInterface::GetHessian(size_t outputIndex, const vector_t& x, const vector_t& p) const {
  vector_t w = vector_t::Zero(rangeDim_);
  w[outputIndex] = 1.0;

  return GetHessian(w, x, p);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t CppAdInterface::GetHessian(const vector_t& w, const vector_t& x, const vector_t& p) const {
  // Concatenate input
  vector_t xp(variableDim_ + parameterDim_);
  xp << x, p;
  CppAD::cg::ArrayView<const scalar_t> xpArrayView(xp.data(), xp.size());

  std::vector<scalar_t> sparseHessian(nnzHessian_);
  CppAD::cg::ArrayView<scalar_t> sparseHessianArrayView(sparseHessian);
  size_t const* rows;
  size_t const* cols;

  CppAD::cg::ArrayView<const scalar_t> wArrayView(w.data(), w.size());

  // Call this particular SparseHessian. Other CppAd functions allocate internal vectors that are incompatible with multithreading.
  model_->SparseHessian(xpArrayView, wArrayView, sparseHessianArrayView, &rows, &cols);

  // Fills upper triangular sparsity of hessian w.r.t variables.
  matrix_t hessian = matrix_t::Zero(variableDim_, variableDim_);
  for (size_t i = 0; i < nnzHessian_; i++) {
    hessian(rows[i], cols[i]) = sparseHessian[i];
  }

  // Copy upper triangular to lower triangular part
  hessian.template triangularView<Eigen::StrictlyLower>() = hessian.template triangularView<Eigen::StrictlyUpper>().transpose();

  assert(hessian.allFinite());
  return hessian;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CppAdInterface::SetFolderNames() {
  if (!folderName_.empty()) {
    libraryFolder_ = folderName_ + "/" + modelName_ + "/cppad_generated";
  } else {
    libraryFolder_ = modelName_ + "/cppad_generated";
  }
  tmpName_ = GetUniqueTemporaryName();
  tmpFolder_ = libraryFolder_ + "/" + tmpName_;
  libraryName_ = libraryFolder_ + "/" + modelName_ + "_lib";
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool CppAdInterface::IsLibraryAvailable() const {
  return std::filesystem::exists(libraryName_ + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CppAdInterface::CreateFolderStructure() const {
  std::filesystem::create_directories(libraryFolder_);
  std::filesystem::create_directories(tmpFolder_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::string CppAdInterface::GetUniqueTemporaryName() const {
  // Random string should be unique for each process and time of calling.
  int randomFromClock = std::chrono::high_resolution_clock::now().time_since_epoch().count() % 1000;
  return std::string("cppadcg_tmp") + std::to_string(randomFromClock) + std::to_string(getpid());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CppAdInterface::SetCompilerOptions(CppAD::cg::GccCompiler<scalar_t>& compiler) const {
  if (!compileFlags_.empty()) {
    // Set compile flags and add required flags for dynamic compilation
    auto compileFlags = compileFlags_;
    compiler.setCompileLibFlags(compileFlags_);
    compiler.addCompileLibFlag("-shared");
    compiler.addCompileLibFlag("-rdynamic");
  }

  compiler.setTemporaryFolder(tmpFolder_);

  // Save sources
  compiler.setSourcesFolder(libraryFolder_);
  compiler.setSaveToDiskFirst(true);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CppAdInterface::SetApproximationOrder(ApproximationOrder approximationOrder, CppAD::cg::ModelCSourceGen<scalar_t>& sourceGen,
                                           ad_fun_t& fun) const {
  switch (approximationOrder) {
    case ApproximationOrder::Second:
      sourceGen.setCreateSparseHessian(true);
      sourceGen.setCustomSparseHessianElements(CreateHessianSparsity(fun));
      // Intentional fall through
    case ApproximationOrder::First:
      sourceGen.setCreateSparseJacobian(true);
      sourceGen.setCustomSparseJacobianElements(CreateJacobianSparsity(fun));
      // Intentional fall through
    case ApproximationOrder::Zero:
      break;
    default:
      throw std::runtime_error("CppAdInterface: Invalid approximation order");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CppAdInterface::SetSparsityNonzeros() {
  if (model_->isJacobianSparsityAvailable()) {
    nnzJacobian_ = cppad_sparsity::GetNumberOfNonZeros(model_->JacobianSparsitySet());
  }
  if (model_->isHessianSparsityAvailable()) {
    nnzHessian_ = cppad_sparsity::GetNumberOfNonZeros(model_->HessianSparsitySet());
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
cppad_sparsity::SparsityPattern CppAdInterface::CreateJacobianSparsity(ad_fun_t& fun) const {
  auto trueSparsity = cppad_sparsity::GetJacobianSparsityPattern(fun);
  auto variableSparsity = cppad_sparsity::GetJacobianVariableSparsity(rangeDim_, variableDim_);
  return cppad_sparsity::GetIntersection(trueSparsity, variableSparsity);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
cppad_sparsity::SparsityPattern CppAdInterface::CreateHessianSparsity(ad_fun_t& fun) const {
  auto trueSparsity = cppad_sparsity::GetHessianSparsityPattern(fun);
  auto variableSparsity = cppad_sparsity::GetHessianVariableSparsity(variableDim_, parameterDim_);
  return cppad_sparsity::GetIntersection(trueSparsity, variableSparsity);
}

}  // namespace bolza
