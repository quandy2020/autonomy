/*
 * Copyright 2026 Automanip contributors duyongquan (quandy2020@126.com)
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
 * @file load_data.hpp
 * @brief Load settings / matrices from INFO files (bolza::property_tree).
 */

#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include <bolza/core/misc/property_tree.hpp>

namespace bolza {
namespace loadData {

template <typename T>
inline void PrintValue(std::ostream& stream, const T& value, const std::string& name,
                       bool updated = true, long printWidth = 80) {
  const std::string nameString = " #### '" + name + "'";
  stream << nameString;
  printWidth = std::max<long>(printWidth, static_cast<long>(nameString.size()) + 15);
  stream.width(printWidth - static_cast<long>(nameString.size()));
  const char fill = stream.fill('.');
  if (updated) {
    stream << value << '\n';
  } else {
    stream << value << " (default)\n";
  }
  stream.fill(fill);
}

template <typename T>
inline void LoadPtreeValue(const property_tree::ptree& pt, T& value, const std::string& name,
                           bool verbose, long printWidth = 80) {
  bool updated = true;
  try {
    value = pt.get<T>(name);
  } catch (const property_tree::PtreeBadPath&) {
    updated = false;
  }
  if (verbose) {
    const std::string nameString = name.substr(name.find_last_of('.') + 1);
    PrintValue(std::cerr, value, nameString, updated, printWidth);
  }
}

template <typename cpp_data_t>
inline void LoadCppDataType(const std::string& filename, const std::string& dataName,
                            cpp_data_t& value) {
  property_tree::ptree pt;
  property_tree::read_info(filename, pt);
  value = pt.get<cpp_data_t>(dataName);
}

template <typename Derived>
inline void LoadEigenMatrix(const std::string& filename, const std::string& matrixName,
                            Eigen::MatrixBase<Derived>& matrix) {
  using scalar_t = typename Eigen::MatrixBase<Derived>::Scalar;
  const size_t rows = static_cast<size_t>(matrix.rows());
  const size_t cols = static_cast<size_t>(matrix.cols());
  if (rows == 0 || cols == 0) {
    throw std::runtime_error("[loadEigenMatrix] Loading empty matrix \"" + matrixName +
                             "\" is not allowed.");
  }

  property_tree::ptree pt;
  property_tree::read_info(filename, pt);

  const scalar_t scaling = pt.get<scalar_t>(matrixName + ".scaling", 1.0);
  const scalar_t defaultValue = pt.get<scalar_t>(matrixName + ".default", 0.0);

  size_t numFailed = 0;
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      scalar_t aij;
      try {
        aij = pt.get<scalar_t>(matrixName + ".(" + std::to_string(i) + "," + std::to_string(j) +
                               ")");
      } catch (const std::exception&) {
        aij = defaultValue;
        numFailed++;
      }
      matrix(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) = scaling * aij;
    }
  }

  if (numFailed == static_cast<size_t>(matrix.size())) {
    throw std::runtime_error("[loadEigenMatrix] Could not load matrix \"" + matrixName +
                             "\" from file \"" + filename + "\".");
  } else if (numFailed > 0) {
    std::cerr << "WARNING: Loaded at least one default value in matrix: \"" + matrixName + "\"\n";
  }
}

template <typename T>
inline void LoadStdVector(const std::string& filename, const std::string& topicName,
                          std::vector<T>& loadVector, bool verbose = true) {
  property_tree::ptree pt;
  property_tree::read_info(filename, pt);

  std::vector<T> backup;
  backup.swap(loadVector);
  loadVector.clear();

  size_t vectorSize = 0;
  while (true) {
    try {
      loadVector.push_back(pt.get<T>(topicName + ".[" + std::to_string(vectorSize) + "]"));
      vectorSize++;
    } catch (const std::exception&) {
      if (vectorSize == 0) {
        loadVector.swap(backup);
      }
      break;
    }
  }

  if (verbose) {
    std::cerr << " #### '" << topicName << "': {";
    if (vectorSize == 0) {
      std::cerr << " }\n";
    } else {
      for (size_t i = 0; i < vectorSize; i++) {
        std::cerr << loadVector[i] << ", ";
      }
      std::cerr << "\b\b}\n";
    }
  }
}

}  // namespace loadData
}  // namespace bolza
