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
 * @file pinocchio_interface.hpp
 * @brief Pinocchio interface class contatining robot model and data.
 */


#include <Eigen/Dense>
#include <iosfwd>
#include <memory>
#include <string>
#include <type_traits>

#include <bolza/core/types.hpp>
#include <bolza/core/automatic_differentiation/types.hpp>
#include <cppad/cg.hpp>

/* Forward declaration of main pinocchio types */
#include "bolza/pinocchio/interface/pinocchio_forward_declaration.hpp"

#include <urdf_model/model.h>

namespace bolza {

template <typename SCALAR>
class PinocchioInterfaceTpl;

using PinocchioInterface = PinocchioInterfaceTpl<scalar_t>;
using PinocchioInterfaceCppAd = PinocchioInterfaceTpl<ad_scalar_t>;

/**
 * Pinocchio interface class contatining robot model and data.
 * The robot model is shared between interface instances.
 */
template <typename SCALAR>
class PinocchioInterfaceTpl final {
 public:
  using Model = pinocchio::ModelTpl<SCALAR, 0, pinocchio::JointCollectionDefaultTpl>;
  using Data = pinocchio::DataTpl<SCALAR, 0, pinocchio::JointCollectionDefaultTpl>;
  using JointModel = pinocchio::JointModelTpl<scalar_t, 0, pinocchio::JointCollectionDefaultTpl>;

  // Template for conditional compilation using SFINAE
  template <typename T>
  using EnableIfScalar_t = typename std::enable_if<std::is_same<T, scalar_t>::Value, bool>::type;

  /**
   * @brief Construct from given pinocchio model.
   * @param [in] model: pinocchio model
   * @param [in] urdfModelPtr: Urdf Model Ptr.
   */
  explicit PinocchioInterfaceTpl(const Model& model, const std::shared_ptr<const ::urdf::ModelInterface> urdfModelPtr =
                                                         std::shared_ptr<const ::urdf::ModelInterface>());

  /** Destructor */
  ~PinocchioInterfaceTpl();

  /**
   * @brief Copy constructor.
   * @param [in] rhs: Rhs.
   */
  PinocchioInterfaceTpl(const PinocchioInterfaceTpl& rhs);

  /**
   * @brief Move constructor.
   * @param [in,out] rhs: Rhs.
   */
  PinocchioInterfaceTpl(PinocchioInterfaceTpl&& rhs);

  /** Copy assignment operator */
  PinocchioInterfaceTpl& operator=(const PinocchioInterfaceTpl& rhs);

  /** Move assignment */
  PinocchioInterfaceTpl<SCALAR>& operator=(PinocchioInterfaceTpl&& rhs);

  /**
   * @brief Get the pinocchio model.
   * @return Result value.
   */
  const Model& GetModel() const { return *robotModelPtr_; }

  /**
   * @brief Get the pinocchio data.
   * @return Result value.
   */
  Data& GetData() { return *robotDataPtr_; }
  /**
   * @brief Get Data.
   * @return Result value.
   */
  const Data& GetData() const { return *robotDataPtr_; }

  /**
   * @brief Get the urdf model.
   * @return Owning pointer.
   */
  const std::shared_ptr<const ::urdf::ModelInterface>& GetUrdfModelPtr() const { return urdfModelPtr_; }

  /** Cast pinocchio interface to CppAD scalar type. */
  template <typename T = SCALAR, EnableIfScalar_t<T> = true>
  /**
   * @brief To Cpp Ad.
   * @return Result value.
   */
  PinocchioInterfaceCppAd ToCppAd() const;

  friend std::ostream& operator<<(std::ostream& os, const PinocchioInterfaceTpl<scalar_t>& p);

 private:
  std::shared_ptr<const Model> robotModelPtr_;
  std::unique_ptr<Data> robotDataPtr_;
  std::shared_ptr<const ::urdf::ModelInterface> urdfModelPtr_;
};

/** Print PinocchioInterfaceTpl info to stream */
std::ostream& operator<<(std::ostream& os, const PinocchioInterface& p);

/* Explicit template instantiation for scalar_t and ad_scalar_t */
extern template class PinocchioInterfaceTpl<scalar_t>;
extern template class PinocchioInterfaceTpl<ad_scalar_t>;

}  // namespace bolza
