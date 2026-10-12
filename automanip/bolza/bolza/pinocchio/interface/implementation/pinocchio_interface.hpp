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

#include <pinocchio/codegen/cppadcg.hpp>
#include <pinocchio/fwd.hpp>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace bolza {

/******************************************************************************************************/

/**
 * @file pinocchio_interface.hpp
 * @brief pinocchio interface (bolza).
 */

/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
/**
 * @brief Pinocchio Interface Tpl.
 * @param [in] model: Model.
 * @param [in] urdfModelPtr: Urdf Model Ptr.
 * @return Result value.
 */
PinocchioInterfaceTpl<SCALAR>::PinocchioInterfaceTpl(const Model& model, const std::shared_ptr<const ::urdf::ModelInterface> urdfModelPtr) {
  robotModelPtr_ = std::make_shared<const Model>(model);
  robotDataPtr_ = std::make_unique<Data>(*robotModelPtr_);
  if (urdfModelPtr) {
    // This makes a copy of the urdfModelPtr, which guarantees constness of the urdf
    urdfModelPtr_ = std::make_shared<const ::urdf::ModelInterface>(*urdfModelPtr);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
/**
 * @brief Destructor.
 */
PinocchioInterfaceTpl<SCALAR>::~PinocchioInterfaceTpl() = default;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
PinocchioInterfaceTpl<SCALAR>::PinocchioInterfaceTpl(const PinocchioInterfaceTpl<SCALAR>& rhs)
    /**
     * @brief Robot Model Ptr.
     * @param [in] urdfModelPtr_: Urdf Model Ptr.
     * @return Result value.
     */
    : robotModelPtr_(rhs.robotModelPtr_), robotDataPtr_(new Data(*rhs.robotDataPtr_)), urdfModelPtr_(rhs.urdfModelPtr_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
PinocchioInterfaceTpl<SCALAR>::PinocchioInterfaceTpl(PinocchioInterfaceTpl<SCALAR>&& rhs)
    /**
     * @brief Robot Model Ptr.
     * @return Result value.
     */
    : robotModelPtr_(std::move(rhs.robotModelPtr_)),
      robotDataPtr_(std::move(rhs.robotDataPtr_)),
      urdfModelPtr_(std::move(rhs.urdfModelPtr_)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
PinocchioInterfaceTpl<SCALAR>& PinocchioInterfaceTpl<SCALAR>::operator=(const PinocchioInterfaceTpl<SCALAR>& rhs) {
  robotModelPtr_ = rhs.robotModelPtr_;
  /**
   * @brief Reset.
   * @return Result value.
   */
  robotDataPtr_.reset(new Data(*rhs.robotDataPtr_));
  urdfModelPtr_ = rhs.urdfModelPtr_;
  return *this;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
PinocchioInterfaceTpl<SCALAR>& PinocchioInterfaceTpl<SCALAR>::operator=(PinocchioInterfaceTpl<SCALAR>&& rhs) {
  /**
   * @brief Swap.
   * @param [in] robotModelPtr_: Robot Model Ptr.
   * @param [in] robotModelPtr_: Robot Model Ptr.
   * @return Result value.
   */
  std::swap(robotModelPtr_, rhs.robotModelPtr_);
  /**
   * @brief Swap.
   * @param [in] robotDataPtr_: Robot Data Ptr.
   * @param [in] robotDataPtr_: Robot Data Ptr.
   * @return Result value.
   */
  std::swap(robotDataPtr_, rhs.robotDataPtr_);
  /**
   * @brief Swap.
   * @param [in] urdfModelPtr_: Urdf Model Ptr.
   * @param [in] urdfModelPtr_: Urdf Model Ptr.
   * @return Result value.
   */
  std::swap(urdfModelPtr_, rhs.urdfModelPtr_);
  return *this;
}

}  // namespace bolza
