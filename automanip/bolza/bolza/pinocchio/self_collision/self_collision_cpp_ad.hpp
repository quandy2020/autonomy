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
 * @file self_collision_cpp_ad.hpp
 * @brief Constructor.
 */


#include <bolza/core/automatic_differentiation/cpp_ad_interface.hpp>
#include "bolza/pinocchio/interface/pinocchio_interface.hpp"

#include "bolza/pinocchio/self_collision/pinocchio_geometry_interface.hpp"

namespace bolza {

class SelfCollisionCppAd {
 public:
  using vector3_t = Eigen::Matrix<ad_scalar_t, 3, 1>;
  using quaternion_t = Eigen::Quaternion<ad_scalar_t>;

  /**
   * Constructor
   *
   * @param [in] pinocchioInterface: pinocchio interface of the robot model
   * @param [in] pinocchioGeometryInterface: pinocchio geometry interface of the robot model
   * @param [in] minimumDistance: minimum allowed distance between each collision pair
   * @param [in] modelName: name of the generate model library
   * @param [in] modelFolder: folder to save the model library files to
   * @param [in] recompileLibraries: If true, the model library will be newly compiled. If false, an existing library will be loaded if
   *                                  available.
   * @param [in] verbose: print information.
   */
  SelfCollisionCppAd(const PinocchioInterface& pinocchioInterface, PinocchioGeometryInterface pinocchioGeometryInterface,
                     scalar_t minimumDistance, const std::string& modelName, const std::string& modelFolder = "/tmp/ocs2",
                     bool recompileLibraries = true, bool verbose = true);

  /** Default destructor */
  ~SelfCollisionCppAd() = default;

  /**
   * @brief Copy constructor.
   * @param [in] rhs: Rhs.
   */
  SelfCollisionCppAd(const SelfCollisionCppAd& rhs);

  /**
   * @brief Get the number of collision pairs.
   * @return Result value.
   */
  size_t GetNumCollisionPairs() const { return pinocchioGeometryInterface_.getNumCollisionPairs(); }

  /**
   * Evaluate the distance violation
   * Computes the distance results of all collision pairs through PinocchioGeometryInterface
   * and the violation compared with the specified minimum distance.
   *
   * @note Requires updated forwardKinematics() on pinocchioInterface.
   *
   * @param [in] pinocchioInterface: pinocchio interface of the robot model
   * @return the differences between the distance of each collision pair and the minimum distance
   */
  vector_t GetValue(const PinocchioInterface& pinocchioInterface) const;

  /**
   * Evaluate the linear approximation of the distance function
   *
   * @note Requires updated forwardKinematics() on pinocchioInterface.
   *
   * @param [in] pinocchioInterface: pinocchio interface of the robot model
   * @param [in] q: pinocchio coordinates
   * @return the pair of the distance violation and the first derivative of the distance against q
   */
  std::pair<vector_t, matrix_t> GetLinearApproximation(const PinocchioInterface& pinocchioInterface, const vector_t& q) const;

 private:
  /**
   * @brief Sets all the required CppAdCodeGenInterfaces.
   * @param [in,out] pinocchioInterfaceAd: Pinocchio Interface Ad.
   * @param [in] modelName: Model Name.
   * @param [in] modelFolder: Model Folder.
   */
  void SetADInterfaces(PinocchioInterfaceCppAd& pinocchioInterfaceAd, const std::string& modelName, const std::string& modelFolder);

  // From the current state of the robot, and the closest points in world frame, compute the positions of the points in link frame
  // In this case : size of state = stateDim, size of points = 3*2*number of collision pairs + 1 (for sign indicator)
  // Returns a vector that is of length |3*2*number of collision pairs + 1 (for sign indicator)|
  /**
   * @brief Compute Link Points Ad.
   * @param [in,out] pinocchioInterfaceAd: Pinocchio Interface Ad.
   * @param [in] state: State.
   * @param [in] points: Points.
   * @return Result value.
   */
  ad_vector_t ComputeLinkPointsAd(PinocchioInterfaceCppAd& pinocchioInterfaceAd, const ad_vector_t& state, const ad_vector_t& points) const;
  // From the current state of the robot, and the closest points in link frames, calculate the distances wrt state
  // In this case : size of state = stateDim, size of points = 3*2*number of collision pairs + 1 (for sign indicator)
  // Returns a vector that is of length |collisionPairs|
  /**
   * @brief Distance Calculation Ad.
   * @param [in,out] pinocchioInterfaceAd: Pinocchio Interface Ad.
   * @param [in] state: State.
   * @param [in] points: Points.
   * @return Result value.
   */
  ad_vector_t DistanceCalculationAd(PinocchioInterfaceCppAd& pinocchioInterfaceAd, const ad_vector_t& state,
                                    const ad_vector_t& points) const;

  // Number of params per result = 3 + 3 + 1 (nearest point 1, nearest point 2, sign indicator)
  const size_t numberOfParamsPerResult_ = 7;

  std::unique_ptr<CppAdInterface> cppAdInterfaceDistanceCalculation_;
  std::unique_ptr<CppAdInterface> cppAdInterfaceLinkPoints_;

  PinocchioGeometryInterface pinocchioGeometryInterface_;
  scalar_t minimumDistance_;
};

} /* namespace bolza */
