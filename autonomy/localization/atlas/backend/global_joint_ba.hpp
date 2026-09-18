/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_BACKEND_GLOBAL_JOINT_BA_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_BACKEND_GLOBAL_JOINT_BA_HPP_

#include "autonomy/localization/atlas/estimate/lidar_residual_source.hpp"
#include "autonomy/localization/atlas/estimate/residual_mask.hpp"
#include "autonomy/localization/atlas/estimate/residual_odom.hpp"

#include <memory>
#include <mutex>

namespace autonomy::localization::atlas {

class LocalMapping;
class Tracking;

namespace data {
class keyframe;
class map_database;
} // namespace data

namespace backend {

class GlobalJointBA {
public:
    /**
     * Global / loop Joint BA — same ResidualMask + State as LocalJointBA.
     * optimize() may stage vision/imu global BA then lidar/odom refine within
     * one call; this is the Atlas LIVO joint problem (not a second system).
     */
    explicit GlobalJointBA(data::map_database* map_db,
                           const unsigned int num_iter = 10,
                           const bool use_huber_kernel = false,
                           const bool verbose = false);

    /**
     * Destructor
     */
    ~GlobalJointBA() = default;

    /**
     * Set the mapping module
     */
    void set_mapping_module(LocalMapping* mapper);

    void set_tracking_module(Tracking* tracker);

    void set_residual_mask(estimate::ResidualMask mask);
    void set_lidar_residual_source(estimate::ILidarResidualSource* src);
    void set_odom_residual_source(estimate::IOdomResidualSource* src);

    /**
     * Abort loop BA externally
     */
    void abort();

    /**
     * Loop BA is running or not
     */
    bool is_running() const;

    /**
     * Run loop BA
     */
    void optimize(const std::shared_ptr<data::keyframe>& curr_keyfrm);

private:
    //! map database
    data::map_database* map_db_ = nullptr;

    //! mapping module
    LocalMapping* mapper_ = nullptr;

    //! tracking module (paused during loop BA to avoid map races)
    Tracking* tracker_ = nullptr;

    //! Default all-true for backward compat until Pipeline sets runtime mask.
    estimate::ResidualMask mask_{true, true, true, true};
    estimate::ILidarResidualSource* lidar_src_ = nullptr;
    estimate::IOdomResidualSource* odom_src_ = nullptr;

    //! number of iteration for optimization
    const unsigned int num_iter_ = 10;
    //! True if using Huber kernel (for g2o)
    const bool use_huber_kernel_ = false;
    //! Verbosity (for g2o)
    const bool verbose_ = false;

    //-----------------------------------------
    // thread management

    //! mutex for access to pause procedure
    mutable std::mutex mtx_thread_;

    //! flag to abort loop BA
    bool abort_loop_BA_ = false;

    //! flag which indicates loop BA is running or not
    bool loop_BA_is_running_ = false;
};

}  // namespace backend

namespace module {
using loop_bundle_adjuster = backend::GlobalJointBA;
}  // namespace module

}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_BACKEND_GLOBAL_JOINT_BA_HPP_
