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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_MODULE_LOOP_BUNDLE_ADJUSTER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_MODULE_LOOP_BUNDLE_ADJUSTER_HPP_

#include <mutex>

namespace autonomy::localization::atlas {

class mapping_module;

namespace data {
class keyframe;
class map_database;
} // namespace data

namespace module {

class loop_bundle_adjuster {
public:
    /**
     * Constructor
     */
    explicit loop_bundle_adjuster(data::map_database* map_db,
                                  const unsigned int num_iter = 10,
                                  const bool use_huber_kernel = false,
                                  const bool verbose = false);

    /**
     * Destructor
     */
    ~loop_bundle_adjuster() = default;

    /**
     * Set the mapping module
     */
    void set_mapping_module(mapping_module* mapper);

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
    mapping_module* mapper_ = nullptr;

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

} // namespace module
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_MODULE_LOOP_BUNDLE_ADJUSTER_HPP_
