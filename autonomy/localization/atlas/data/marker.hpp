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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_DATA_MARKER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_DATA_MARKER_HPP_

#include "autonomy/localization/atlas/type.hpp"

#include <mutex>
#include <Eigen/Core>
#include <sqlite3.h>

namespace autonomy::localization::atlas {
namespace marker_model {
class base;
}

namespace data {

class keyframe;

class marker {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! constructor
    marker(const eigen_alloc_vector<Vec3_t>& corners_pos_w, unsigned int id, const std::shared_ptr<marker_model::base>& marker_model);

    void set_corner_pos(const eigen_alloc_vector<Vec3_t>& corner_pos_w);

    // Factory method to load marker from db
    static std::shared_ptr<marker> from_stmt(sqlite3_stmt* stmt,
                                             std::unordered_map<unsigned int, std::shared_ptr<autonomy::localization::atlas::data::keyframe>>& keyframes);

    // Save marker info to db
    static std::vector<std::pair<std::string, std::string>> columns() {
        return std::vector<std::pair<std::string, std::string>>{
            {"corners_pos_w", "BLOB"},
            {"keep_fixed", "INTEGER"},
            {"n_observations", "INTEGER"},
            {"observations", "BLOB"},
            {"initialized_before", "INTEGER"}};
    };
    bool bind_to_stmt(sqlite3* db, sqlite3_stmt* stmt) const;

    //! corner positions
    eigen_alloc_vector<Vec3_t> corners_pos_w_;

    //! marker ID
    unsigned int id_;

    bool keep_fixed_ = false;

    bool initialized_before_ = false;

    //! marker model
    std::shared_ptr<marker_model::base> marker_model_;

    //! observed keyframes
    std::unordered_map<unsigned int, std::shared_ptr<keyframe>> observations_;

    mutable std::mutex mtx_position_;
};

} // namespace data
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_DATA_MARKER_HPP_
