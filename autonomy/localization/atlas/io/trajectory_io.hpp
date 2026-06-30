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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_IO_TRAJECTORY_IO_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_IO_TRAJECTORY_IO_HPP_

#include <string>

namespace autonomy::localization::atlas {

namespace data {
class map_database;
} // namespace data

namespace io {

class trajectory_io {
public:
    /**
     * Constructor
     */
    explicit trajectory_io(data::map_database* map_db);

    /**
     * Destructor
     */
    ~trajectory_io() = default;

    /**
     * Save the frame trajectory in the specified format
     */
    void save_frame_trajectory(const std::string& path, const std::string& format) const;

    /**
     * Save the keyframe trajectory in the specified format
     */
    void save_keyframe_trajectory(const std::string& path, const std::string& format) const;

private:
    //! map_database
    data::map_database* const map_db_ = nullptr;
};

} // namespace io
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_IO_TRAJECTORY_IO_HPP_
