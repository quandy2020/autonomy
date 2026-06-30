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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_DATA_CAMERA_DATABASE_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_DATA_CAMERA_DATABASE_HPP_

#include <mutex>
#include <unordered_map>

#include <nlohmann/json_fwd.hpp>

typedef struct sqlite3 sqlite3;

namespace autonomy::localization::atlas {

namespace camera {
class base;
} // namespace camera

namespace data {

class camera_database {
public:
    explicit camera_database();

    ~camera_database();

    void add_camera(camera::base* camera);

    camera::base* get_camera(const std::string& camera_name) const;

    void from_json(const nlohmann::json& json_cameras);

    nlohmann::json to_json() const;

    bool from_db(sqlite3* db);

    bool to_db(sqlite3* db) const;

private:
    //-----------------------------------------
    //! mutex to access the database
    mutable std::mutex mtx_database_;
    //! database (key: camera name, value: pointer of camera::base)
    std::unordered_map<std::string, camera::base*> cameras_;
};

} // namespace data
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_DATA_CAMERA_DATABASE_HPP_
