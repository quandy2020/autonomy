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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_DATA_ORB_PARAMS_DATABASE_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_DATA_ORB_PARAMS_DATABASE_HPP_

#include <mutex>
#include <unordered_map>

#include <nlohmann/json_fwd.hpp>

namespace autonomy::localization::atlas {

namespace feature {
struct orb_params;
} // namespace feature

namespace data {

class orb_params_database {
public:
    explicit orb_params_database();

    ~orb_params_database();

    void add_orb_params(feature::orb_params* orb_params);

    feature::orb_params* get_orb_params(const std::string& orb_params_name) const;

    void from_json(const nlohmann::json& json_orb_params);

    nlohmann::json to_json() const;

private:
    //-----------------------------------------
    //! mutex to access the database
    mutable std::mutex mtx_database_;
    //! database (key: orb_params name, value: pointer of feature::orb_params)
    std::unordered_map<std::string, feature::orb_params*> orb_params_database_;
};

} // namespace data
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_DATA_ORB_PARAMS_DATABASE_HPP_
