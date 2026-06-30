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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_IO_MAP_DATABASE_IO_SQLITE3_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_IO_MAP_DATABASE_IO_SQLITE3_HPP_

#include "autonomy/localization/atlas/io/map_database_io_base.hpp"
#include "autonomy/localization/atlas/data/bow_vocabulary.hpp"

#include <string>

typedef struct sqlite3 sqlite3;

namespace autonomy::localization::atlas {

namespace data {
class camera_database;
class bow_database;
class map_database;
} // namespace data

namespace io {

class map_database_io_sqlite3 : public map_database_io_base {
public:
    /**
     * Constructor
     */
    map_database_io_sqlite3() = default;

    /**
     * Destructor
     */
    virtual ~map_database_io_sqlite3() = default;

    /**
     * Save the map database as MessagePack
     */
    bool save(const std::string& path,
              const data::camera_database* const cam_db,
              const data::orb_params_database* const orb_params_db,
              const data::map_database* const map_db) override;

    /**
     * Load the map database from MessagePack
     */
    bool load(const std::string& path,
              data::camera_database* cam_db,
              data::orb_params_database* orb_params_db,
              data::map_database* map_db,
              data::bow_database* bow_db,
              data::bow_vocabulary* bow_vocab) override;

private:
    bool save_stats(sqlite3* db, const data::map_database* map_db) const;
    bool load_stats(sqlite3* db, data::map_database* map_db) const;
};

} // namespace io
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_IO_MAP_DATABASE_IO_SQLITE3_HPP_
