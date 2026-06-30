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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_IO_MAP_DATABASE_IO_FACTORY_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_IO_MAP_DATABASE_IO_FACTORY_HPP_

#include "autonomy/localization/atlas/data/bow_vocabulary.hpp"
#include "autonomy/localization/atlas/io/map_database_io_base.hpp"
#include "autonomy/localization/atlas/io/map_database_io_msgpack.hpp"
#include "autonomy/localization/atlas/io/map_database_io_sqlite3.hpp"

#include <string>

namespace autonomy::localization::atlas {

namespace data {
class camera_database;
class bow_database;
class map_database;
} // namespace data

namespace io {

class map_database_io_factory {
public:
    static std::shared_ptr<map_database_io_base> create(const std::string& map_format) {
        std::shared_ptr<map_database_io_base> map_database_io;
        if (map_format == "sqlite3") {
            map_database_io = std::make_shared<io::map_database_io_sqlite3>();
        }
        else if (map_format == "msgpack") {
            map_database_io = std::make_shared<io::map_database_io_msgpack>();
        }
        else {
            throw std::runtime_error("Invalid map format: " + map_format);
        }
        return map_database_io;
    }
};

} // namespace io
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_IO_MAP_DATABASE_IO_FACTORY_HPP_
