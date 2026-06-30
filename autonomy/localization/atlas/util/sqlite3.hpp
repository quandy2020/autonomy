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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_UTIL_SQLITE3_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_UTIL_SQLITE3_HPP_

#include <vector>
#include <string>

typedef struct sqlite3 sqlite3;
typedef struct sqlite3_stmt sqlite3_stmt;

namespace autonomy::localization::atlas {
namespace util {
namespace sqlite3_util {

bool create_table(sqlite3* db,
                  const std::string& name,
                  const std::vector<std::pair<std::string, std::string>>& columns);
bool drop_table(sqlite3* db,
                const std::string& name);
bool begin(sqlite3* db);
bool next(sqlite3* db, sqlite3_stmt* stmt);
bool commit(sqlite3* db);
sqlite3_stmt* create_select_stmt(sqlite3* db, const std::string& table_name);
sqlite3_stmt* create_insert_stmt(sqlite3* db,
                                 const std::string& name,
                                 const std::vector<std::pair<std::string, std::string>>& columns);

} // namespace sqlite3_util
} // namespace util
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_UTIL_SQLITE3_HPP_
