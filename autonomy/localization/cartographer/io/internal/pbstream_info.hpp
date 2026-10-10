/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CARTOGRAPHER_IO_INTERNAL_PBSTREAM_INFO_H_
#define CARTOGRAPHER_IO_INTERNAL_PBSTREAM_INFO_H_

#include <string>

namespace cartographer {
namespace io {

struct PbstreamInfoOptions {
    std::string pbstream_filename;
    bool all_debug_strings = false;
};

// info subtool for pbstream swiss army knife.
int pbstream_info(const PbstreamInfoOptions& options);

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_INTERNAL_PBSTREAM_INFO_H_
