/**
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "autolink/proto/record.pb.h"

#include "autolink/common/log.hpp"
#include "autolink/record/file/record_file_reader.hpp"
#include "autolink/record/file/record_file_writer.hpp"

using ::autolink::proto::ChannelCache;
using ::autolink::proto::ChunkBody;
using ::autolink::proto::ChunkHeader;
using ::autolink::proto::Header;

namespace autolink {
namespace record {

class Recoverer
{
public:
    Recoverer(const std::string& input_file, const std::string& output_file);
    virtual ~Recoverer();
    bool Proc();

private:
    RecordFileReader reader_;
    RecordFileWriter writer_;
    std::string input_file_;
    std::string output_file_;
    std::vector<std::string> channel_vec_;
};

}  // namespace record
}  // namespace autolink