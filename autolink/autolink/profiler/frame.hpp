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

#include <list>
#include <stack>
#include <string>

#include "autolink/profiler/block.hpp"

namespace autolink {
namespace profiler {

class Frame
{
public:
    void Push(Block* block);
    Block* Top();
    void Pop();

    bool DumpToFile(const std::string& coroutine_name);
    void Clear();

    std::uint32_t size() const {
        return stack_.size();
    }
    bool finished() const {
        return stack_.empty();
    }

private:
    std::stack<Block*> stack_;
    std::list<Block> storage_;
};

}  // namespace profiler
}  // namespace autolink