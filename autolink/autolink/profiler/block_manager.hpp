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

#include <string>
#include <unordered_map>

#include "autolink/common/macros.hpp"
#include "autolink/profiler/block.hpp"
#include "autolink/profiler/frame.hpp"

namespace autolink {
namespace profiler {

class BlockManager
{
public:
    using RoutineName = std::string;
    using RoutineFrameMap = std::unordered_map<RoutineName, Frame>;

public:
    void StartBlock(Block* block);

    void EndBlock();

private:
    std::string GetRoutineName();
    Frame* GetRoutineFrame();

private:
    static thread_local RoutineFrameMap routine_frame_map_;

    DECLARE_SINGLETON(BlockManager)
};

}  // namespace profiler
}  // namespace autolink