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

#include "autolink/profiler/block.hpp"
#include "autolink/profiler/block_manager.hpp"

namespace autolink {
namespace profiler {

#if ENABLE_PROFILER

#if defined(__GNUC__) || defined(__GNUG__)
#define AFUNC __PRETTY_FUNCTION__
#elif defined(__clang__)
#define AFUNC __PRETTY_FUNCTION__
#else
#define AFUNC __func__
#endif

#define TOKEN_JOIN(x, y) x##y
#define UNIQUE_NAME(x) TOKEN_JOIN(prefix_perf, x)

#define PERF_BLOCK(name, ...)                                 \
    autolink::profiler::Block UNIQUE_NAME(__LINE__)(name);    \
    autolink::profiler::BlockManager::Instance()->StartBlock( \
        &UNIQUE_NAME(__LINE__));

#define PERF_BLOCK_END autolink::profiler::BlockManager::Instance()->EndBlock();

#define PERF_FUNCTION(...) PERF_BLOCK(AFUNC, ##__VA_ARGS__)

#else

#define PERF_BLOCK(...)
#define PERF_BLOCK_END
#define PERF_FUNCTION(...)

#endif  // #if ENABLE_PROFILER

}  // namespace profiler
}  // namespace autolink