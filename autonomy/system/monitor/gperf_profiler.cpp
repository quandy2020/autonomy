/*
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

#include "autonomy/system/monitor/gperf_profiler.hpp"

// 只有在编译时显式定义 HAVE_GPERFTOOLS_PROFILER / HAVE_GPERFTOOLS_HEAP
// 并且可用时才启用 gperftools，避免链接期对符号的硬依赖。
#if defined(HAVE_GPERFTOOLS_PROFILER) && HAVE_GPERFTOOLS_PROFILER
#include "gperftools/profiler.h"
#else
#undef HAVE_GPERFTOOLS_PROFILER
#define HAVE_GPERFTOOLS_PROFILER 0
#endif

#if defined(HAVE_GPERFTOOLS_HEAP) && HAVE_GPERFTOOLS_HEAP
#include "gperftools/heap-profiler.h"
#else
#undef HAVE_GPERFTOOLS_HEAP
#define HAVE_GPERFTOOLS_HEAP 0
#endif

namespace autonomy {
namespace system {
namespace monitor {

void GperfProfiler::StartCpuProfile(const std::string& filename) {
#if HAVE_GPERFTOOLS_PROFILER
  ProfilerStart(filename.c_str());
#else
  (void)filename;
#endif
}

void GperfProfiler::StopCpuProfile() {
#if HAVE_GPERFTOOLS_PROFILER
  ProfilerStop();
#endif
}

void GperfProfiler::StartHeapProfile(const std::string& filename) {
#if HAVE_GPERFTOOLS_HEAP
  HeapProfilerStart(filename.c_str());
#else
  (void)filename;
#endif
}

void GperfProfiler::StopHeapProfile() {
#if HAVE_GPERFTOOLS_HEAP
  HeapProfilerStop();
#else
#endif
}

bool GperfProfiler::IsCpuProfilingAvailable() {
#if HAVE_GPERFTOOLS_PROFILER
  return true;
#else
  return false;
#endif
}

bool GperfProfiler::IsHeapProfilingAvailable() {
#if HAVE_GPERFTOOLS_HEAP
  return true;
#else
  return false;
#endif
}

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
