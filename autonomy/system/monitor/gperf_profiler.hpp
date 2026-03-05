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

#pragma once

#include <string>

namespace autonomy {
namespace system {
namespace monitor {

/**
 * 封装 gperftools (gperf) CPU/堆分析启停，便于通过 MonitorOptions 参数选定。
 * 当未安装 gperftools 时接口为空实现。
 */
class GperfProfiler {
 public:
  GperfProfiler() = default;

  /// 开始 CPU 分析，filename 为输出 .prof 路径
  void StartCpuProfile(const std::string& filename);
  /// 停止 CPU 分析并写文件
  void StopCpuProfile();

  /// 开始堆分析，filename 为输出路径
  void StartHeapProfile(const std::string& filename);
  /// 停止堆分析
  void StopHeapProfile();

  /// 是否支持 CPU 分析（编译时是否链接了 gperftools）
  static bool IsCpuProfilingAvailable();
  /// 是否支持堆分析
  static bool IsHeapProfilingAvailable();
};

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
