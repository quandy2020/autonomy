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

#include <glog/logging.h>

#include "autonomy/common/gflags.hpp"
#include "autonomy/transform/transform_query.hpp"

// ============================================================================
// 命令行参数定义
// ============================================================================

DEFINE_string(source_frame, "", "Source coordinate frame ID (required)");
DEFINE_string(target_frame, "", "Target coordinate frame ID (required)");
DEFINE_double(timeout, 1.0, "Lookup timeout in seconds (default: 1.0)");
DEFINE_bool(continuous, false, "Continuously print transform updates");
DEFINE_double(rate, 1.0, "Update rate in Hz when using --continuous (default: 1.0)");
DEFINE_bool(verbose_output, false, "Print verbose transform information");
DEFINE_bool(euler_angles, false, "Display rotation as Euler angles (roll, pitch, yaw)");
DEFINE_bool(matrix_format, false, "Display transformation as 4x4 matrix");
DEFINE_bool(list_frames, false, "List all available frames");
DEFINE_double(wait_time, 0.0, "Wait time in seconds before querying (default: 0.0)");

// ============================================================================
// 主函数
// ============================================================================

int main(int argc, char** argv)
{
    google::SetUsageMessage(
        "\n\n"
        "\033[1;32m=== Autonomy TF Query Tool ===\033[0m\n"
        "\n"
        "Query and display coordinate frame transformations.\n"
        "\n"
        "\033[1;33mBasic Usage:\033[0m\n"
        "  tf_query --source_frame=<frame1> --target_frame=<frame2>\n"
        "\n"
        "\033[1;33mExamples:\033[0m\n"
        "\n"
        "1. Query single transform:\n"
        "   tf_query --source_frame=base_link --target_frame=map\n"
        "\n"
        "2. Continuous monitoring:\n"
        "   tf_query --source_frame=base_link --target_frame=map --continuous --rate=10\n"
        "\n"
        "3. Verbose output with Euler angles:\n"
        "   tf_query --source_frame=base_link --target_frame=map --verbose_output --euler_angles\n"
        "\n"
        "4. Matrix format:\n"
        "   tf_query --source_frame=base_link --target_frame=map --matrix_format\n"
        "\n"
        "5. List all available frames:\n"
        "   tf_query --list_frames\n"
        "\n"
        "\033[1;33mKey Features:\033[0m\n"
        "  • Query transforms between any two coordinate frames\n"
        "  • Display as quaternion, Euler angles, or transformation matrix\n"
        "  • Continuous monitoring mode with configurable rate\n"
        "  • Detailed verbose output including norms and distances\n"
        "  • Beautiful formatted output with Unicode box drawing\n"
        "\n"
    );
    
    // 初始化 Google Logging
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = 1;
    FLAGS_colorlogtostderr = 1;
    
    // 解析命令行参数
    google::ParseCommandLineFlags(&argc, &argv, true);
    
    // 从命令行参数构建选项
    autonomy::transform::TransformQueryOptions options;
    options.source_frame = FLAGS_source_frame;
    options.target_frame = FLAGS_target_frame;
    options.timeout = FLAGS_timeout;
    options.wait_time = FLAGS_wait_time;
    options.verbose_output = FLAGS_verbose_output;
    options.euler_angles = FLAGS_euler_angles;
    options.matrix_format = FLAGS_matrix_format;
    options.continuous = FLAGS_continuous;
    options.rate = FLAGS_rate;
    options.list_frames = FLAGS_list_frames;
    
    // 创建并运行 Transform 查询工具
    autonomy::transform::TransformQuery query_tool(options);
    int exit_code = query_tool.Run();
    
    // 清理
    google::ShutdownGoogleLogging();
    
    return exit_code;
}
