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
#include <signal.h>
#include <atomic>
#include <chrono>
#include <thread>
#include <filesystem>

#include "autonomy/common/version.hpp"
#include "autonomy/common/gflags.hpp"
#include "autonomy/transform/transform_server.hpp"

// 定义额外的命令行参数
DEFINE_string(transform_config_file, "", 
    "Path to the transform configuration file (YAML or Lua format). "
    "If not specified, uses configuration_directory + transform/transform.lua");

DEFINE_bool(use_yaml_config, false,
    "If true, use YAML configuration format instead of Lua");

DEFINE_double(tf_buffer_cache_time, 10.0,
    "TF buffer cache time in seconds (default: 10.0)");

DEFINE_bool(enable_static_transforms, true,
    "Enable static transform broadcasting (default: true)");

DEFINE_string(static_transform_config_file, "",
    "Path to static transform YAML configuration file. "
    "If not specified, uses configuration_directory + transform/static_transform.yaml");

DEFINE_bool(print_statistics, false,
    "Print transform server statistics periodically");

DEFINE_int32(statistics_interval, 10,
    "Statistics print interval in seconds (default: 10)");

namespace autonomy {
namespace transform {
namespace {

// 全局退出标志
std::atomic<bool> g_shutdown_requested{false};

// 全局 TransformServer 指针，用于信号处理
TransformServer::SharedPtr g_transform_server = nullptr;

/**
 * @brief 信号处理函数
 * 
 * 处理 SIGINT (Ctrl+C) 和 SIGTERM 信号，优雅地关闭服务
 */
void SignalHandler(int sig)
{
    const char* sig_name = (sig == SIGINT) ? "SIGINT" : "SIGTERM";
    LOG(INFO) << "Received " << sig_name << " signal, shutting down...";
    
    g_shutdown_requested = true;
    
    // 停止 TransformServer
    if (g_transform_server && g_transform_server->IsRunning()) {
        LOG(INFO) << "Stopping TransformServer...";
        g_transform_server->Stop();
    }
}

/**
 * @brief 注册信号处理器
 */
void RegisterSignalHandlers()
{
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);
}

/**
 * @brief 获取配置文件路径
 * 
 * 根据命令行参数构建完整的配置文件路径
 */
std::string GetConfigFilePath()
{
    // 如果直接指定了配置文件路径，使用该路径
    if (!FLAGS_transform_config_file.empty()) {
        return FLAGS_transform_config_file;
    }
    
    // 否则从配置目录构建路径
    if (autonomy::common::FLAGS_configuration_directory.empty()) {
        LOG(ERROR) << "Configuration directory not specified!";
        return "";
    }
    
    std::string config_file = autonomy::common::FLAGS_configuration_directory;
    if (FLAGS_use_yaml_config) {
        config_file += "/transform/transform.yaml";
    } else {
        config_file += "/transform/transform.lua";
    }
    
    return config_file;
}

/**
 * @brief 打印服务器状态
 */
void PrintServerStatus(const TransformServer::SharedPtr& server)
{
    if (!server) {
        LOG(WARNING) << "TransformServer is null";
        return;
    }
    
    LOG(INFO) << "TransformServer Status:";
    LOG(INFO) << "  - Initialized: " << (server->IsInitialized() ? "Yes" : "No");
    LOG(INFO) << "  - Running: " << (server->IsRunning() ? "Yes" : "No");
    
    // 测试TF查询功能
    try {
        auto current_time = commsgs::builtin_interfaces::Time::Now();
        bool can_transform = server->CanTransform("base_link", "map", current_time);
        LOG(INFO) << "  - Can transform (map -> base_link): " 
                  << (can_transform ? "Yes" : "No");
    } catch (const std::exception& e) {
        LOG(INFO) << "  - Transform query not available yet: " << e.what();
    }
}

/**
 * @brief 统计信息打印线程
 */
void StatisticsThread(const TransformServer::SharedPtr& server)
{
    while (!g_shutdown_requested) {
        std::this_thread::sleep_for(std::chrono::seconds(FLAGS_statistics_interval));
        
        if (!g_shutdown_requested) {
            LOG(INFO) << "=== Transform Server Statistics ===";
            PrintServerStatus(server);
        }
    }
}

/**
 * @brief 主运行函数
 */
int Run()
{
    // 注册信号处理器
    RegisterSignalHandlers();
    
    // 显示版本信息
    autonomy::common::ShowVersion();
    LOG(INFO) << "Starting Transform Server...";
    
    // 获取配置文件路径
    std::string config_file = GetConfigFilePath();
    if (config_file.empty()) {
        LOG(ERROR) << "No configuration file specified!";
        LOG(ERROR) << "Please provide configuration using one of:";
        LOG(ERROR) << "  --transform_config_file=<path>";
        LOG(ERROR) << "  --configuration_directory=<dir> [--use_yaml_config]";
        return EXIT_FAILURE;
    }
    
    // 检查配置文件是否存在
    if (!std::filesystem::exists(config_file)) {
        LOG(ERROR) << "Configuration file not found: " << config_file;
        return EXIT_FAILURE;
    }
    
    LOG(INFO) << "Using configuration file: " << config_file;
    
    // 创建 TransformServer
    g_transform_server = std::make_shared<TransformServer>();
    
    // 初始化服务器
    bool init_success = false;
    if (FLAGS_use_yaml_config) {
        // 使用 YAML 配置
        init_success = g_transform_server->InitializeFromYaml(config_file);
    } else {
        // 使用 Lua 配置（通过 proto 配置）
        TransformServerConfig config;
        config.buffer_cache_time = FLAGS_tf_buffer_cache_time;
        
        if (!FLAGS_static_transform_config_file.empty()) {
            config.static_transform_config_path = FLAGS_static_transform_config_file;
        } else if (!autonomy::common::FLAGS_configuration_directory.empty()) {
            config.static_transform_config_path = 
                autonomy::common::FLAGS_configuration_directory + 
                "/transform/static_transform.yaml";
        }
        
        init_success = g_transform_server->InitializeFromConfig(config);
    }
    
    if (!init_success) {
        LOG(ERROR) << "Failed to initialize TransformServer";
        return EXIT_FAILURE;
    }
    
    LOG(INFO) << "TransformServer initialized successfully";
    
    // 启动服务器（Start() 返回 void，使用 try-catch 处理异常）
    try {
        g_transform_server->Start();
        LOG(INFO) << "TransformServer started successfully";
    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to start TransformServer: " << e.what();
        return EXIT_FAILURE;
    }
    PrintServerStatus(g_transform_server);
    
    // 启动统计信息打印线程（如果启用）
    std::thread stats_thread;
    if (FLAGS_print_statistics) {
        stats_thread = std::thread(StatisticsThread, g_transform_server);
    }
    
    // 主循环：保持运行直到收到退出信号
    LOG(INFO) << "Transform Server is running. Press Ctrl+C to exit.";
    while (!g_shutdown_requested) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // 等待统计线程结束
    if (stats_thread.joinable()) {
        stats_thread.join();
    }
    
    // 停止服务器
    LOG(INFO) << "Stopping TransformServer...";
    g_transform_server->Stop();
    
    LOG(INFO) << "Transform Server shutdown complete";
    return EXIT_SUCCESS;
}

} // namespace 
} // namespace transform
} // namespace autonomy

int main(int argc, char **argv)
{
    google::SetUsageMessage(
        "\n\n"
        "\033[1;32m=== Autonomy Transform Server ===\033[0m\n"
        "\n"
        "This program provides TF (Transform) services for the autonomy robot framework.\n"
        "\n"
        "\033[1;33mUsage Examples:\033[0m\n"
        "\n"
        "1. Use Lua configuration:\n"
        "   autonomy.transform.transform_server \\\n"
        "     -configuration_directory /workspace/autonomy/configuration_files \\\n"
        "     -configuration_basename transform.lua\n"
        "\n"
        "2. Use YAML configuration:\n"
        "   autonomy.transform.transform_server \\\n"
        "     -transform_config_file /workspace/autonomy/configuration_files/transform/transform.yaml \\\n"
        "     -use_yaml_config\n"
        "\n"
        "3. Use custom static transform config:\n"
        "   autonomy.transform.transform_server \\\n"
        "     -configuration_directory /workspace/autonomy/configuration_files \\\n"
        "     -static_transform_config_file /path/to/static_transform.yaml\n"
        "\n"
        "4. Enable statistics printing:\n"
        "   autonomy.transform.transform_server \\\n"
        "     -configuration_directory /workspace/autonomy/configuration_files \\\n"
        "     -print_statistics \\\n"
        "     -statistics_interval 5\n"
        "\n"
        "\033[1;33mKey Features:\033[0m\n"
        "  - TF buffer management with configurable cache time\n"
        "  - Static transform broadcasting from YAML configuration\n"
        "  - Dynamic transform listener and broadcaster\n"
        "  - Transform lookup and query services\n"
        "  - Graceful shutdown on SIGINT/SIGTERM\n"
        "\n");

    // 初始化 Google Logging
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = 1;  // 输出到 stderr
    FLAGS_colorlogtostderr = 1;  // 彩色日志
    
    // 解析命令行参数
    google::ParseCommandLineFlags(&argc, &argv, true);
    
    // 处理 verbose 标志
    if (autonomy::common::FLAGS_verbose) {
        autonomy::common::ShowVersion();
        return EXIT_SUCCESS;
    }
    
    // 运行服务器
    int exit_code = autonomy::transform::Run();
    
    // 清理
    google::ShutdownGoogleLogging();
    
    return exit_code;
}

