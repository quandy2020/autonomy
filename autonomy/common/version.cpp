/*
 * Copyright 2024 The Openbot Beginner Authors
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

#include <iostream>

#include "absl/strings/str_cat.h"
#include "glog/logging.h"
#include "autonomy/common/version.hpp"

#define RED_INFO(text)    "\033[31m" << text << "\033[0m"
#define GREEN_INFO(text)  "\033[32m" << text << "\033[0m"
#define YELLOW_INFO(text) "\033[33m" << text << "\033[0m"
#define BLUE_INFO(text)   "\033[34m" << text << "\033[0m"
#define MAGENTA_INFO(text)  "\033[35m" << text << "\033[0m"  // 紫色/洋红色
#define CYAN_INFO(text)     "\033[36m" << text << "\033[0m"  // 青色
#define WHITE_INFO(text)    "\033[37m" << text << "\033[0m"  // 白色

// 亮色/高亮版本
#define BRIGHT_RED_INFO(text)      "\033[91m" << text << "\033[0m"
#define BRIGHT_GREEN_INFO(text)    "\033[92m" << text << "\033[0m"
#define BRIGHT_YELLOW_INFO(text)   "\033[93m" << text << "\033[0m"
#define BRIGHT_BLUE_INFO(text)     "\033[94m" << text << "\033[0m"
#define BRIGHT_MAGENTA_INFO(text)  "\033[95m" << text << "\033[0m"
#define BRIGHT_CYAN_INFO(text)     "\033[96m" << text << "\033[0m"
#define BRIGHT_WHITE_INFO(text)    "\033[97m" << text << "\033[0m"

// 背景颜色
#define BG_RED_INFO(text)      "\033[41m" << text << "\033[0m"
#define BG_GREEN_INFO(text)    "\033[42m" << text << "\033[0m"
#define BG_YELLOW_INFO(text)   "\033[43m" << text << "\033[0m"
#define BG_BLUE_INFO(text)     "\033[44m" << text << "\033[0m"
#define BG_MAGENTA_INFO(text)  "\033[45m" << text << "\033[0m"
#define BG_CYAN_INFO(text)     "\033[46m" << text << "\033[0m"
#define BG_WHITE_INFO(text)    "\033[47m" << text << "\033[0m"

// 文本样式
#define BOLD_INFO(text)        "\033[1m" << text << "\033[0m"      // 加粗
#define DIM_INFO(text)         "\033[2m" << text << "\033[0m"      // 暗淡
#define ITALIC_INFO(text)      "\033[3m" << text << "\033[0m"      // 斜体
#define UNDERLINE_INFO(text)   "\033[4m" << text << "\033[0m"      // 下划线
#define BLINK_INFO(text)       "\033[5m" << text << "\033[0m"      // 闪烁
#define REVERSE_INFO(text)     "\033[7m" << text << "\033[0m"      // 反色
#define HIDDEN_INFO(text)      "\033[8m" << text << "\033[0m"      // 隐藏

// 组合样式（颜色+样式）
#define BOLD_RED_INFO(text)    "\033[1;31m" << text << "\033[0m"
#define UNDERLINE_BLUE_INFO(text) "\033[4;34m" << text << "\033[0m"
#define BOLD_CYAN_INFO(text)   "\033[1;36m" << text << "\033[0m"
// 可以按需组合其他样式

// 256色模式（扩展颜色）
#define COLOR_256_INFO(text, code) "\033[38;5;" << code << "m" << text << "\033[0m"
#define BG_COLOR_256_INFO(text, code) "\033[48;5;" << code << "m" << text << "\033[0m"

// RGB颜色（真彩色，如果终端支持）
#define RGB_INFO(text, r, g, b) "\033[38;2;" << r << ";" << g << ";" << b << "m" << text << "\033[0m"
#define BG_RGB_INFO(text, r, g, b) "\033[48;2;" << r << ";" << g << ";" << b << "m" << text << "\033[0m"

// 预定义的一些常用256色
#define ORANGE_INFO(text)      COLOR_256_INFO(text, 208)  // 橙色
#define PINK_INFO(text)        COLOR_256_INFO(text, 213)  // 粉色
#define PURPLE_INFO(text)      COLOR_256_INFO(text, 129)  // 紫色
#define TEAL_INFO(text)        COLOR_256_INFO(text, 30)   // 蓝绿色
#define LIME_INFO(text)        COLOR_256_INFO(text, 118)  // 亮绿色
#define BROWN_INFO(text)       COLOR_256_INFO(text, 130)  // 棕色
#define GRAY_INFO(text)        COLOR_256_INFO(text, 244)  // 灰色

namespace autonomy {
namespace common {
namespace {

const char* AUTONOMY_VERSION = "${AUTONOMY_VERSION}";
const char* AUTONOMY_COMMIT_ID = "${GIT_COMMIT_ID}";
const char* AUTONOMY_COMMIT_DATE = "${GIT_COMMIT_DATE}";

// 项目信息
const std::string PROJECT_NAME = "autonomy";
const std::string PROJECT_VERSION = "";

// Git提交信息
const std::string GIT_COMMIT_AUTHOR = "";
const std::string GIT_COMMIT_EMAIL = "";
const std::string GIT_COMMIT_DATE = "";
const std::string GIT_VERSION = "";
const std::string GIT_BRANCH = "";

// 构建信息
const std::string BUILD_TIMESTAMP = "2025-11-21 03:11:16";
const std::string BUILD_HOST = "74ad12676a7f";
const std::string BUILD_USER = "root";

// 系统信息
const std::string SYSTEM_NAME = "Linux";
const std::string SYSTEM_PROCESSOR = "x86_64";
const std::string SYSTEM_VERSION = "6.8.0-60-generic";
const std::string COMPILER_ID = "GNU";
const std::string COMPILER_VERSION = "13.3.0";

}  // namespace

std::string GetVersionInfo() 
{
  return absl::StrCat("Autonomy ", AUTONOMY_VERSION);
}

std::string GetBuildInfo() 
{
#if defined(AUTONOMY_CUDA_ENABLED)
  const char* cuda_info = "with CUDA";
#else
  const char* cuda_info = "without CUDA";
#endif
  return absl::StrCat("Autonomy develop info: Commit ", AUTONOMY_COMMIT_ID, " on ", AUTONOMY_COMMIT_DATE, " ", cuda_info);
}

std::string GetGitCommitID()
{
  return absl::StrCat("Autonomy git commit ID: ", AUTONOMY_COMMIT_ID, " on ", AUTONOMY_COMMIT_DATE);
}

std::string GetCudaInfo()
{
#if defined(AUTONOMY_CUDA_ENABLED)
  const char* cuda_info = "with CUDA";
#else
  const char* cuda_info = "without CUDA";
#endif
  return cuda_info;
}

void ShowVersion() 
{
    LOG(INFO) << RED_INFO("==============================================");
    LOG(INFO) << RED_INFO("        Autonomy Version Information          ");
    LOG(INFO) << RED_INFO("==============================================");
    LOG(INFO) << RED_INFO("--[ Autonomy for robotics develop everyone ]-- ");

    // 项目信息
    LOG(INFO) << GREEN_INFO("Project Info:");
    LOG(INFO) << GREEN_INFO(absl::StrCat("  [   name  ]: ", PROJECT_NAME));
    LOG(INFO) << GREEN_INFO(absl::StrCat("  [ version ]: ", common::GetVersionInfo().c_str()));
    LOG(INFO) << GREEN_INFO(absl::StrCat("  [ develop ]: ", common::GetGitCommitID().c_str()));
    LOG(INFO) << GREEN_INFO(absl::StrCat("  [ cuda    ]: ", common::GetCudaInfo().c_str()));
    LOG(INFO) << GREEN_INFO("  [ contact ]: " << "quandy2020@126.com");

    // Git信息
    LOG(INFO) << BLUE_INFO("Git Info:");
    LOG(INFO) << BLUE_INFO(absl::StrCat("  [  author ]: ", GIT_COMMIT_AUTHOR));
    LOG(INFO) << BLUE_INFO(absl::StrCat("  [  e-mail ]: ", GIT_COMMIT_EMAIL));
    LOG(INFO) << BLUE_INFO(absl::StrCat("  [  time   ]: ", GIT_COMMIT_DATE));
    LOG(INFO) << BLUE_INFO(absl::StrCat("  [ version ]: ", GIT_VERSION));
    LOG(INFO) << BLUE_INFO(absl::StrCat("  [ branch  ]: ", GIT_BRANCH));
  
    // 构建信息
    LOG(INFO) << YELLOW_INFO("Build Info:");
    LOG(INFO) << YELLOW_INFO(absl::StrCat("  [  time   ]: ", BUILD_TIMESTAMP));
    LOG(INFO) << YELLOW_INFO(absl::StrCat("  [  host   ]: ", BUILD_HOST));
    LOG(INFO) << YELLOW_INFO(absl::StrCat("  [  user   ]: ", BUILD_USER));

    // 系统信息
    LOG(INFO) << MAGENTA_INFO("System Info:");
    LOG(INFO) << MAGENTA_INFO(absl::StrCat("  [ system  ]: ", SYSTEM_NAME));
    LOG(INFO) << MAGENTA_INFO(absl::StrCat("  [  arch   ]: ", SYSTEM_PROCESSOR));
    LOG(INFO) << MAGENTA_INFO(absl::StrCat("  [ version ]: ", SYSTEM_VERSION));
    LOG(INFO) << MAGENTA_INFO(absl::StrCat("  [   g++   ]: ", COMPILER_ID));
    LOG(INFO) << MAGENTA_INFO(absl::StrCat("  [ version ]: ", COMPILER_VERSION));

    LOG(INFO) << RED_INFO("==============================================");
}

}  // namespace common
}  // namespace autonomy 
