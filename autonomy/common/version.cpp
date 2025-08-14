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

namespace autonomy {
namespace common {
namespace {

const char* AUTONOMY_VERSION = "0.0.1";
const char* AUTONOMY_COMMIT_ID = "2c392b5";
const char* AUTONOMY_COMMIT_DATE = "2025-08-14";

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
    LOG(INFO) << "--[ Autonomy for robotics develop everyone ]-- ";
    LOG(INFO) << "  [ version ]: " << common::GetVersionInfo().c_str() ;
    LOG(INFO) << "  [ develop ]: " << common::GetGitCommitID().c_str();
    LOG(INFO) << "  [ cuda    ]: " << common::GetCudaInfo().c_str();
    LOG(INFO) << "  [ contact ]: " << "quandy2020@126.com";
    LOG(INFO) << "---------------------------------------------- ";
}

}  // namespace common
}  // namespace autonomy 
