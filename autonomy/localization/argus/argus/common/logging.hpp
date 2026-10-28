/*
 * Copyright 2026 The Openbot Authors
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

/**
 * @file logging.hpp
 * @brief glog wrapper. `ARGUS_*` macros map to `LOG` / `CHECK`.
 */

#ifndef ARGUS_COMMON_LOGGING_HPP_
#define ARGUS_COMMON_LOGGING_HPP_

#include <string>

#if !defined(GLOG_USE_GLOG_EXPORT) && !defined(GLOG_EXPORT)
#define GLOG_USE_GLOG_EXPORT
#endif
#include <glog/logging.h>

namespace argus {
namespace common {

/**
 * @brief Initialize glog once for the process (idempotent).
 * Messages go to stderr unless @ref SetLogDirectory is used.
 * @param[in] argv0 Program name for `google::InitGoogleLogging`. Null uses
 * `"argus"`.
 */
void InitLogging(const char* argv0);

/** @brief Shut down glog if this process initialized it. Idempotent. */
void ShutdownLogging();

/** @return true after @ref InitLogging until @ref ShutdownLogging. */
bool IsLoggingInitialized();

/**
 * @brief Set glog `FLAGS_minloglevel`.
 * @param[in] severity `google::GLOG_INFO`, `GLOG_WARNING`, `GLOG_ERROR`, or
 * `GLOG_FATAL`.
 */
void SetMinLogSeverity(int severity);

/**
 * @brief Set `FLAGS_log_dir`. An empty directory keeps stderr-only logging.
 */
void SetLogDirectory(const std::string& dir);

}  // namespace common
}  // namespace argus

#define ARGUS_INFO LOG(INFO)
#define ARGUS_WARN LOG(WARNING)
#define ARGUS_WARNING LOG(WARNING)
#define ARGUS_ERROR LOG(ERROR)
#define ARGUS_FATAL LOG(FATAL)
#define ARGUS_VLOG(verboselevel) VLOG(verboselevel)

#define ARGUS_CHECK CHECK
#define ARGUS_CHECK_EQ CHECK_EQ
#define ARGUS_CHECK_NE CHECK_NE
#define ARGUS_CHECK_LE CHECK_LE
#define ARGUS_CHECK_LT CHECK_LT
#define ARGUS_CHECK_GE CHECK_GE
#define ARGUS_CHECK_GT CHECK_GT
#define ARGUS_CHECK_NOTNULL CHECK_NOTNULL
#define ARGUS_DCHECK DCHECK
#define ARGUS_DCHECK_EQ DCHECK_EQ
#define ARGUS_DCHECK_NE DCHECK_NE

#endif  // ARGUS_COMMON_LOGGING_HPP_
