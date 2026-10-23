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
 * @brief glog wrapper. `ATLAS_*` macros map to `LOG` / `CHECK`.
 */

#ifndef ATLAS_COMMON_LOGGING_HPP_
#define ATLAS_COMMON_LOGGING_HPP_

#include <string>

#if !defined(GLOG_USE_GLOG_EXPORT) && !defined(GLOG_EXPORT)
#define GLOG_USE_GLOG_EXPORT
#endif
#include <glog/logging.h>

namespace atlas {
namespace common {

/**
 * @brief Initialize glog once for the process (idempotent).
 * Messages go to stderr unless @ref SetLogDirectory is used.
 * @param[in] argv0 Program name for `google::InitGoogleLogging`. Null uses
 * `"atlas"`.
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
}  // namespace atlas

#define ATLAS_INFO LOG(INFO)
#define ATLAS_WARN LOG(WARNING)
#define ATLAS_WARNING LOG(WARNING)
#define ATLAS_ERROR LOG(ERROR)
#define ATLAS_FATAL LOG(FATAL)
#define ATLAS_VLOG(verboselevel) VLOG(verboselevel)

#define ATLAS_CHECK CHECK
#define ATLAS_CHECK_EQ CHECK_EQ
#define ATLAS_CHECK_NE CHECK_NE
#define ATLAS_CHECK_LE CHECK_LE
#define ATLAS_CHECK_LT CHECK_LT
#define ATLAS_CHECK_GE CHECK_GE
#define ATLAS_CHECK_GT CHECK_GT
#define ATLAS_CHECK_NOTNULL CHECK_NOTNULL
#define ATLAS_DCHECK DCHECK
#define ATLAS_DCHECK_EQ DCHECK_EQ
#define ATLAS_DCHECK_NE DCHECK_NE

#endif  // ATLAS_COMMON_LOGGING_HPP_
