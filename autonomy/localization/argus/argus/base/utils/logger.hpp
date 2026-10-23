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
 * @file logger.hpp
 * @brief Thin glog wrapper for Argus (`ARGUS_*` → `LOG` / `CHECK` / `VLOG`).
 */

#ifndef ARGUS_COMMON_LOGGER_HPP_
#define ARGUS_COMMON_LOGGER_HPP_

#include <string>

// Modern glog (0.7+) requires this before including logging.h when not
// consuming via the CMake exported target compile definitions.
#if !defined(GLOG_USE_GLOG_EXPORT) && !defined(GLOG_EXPORT)
#define GLOG_USE_GLOG_EXPORT
#endif
#include <glog/logging.h>

namespace argus {
namespace common {

/**
 * @brief Initialize glog once (idempotent).
 *
 * Sets `logtostderr=1` by default so logs appear on stderr without a log dir.
 * Call from `main` before other Argus work when embedding Argus.
 * @param argv0 Program name passed to `google::InitGoogleLogging`.
 */
void InitLogging(const char* argv0);

/** @brief Idempotent shutdown (`google::ShutdownGoogleLogging`). */
void ShutdownLogging();

/** @return True after InitLogging has run successfully. */
bool IsLoggingInitialized();

/**
 * @brief Minimum severity: `google::GLOG_INFO` / `WARNING` / `ERROR` / `FATAL`.
 * Maps to `FLAGS_minloglevel`.
 * @param severity glog severity level constant.
 */
void SetMinLogSeverity(int severity);

/**
 * @brief Directory for glog files (`FLAGS_log_dir`). Empty → stderr only.
 * @param dir Log directory path; empty disables file logging.
 */
void SetLogDirectory(const std::string& dir);

}  // namespace common
}  // namespace argus

// Stream logging (glog).
#define ARGUS_INFO LOG(INFO)
#define ARGUS_WARN LOG(WARNING)
#define ARGUS_WARNING LOG(WARNING)
#define ARGUS_ERROR LOG(ERROR)
#define ARGUS_FATAL LOG(FATAL)

#define ARGUS_VLOG(verboselevel) VLOG(verboselevel)

// Assertions (glog).
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

#endif  // ARGUS_COMMON_LOGGER_HPP_
