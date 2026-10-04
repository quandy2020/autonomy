/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file logging.hpp
 * @brief glog-backed logging macros for @c autoviz/common (RViz-style names).
 *
 * Provides a stable Autoviz-facing API so callers do not depend on glog
 * macro names directly. Mirrors the intent of @c rviz_common::logging.
 *
 * @note Prefer these macros over raw @c LOG/@c VLOG in common-layer code.
 */

#pragma once

#include <glog/logging.h>

/**
 * @brief Debug verbosity log (maps to @c VLOG(1)).
 * @see AUTOVIZ_LOG_INFO
 */
#define AUTOVIZ_LOG_DEBUG VLOG(1)

/**
 * @brief Informational log (maps to @c LOG(INFO)).
 */
#define AUTOVIZ_LOG_INFO LOG(INFO)

/**
 * @brief Warning log (maps to @c LOG(WARNING)).
 */
#define AUTOVIZ_LOG_WARNING LOG(WARNING)

/**
 * @brief Error log (maps to @c LOG(ERROR)).
 */
#define AUTOVIZ_LOG_ERROR LOG(ERROR)
