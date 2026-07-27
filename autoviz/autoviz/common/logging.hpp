/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <glog/logging.h>

/** rviz_common::logging — glog-backed macros for autoviz/common. */
#define AUTOVIZ_LOG_DEBUG VLOG(1)
#define AUTOVIZ_LOG_INFO LOG(INFO)
#define AUTOVIZ_LOG_WARNING LOG(WARNING)
#define AUTOVIZ_LOG_ERROR LOG(ERROR)
