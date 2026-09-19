/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file reflection_service.hpp
 * @brief Reflection enablement helpers.
 */

#pragma once

#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace tools {

/**
 * @brief Whether this build linked grpc++_reflection.
 */
inline bool ReflectionLibraryAvailable() {
#if defined(AUTONOMY_HAVE_GRPC_REFLECTION)
    return true;
#else
    return false;
#endif
}

}  // namespace tools
}  // namespace bridge
}  // namespace autonomy
