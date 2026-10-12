#pragma once

/**
 * @file root_finder_type.hpp
 * @brief Enum class for specifying root finding algorithm type.
 */


namespace bolza {

/**
 * Enum class for specifying root finding algorithm type.
 */
enum class RootFinderType { ANDERSON_BJORCK, PEGASUS, ILLINOIS, REGULA_FALSI };

}  // namespace bolza
