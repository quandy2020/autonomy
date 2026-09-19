/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file variable_tag.hpp
 * @brief Tag types for bridge compile-time composition (state_vector style).
 *
 * @details
 * Variable / SampleVariable / ActionVariable form a lightweight tag hierarchy
 * used by GenericSampleCache, action_pack, and message↔tag maps. Traits
 * (`is_variable` / `is_sample_variable` / `is_action_variable`) enable
 * `static_assert` composition checks at pack construction time.
 *
 * @see variable_tags.hpp
 * @see sample_cache.hpp
 * @see action_pack.hpp
 */

#pragma once

#include <type_traits>

#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief Base tag: disambiguate bridge variables from other types.
 *
 * @details
 * Empty marker type. Concrete tags inherit SampleVariable or ActionVariable
 * (both derive from Variable) so packs can constrain membership.
 */
struct Variable {
    /**
     * @brief Shared / weak / unique pointer aliases for Variable.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(Variable)
};

/**
 * @brief Sensor-sample slot variable (stored in GenericSampleCache).
 *
 * @details
 * Concrete sample tags (Image, LaserScan, …) live in variable_tags.hpp and
 * inherit this type so GenericSampleCache can static_assert membership.
 *
 * @see GenericSampleCache
 * @see variable::Image
 */
struct SampleVariable : Variable {
    /**
     * @brief Shared / weak / unique pointer aliases for SampleVariable.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(SampleVariable)
};

/**
 * @brief Action policy variable (maps to ActionBackground CRTP leaf).
 *
 * @details
 * Concrete action tags (NavigateToPose, DriveOnHeading, …) inherit this type
 * and are bound to CRTP policies via BRIDGE_BIND_ACTION_TRAITS / action_pack.
 *
 * @see action_traits
 * @see action_pack
 */
struct ActionVariable : Variable {
    /**
     * @brief Shared / weak / unique pointer aliases for ActionVariable.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(ActionVariable)
};

/**
 * @brief Type trait: true when T inherits Variable.
 *
 * @tparam T Candidate type.
 *
 * @note Evaluates `std::is_base_of<Variable, T>` as true_type / false_type.
 */
template <typename T>
struct is_variable
    : std::conditional_t<std::is_base_of<Variable, T>::value, std::true_type,
                         std::false_type> {
    /**
     * @brief Shared / weak / unique pointer aliases for is_variable.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(is_variable<T>)
};

/**
 * @brief Type trait: true when T inherits SampleVariable.
 *
 * @tparam T Candidate type.
 */
template <typename T>
struct is_sample_variable
    : std::conditional_t<std::is_base_of<SampleVariable, T>::value,
                         std::true_type, std::false_type> {
    /**
     * @brief Shared / weak / unique pointer aliases for is_sample_variable.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(is_sample_variable<T>)
};

/**
 * @brief Type trait: true when T inherits ActionVariable.
 *
 * @tparam T Candidate type.
 */
template <typename T>
struct is_action_variable
    : std::conditional_t<std::is_base_of<ActionVariable, T>::value,
                         std::true_type, std::false_type> {
    /**
     * @brief Shared / weak / unique pointer aliases for is_action_variable.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(is_action_variable<T>)
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
