/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

/**
 * @file computation_request.hpp
 * @brief Types of computation that can be simultaneously requested.
 */


#include <type_traits>

namespace bolza {

/** Types of computation that can be simultaneously requested */
enum class Request { Dynamics = 1, Cost = 2, Constraint = 4, SoftConstraint = 8, Approximation = 16 };

/**
 * A set of computation requests
 * It can be composed by union of two Requests using operator+().
 */
class RequestSet {
 public:
    /**
   * @brief Constructor, implicit conversion allowed 
   * @param [in] computationFlag: Computation Flag.
   */
  constexpr RequestSet(Request computationFlag) : flags_(computationFlag) {}  // NOLINT(google-explicit-constructor)

    /**
   * @brief Check if this Request set contains item 
   * @param [in] item: Item.
   * @return True on success.
   */
  constexpr bool Contains(Request item) const;

    /**
   * @brief Check if this Request set contains any Computation of other (non-empty intersection) 
   * @param [in] other: Other.
   * @return True on success.
   */
  constexpr bool ContainsAny(RequestSet other) const;

    /**
   * @brief Check if this Request set contains all Computations of other (subset) 
   * @param [in] other: Other.
   * @return True on success.
   */
  constexpr bool ContainsAll(RequestSet other) const;

  /** Union of two Request sets */
  friend constexpr RequestSet operator+(RequestSet a, RequestSet b);

 private:
  //! Internal representation of the set through a bitfield
  Request flags_;
};

/** Bitwise | on the underlying type of Request */
constexpr Request operator|(Request lhs, Request rhs) {
  using underlying = typename std::underlying_type<Request>::type;
  return static_cast<Request>(static_cast<underlying>(lhs) | static_cast<underlying>(rhs));
}

/** Bitwise & on the underlying type of Request */
constexpr Request operator&(Request lhs, Request rhs) {
  using underlying = typename std::underlying_type<Request>::type;
  return static_cast<Request>(static_cast<underlying>(lhs) & static_cast<underlying>(rhs));
}

/**
 * @brief Check if the Bitwise representation of two Request has empty union 
 * @param [in] lhs: Lhs.
 * @param [in] rhs: Rhs.
 * @return True on success.
 */
constexpr bool IsUnionEmpty(Request lhs, Request rhs) {
  using underlying = typename std::underlying_type<Request>::type;
  return (static_cast<underlying>(lhs) & static_cast<underlying>(rhs)) == underlying(0);
}

/** Create a RequestSet out of two requests */
constexpr RequestSet operator+(Request a, Request b) {
  return a | b;
}

/**
 * @brief Contains.
 * @param [in] item: Item.
 * @return Result value.
 */
constexpr bool RequestSet::Contains(Request item) const {
  /**
   * @brief Is Union Empty.
   * @param [in] flags_: Flags.
   * @param [in] item: Item.
   * @return Result value.
   */
  return !IsUnionEmpty(flags_, item);
}

/**
 * @brief Contains Any.
 * @param [in] other: Other.
 * @return Result value.
 */
constexpr bool RequestSet::ContainsAny(RequestSet other) const {
  /**
   * @brief Is Union Empty.
   * @param [in] flags_: Flags.
   * @param [in] flags_: Flags.
   * @return Result value.
   */
  return !IsUnionEmpty(flags_, other.flags_);
}

/**
 * @brief Contains All.
 * @param [in] other: Other.
 * @return Result value.
 */
constexpr bool RequestSet::ContainsAll(RequestSet other) const {
  return (flags_ & other.flags_) == other.flags_;
}

constexpr RequestSet operator+(RequestSet a, RequestSet b) {
  return {a.flags_ | b.flags_};
}

}  // namespace bolza
