/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/common/endian.hpp"

#include <cstdint>

#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace common {

namespace {

bool IsLittleEndianRuntime() {
    const uint16_t value = 0x0100;
    return *reinterpret_cast<const uint8_t*>(&value) == 0;
}

}  // namespace

bool IsLittleEndian() {
#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__) && \
    defined(__ORDER_BIG_ENDIAN__)
    return __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__;
#else
    return IsLittleEndianRuntime();
#endif
}

bool IsBigEndian() {
#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__) && \
    defined(__ORDER_BIG_ENDIAN__)
    return __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
#else
    return !IsLittleEndianRuntime();
#endif
}

}  // namespace common
}  // namespace autonomy
