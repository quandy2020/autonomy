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
#include "autonomy/common/logging.hpp"


#include <boost/predef/other/endian.h>

namespace autonomy {
namespace common {

bool IsLittleEndian() {
#if BOOST_ENDIAN_LITTLE_BYTE
  return true;
#elif BOOST_ENDIAN_LITTLE_WORD
  // We do not support such exotic architectures.
  LOG(FATAL) << "Unsupported byte ordering";
#else
  return false;
#endif
}

bool IsBigEndian() {
#if BOOST_ENDIAN_BIG_BYTE
  return true;
#elif BOOST_ENDIAN_BIG_WORD
  // We do not support such exotic architectures.
  LOG(FATAL) << "Unsupported byte ordering";
#else
  return false;
#endif
}

}  // namespace common
}  // namespace autonomy

