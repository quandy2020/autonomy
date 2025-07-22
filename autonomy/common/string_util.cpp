/*
 * Copyright 2024 The OpenRobotic Beginner Authors
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

#include "autonomy/common/string_util.hpp"

#include <cmath>
#include <vector>
#include <algorithm>
#include <cstdarg>
#include <fstream>
#include <sstream>

#include <boost/algorithm/string.hpp>

#include "absl/strings/str_cat.h"

namespace autonomy {
namespace common {
namespace {

static const char kBase64Array[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

std::string Base64Piece(const char in0, const char in1, const char in2) {
  const int triplet = in0 << 16 | in1 << 8 | in2;
  std::string out(4, '=');
  out[0] = kBase64Array[(triplet >> 18) & 0x3f];
  out[1] = kBase64Array[(triplet >> 12) & 0x3f];
  if (in1) {
    out[2] = kBase64Array[(triplet >> 6) & 0x3f];
  }
  if (in2) {
    out[3] = kBase64Array[triplet & 0x3f];
  }
  return out;
}


void StringAppendV(std::string* dst, const char* format, va_list ap) 
{
  // First try with a small fixed size buffer.
  static const int kFixedBufferSize = 1024;
  char fixed_buffer[kFixedBufferSize];

  // It is possible for methods that use a va_list to invalidate
  // the data in it upon use.  The fix is to make a copy
  // of the structure before using it and use that copy instead.
  va_list backup_ap;
  va_copy(backup_ap, ap);
  int result = vsnprintf(fixed_buffer, kFixedBufferSize, format, backup_ap);
  va_end(backup_ap);

  if (result < kFixedBufferSize) {
    if (result >= 0) {
      // Normal case - everything fits.
      dst->append(fixed_buffer, result);
      return;
    }

#ifdef _MSC_VER
    // Error or MSVC running out of space.  MSVC 8.0 and higher
    // can be asked about space needed with the special idiom below:
    va_copy(backup_ap, ap);
    result = vsnprintf(nullptr, 0, format, backup_ap);
    va_end(backup_ap);
#endif

    if (result < 0) {
      // Just an error.
      return;
    }
  }

  // Increase the buffer size to the size requested by vsnprintf,
  // plus one for the closing \0.
  const int variable_buffer_size = result + 1;
  std::unique_ptr<char[]> variable_buffer(new char[variable_buffer_size]);

  // Restore the va_list before we use it again.
  va_copy(backup_ap, ap);
  result =
      vsnprintf(variable_buffer.get(), variable_buffer_size, format, backup_ap);
  va_end(backup_ap);

  if (result >= 0 && result < variable_buffer_size) {
    dst->append(variable_buffer.get(), result);
  }
}

bool IsNotWhiteSpace(const int character) 
{
  return character != ' ' && character != '\n' && character != '\r' &&
         character != '\t';
}

}  // namespace

std::string EncodeBase64(std::string_view in) 
{
  std::string out;
  if (in.empty()) {
    return out;
  }

  const size_t in_size = in.length();
  out.reserve(((in_size - 1) / 3 + 1) * 4);
  for (size_t i = 0; i + 2 < in_size; i += 3) {
    absl::StrAppend(&out, Base64Piece(in[i], in[i + 1], in[i + 2]));
  }
  if (in_size % 3 == 1) {
    absl::StrAppend(&out, Base64Piece(in[in_size - 1], 0, 0));
  }
  if (in_size % 3 == 2) {
    absl::StrAppend(&out, Base64Piece(in[in_size - 2], in[in_size - 1], 0));
  }
  return out;
}

std::string StringPrintf(const char* format, ...) {
  va_list ap;
  va_start(ap, format);
  std::string result;
  StringAppendV(&result, format, ap);
  va_end(ap);
  return result;
}

std::string StringReplace(const std::string& str,
                          const std::string& old_str,
                          const std::string& new_str) {
  if (old_str.empty()) {
    return str;
  }
  size_t position = 0;
  std::string mod_str = str;
  while ((position = mod_str.find(old_str, position)) != std::string::npos) {
    mod_str.replace(position, old_str.size(), new_str);
    position += new_str.size();
  }
  return mod_str;
}

std::string StringGetAfter(const std::string& str, const std::string& key) {
  if (key.empty()) {
    return str;
  }
  std::size_t found = str.rfind(key);
  if (found != std::string::npos) {
    return str.substr(found + key.length(),
                      str.length() - (found + key.length()));
  }
  return "";
}

std::vector<std::string> StringSplit(const std::string& str,
                                     const std::string& delim) {
  std::vector<std::string> elems;
  // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
  boost::split(elems, str, boost::is_any_of(delim), boost::token_compress_on);
  return elems;
}

bool StringStartsWith(const std::string& str, const std::string& prefix) {
  return !prefix.empty() && prefix.size() <= str.size() &&
         str.substr(0, prefix.size()) == prefix;
}

void StringLeftTrim(std::string* str) {
  str->erase(str->begin(),
             std::find_if(str->begin(), str->end(), IsNotWhiteSpace));
}

void StringRightTrim(std::string* str) {
  str->erase(std::find_if(str->rbegin(), str->rend(), IsNotWhiteSpace).base(),
             str->end());
}

void StringTrim(std::string* str) {
  StringLeftTrim(str);
  StringRightTrim(str);
}

void StringToLower(std::string* str) {
  std::transform(str->begin(), str->end(), str->begin(), ::tolower);
}

void StringToUpper(std::string* str) {
  std::transform(str->begin(), str->end(), str->begin(), ::toupper);
}

bool StringContains(const std::string& str, const std::string& sub_str) {
  return str.find(sub_str) != std::string::npos;
}

}  // namespace common
}  // namespace autonomy
