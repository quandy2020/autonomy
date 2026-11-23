/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>
#include <string_view>

namespace autoviz {
namespace bridge {

inline std::string Base64Encode(std::string_view input) {
  constexpr const char kAlphabet[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string result;
  result.reserve((input.size() + 2) / 3 * 4);
  const auto* data = reinterpret_cast<const unsigned char*>(input.data());
  size_t i = 0;
  for (; i + 2 < input.size(); i += 3) {
    result.push_back(kAlphabet[data[i] >> 2]);
    result.push_back(
        kAlphabet[static_cast<int>(((data[i] & 0x03) << 4) |
                                   (data[i + 1] >> 4))]);
    result.push_back(
        kAlphabet[static_cast<int>(((data[i + 1] & 0x0f) << 2) |
                                   (data[i + 2] >> 6))]);
    result.push_back(kAlphabet[data[i + 2] & 0x3f]);
  }
  if (input.size() - i == 2) {
    result.push_back(kAlphabet[data[i] >> 2]);
    result.push_back(
        kAlphabet[static_cast<int>(((data[i] & 0x03) << 4) |
                                   (data[i + 1] >> 4))]);
    result.push_back(kAlphabet[static_cast<int>((data[i + 1] & 0x0f) << 2)]);
    result.push_back('=');
  } else if (input.size() - i == 1) {
    result.push_back(kAlphabet[data[i] >> 2]);
    result.push_back(kAlphabet[static_cast<int>((data[i] & 0x03) << 4)]);
    result.push_back('=');
    result.push_back('=');
  }
  return result;
}

}  // namespace bridge
}  // namespace autoviz
