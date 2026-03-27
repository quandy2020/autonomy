// 将 foxglove-sdk C++ schemas 编码为 protobuf 二进制，供 RawChannel 使用。
#pragma once

#include <string>

#if defined(AUTOVIZ_HAS_FOXGLOVE) && AUTOVIZ_HAS_FOXGLOVE && \
    defined(AUTOVIZ_FOXGLOVE_HAS_ENCODE) && AUTOVIZ_FOXGLOVE_HAS_ENCODE

#include <foxglove/error.hpp>
#include <foxglove/schemas.hpp>

namespace autoviz {
namespace converter {

template <typename FoxgloveMsgT>
inline std::string SerializeFoxglove(FoxgloveMsgT msg) {
  size_t required = 0;
  foxglove::FoxgloveError err = msg.encode(nullptr, 0, &required);
  if (err != foxglove::FoxgloveError::Ok &&
      err != foxglove::FoxgloveError::BufferTooShort) {
    return {};
  }
  std::string out;
  out.resize(required > 0 ? required : 4096);
  size_t written = 0;
  err = msg.encode(reinterpret_cast<uint8_t*>(out.data()), out.size(), &written);
  if (err == foxglove::FoxgloveError::BufferTooShort) {
    out.resize(written);
    err = msg.encode(reinterpret_cast<uint8_t*>(out.data()), out.size(), &written);
  }
  if (err != foxglove::FoxgloveError::Ok) {
    return {};
  }
  out.resize(written);
  return out;
}

inline std::string SerializeSceneUpdate(foxglove::schemas::SceneUpdate msg) {
  return SerializeFoxglove(std::move(msg));
}

inline std::string SerializeLog(foxglove::schemas::Log msg) {
  return SerializeFoxglove(std::move(msg));
}

}  // namespace converter
}  // namespace autoviz

#endif
