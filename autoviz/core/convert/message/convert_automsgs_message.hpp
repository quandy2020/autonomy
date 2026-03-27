// 将任意 automsgs 消息字节转为 Foxglove 负载（策略见 registry）。
#pragma once

#include <cstddef>
#include <string>

namespace autoviz {
namespace converter {

struct FoxgloveConvertedMessage {
  bool ok{false};
  std::string schema_name;
  std::string schema_encoding{"protobuf"};
  std::string schema_data;
  std::string payload;
};

/// 按 `FindFoxgloveRule(proto_full_name)` 的策略转换。
bool ConvertMessageToFoxglove(const std::string& proto_full_name, const void* serialized_data,
                              std::size_t serialized_size, FoxgloveConvertedMessage* out);

inline bool ConvertMessageToFoxglove(const std::string& proto_full_name,
                                     const std::string& serialized, FoxgloveConvertedMessage* out) {
  return ConvertMessageToFoxglove(proto_full_name, serialized.data(), serialized.size(), out);
}

}  // namespace converter
}  // namespace autoviz
