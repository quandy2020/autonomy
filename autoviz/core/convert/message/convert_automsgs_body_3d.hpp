// automsgs → Foxglove（激光/点云/3D 等）：由 ConvertMessageToFoxglove 调度。
#pragma once

#if defined(AUTOVIZ_HAS_FOXGLOVE) && AUTOVIZ_HAS_FOXGLOVE && \
    defined(AUTOVIZ_FOXGLOVE_HAS_ENCODE) && AUTOVIZ_FOXGLOVE_HAS_ENCODE

#include "autonomy/autoviz/core/convert/message/convert_automsgs_message.hpp"
#include "autonomy/autoviz/core/convert/registry/automsgs_foxglove_registry.hpp"

namespace autoviz {
namespace converter {

/// 返回 true 表示已走专用路径（`out->ok` 表示 encode 是否成功）；false 时上层回退 `BuildFallbackLog`。
bool TryConvertBodyToFoxglove(AutomsgsFoxgloveStrategy strategy, const std::string& proto_full_name,
                              const void* serialized_data, std::size_t serialized_size,
                              FoxgloveConvertedMessage* out);

}  // namespace converter
}  // namespace autoviz

#endif
