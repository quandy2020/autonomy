// `nav_msgs/Path` → `foxglove::schemas::SceneUpdate`（线带）。
#pragma once

#include "automsgs/msgs/nav_msgs/path.pb.h"

#if defined(AUTOVIZ_HAS_FOXGLOVE) && AUTOVIZ_HAS_FOXGLOVE && \
    defined(AUTOVIZ_FOXGLOVE_HAS_ENCODE) && AUTOVIZ_FOXGLOVE_HAS_ENCODE
#include <foxglove/schemas.hpp>
#endif

namespace autoviz {
namespace converter {

#if defined(AUTOVIZ_HAS_FOXGLOVE) && AUTOVIZ_HAS_FOXGLOVE && \
    defined(AUTOVIZ_FOXGLOVE_HAS_ENCODE) && AUTOVIZ_FOXGLOVE_HAS_ENCODE

/// nav_msgs/Path -> SceneUpdate（LinePrimitive LINE_STRIP），与 autonomy 中 planning_msgs 转换思路一致。
foxglove::schemas::SceneUpdate PathToSceneUpdate(const automsgs::msgs::nav_msgs::Path& path);

#endif

}  // namespace converter
}  // namespace autoviz
