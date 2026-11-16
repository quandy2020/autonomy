#ifndef AUTONOMY_PERCEPTION_TASKS_LIFTING_LIFTING_TASK_HPP_
#define AUTONOMY_PERCEPTION_TASKS_LIFTING_LIFTING_TASK_HPP_

#include <string>

#include <automsgs/msgs/vision_msgs/detection3d_array.pb.h>
#include "autonomy/common/network/common/tensor.hpp"
#include "autonomy/perception/proto/perception.pb.h"

namespace autonomy::perception::lifting {

// Each box row is [cx,cy,cz,sx,sy,sz,qx,qy,qz,qw,score,class_index].
// Centre, extents and orientation must already use the request frame.
bool DecodeBoxes(
    const common::network::Tensor& boxes,
    const common::network::ModelTensorInfo& metadata,
    const base::proto::BoxerDecodeOptions& options,
    const automsgs::msgs::std_msgs::Header& header,
    automsgs::msgs::vision_msgs::Detection3DArray* output,
    std::string* error);

}  // namespace autonomy::perception::lifting

#endif
