#ifndef AUTONOMY_PERCEPTION_INFERENCE_IMAGE_INPUT_HPP_
#define AUTONOMY_PERCEPTION_INFERENCE_IMAGE_INPUT_HPP_

#include <vector>

#include "autonomy/common/network/network.hpp"
#include "autonomy/perception/proto/perception.pb.h"
#include <automsgs/msgs/sensor_msgs/image.pb.h>

namespace autonomy::perception::base {

bool RunImageInference(
    const common::network::Engine& engine, const proto::BaseOptions& options,
    const automsgs::msgs::sensor_msgs::Image& rgb,
    common::network::ResizePolicy resize, common::network::RunResult* result,
    std::string* error);

bool ReadPrimaryOutput(
    const common::network::RunResult& result,
    const std::vector<common::network::ModelTensorInfo>& infos,
    const std::string& name, const float** data, size_t* count,
    std::vector<int64_t>* shape,
    std::string* error);

}  // namespace autonomy::perception::base

#endif  // AUTONOMY_PERCEPTION_INFERENCE_IMAGE_INPUT_HPP_
