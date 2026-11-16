#include "autonomy/perception/inference/image_input.hpp"

#include "autonomy/perception/tasks/task_base.hpp"

#include <algorithm>
#include <limits>

#include <opencv2/core.hpp>

namespace autonomy::perception::base {

bool RunImageInference(
    const common::network::Engine& engine, const proto::BaseOptions& options,
    const automsgs::msgs::sensor_msgs::Image& rgb,
    common::network::ResizePolicy resize, common::network::RunResult* result,
    std::string* error) {
    if (rgb.width() == 0 || rgb.height() == 0 ||
        rgb.width() > std::numeric_limits<int>::max() ||
        rgb.height() > std::numeric_limits<int>::max() ||
        rgb.step() < 3ULL * rgb.width() ||
        rgb.data().size() < static_cast<uint64_t>(rgb.step()) * rgb.height() ||
        (rgb.encoding() != "rgb8" && rgb.encoding() != "bgr8")) {
        SetTaskError(error, "Invalid RGB input.");
        return false;
    }

    cv::Mat image(rgb.height(), rgb.width(), CV_8UC3,
                  const_cast<char*>(rgb.data().data()), rgb.step());
    common::network::PreprocessOptions prep;
    prep.resize = resize;
    prep.default_width = options.input_width();
    prep.default_height = options.input_height();
    prep.swap_red_blue = rgb.encoding() == "bgr8";
    return common::network::RunPipeline(
        const_cast<common::network::Engine*>(&engine), image, prep, result, error);
}

bool ReadPrimaryOutput(
    const common::network::RunResult& result,
    const std::vector<common::network::ModelTensorInfo>& infos,
    const std::string& name, const float** data, size_t* count,
    std::vector<int64_t>* shape,
    std::string* error) {
    const auto output = result.outputs.find(name);
    const auto info = std::find_if(infos.begin(), infos.end(), [&](const auto& item) {
        return item.name == name;
    });
    if (output == result.outputs.end() || info == infos.end() ||
        !output->second.TryViewFloat32(data, count, error) || !*data) {
        SetTaskError(error, "Missing or invalid float32 output tensor: " + name);
        return false;
    }
    return common::network::ResolveShapeForElementCount(*info, *count, shape, error);
}

}  // namespace autonomy::perception::base

