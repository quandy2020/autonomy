#include "autonomy/perception/tasks/lifting/lifting_task.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace autonomy::perception::lifting {
namespace network = autonomy::common::network;

bool DecodeBoxes(
    const network::Tensor& boxes, const network::ModelTensorInfo& metadata,
    const base::proto::BoxerDecodeOptions& options,
    const automsgs::msgs::std_msgs::Header& header,
    automsgs::msgs::vision_msgs::Detection3DArray* output,
    std::string* error) {
    auto fail = [&](const char* detail) {
        if (error)
            *error = detail;
        return false;
    };
    const float* values = nullptr;
    size_t count = 0;
    std::vector<int64_t> shape;
    const auto threshold = options.minimum_score();
    if (!output || header.frame_id().empty() ||
        header.frame_id() != options.frame_id() ||
        !std::isfinite(threshold) || threshold < 0 || threshold > 1 ||
        !boxes.TryViewFloat32(&values, &count, error) || (count && !values) ||
        !network::ResolveShapeForElementCount(metadata, count, &shape, error) ||
        shape.size() != 2 || shape[1] != 12) {
        return fail("Boxer expects float32 boxes [N,12] in the configured frame.");
    }

    automsgs::msgs::vision_msgs::Detection3DArray decoded;
    decoded.mutable_header()->CopyFrom(header);
    for (size_t i = 0; i < count; i += 12) {
        const auto* row = values + i;
        if (!std::all_of(row, row + 12, [](float v) { return std::isfinite(v); }) ||
            row[3] <= 0 || row[4] <= 0 || row[5] <= 0 ||
            row[10] < 0 || row[10] > 1 || row[11] < 0 ||
            static_cast<double>(row[11]) >= options.class_names_size() ||
            std::floor(row[11]) != row[11]) {
            return fail("Invalid Boxer box row.");
        }
        if (row[10] < threshold)
            continue;
        const double norm = std::hypot(std::hypot(row[6], row[7]),
                                       std::hypot(row[8], row[9]));
        if (std::abs(norm - 1.0) > 1e-3)
            return fail("Invalid Boxer quaternion.");

        auto* detection = decoded.add_detections();
        detection->mutable_header()->CopyFrom(header);
        auto* box = detection->mutable_bbox();
        box->mutable_size()->set_x(row[3]);
        box->mutable_size()->set_y(row[4]);
        box->mutable_size()->set_z(row[5]);
        auto* pose = box->mutable_center();
        pose->mutable_position()->set_x(row[0]);
        pose->mutable_position()->set_y(row[1]);
        pose->mutable_position()->set_z(row[2]);
        pose->mutable_orientation()->set_x(row[6] / norm);
        pose->mutable_orientation()->set_y(row[7] / norm);
        pose->mutable_orientation()->set_z(row[8] / norm);
        pose->mutable_orientation()->set_w(row[9] / norm);
        auto* hypothesis = detection->add_results()->mutable_hypothesis();
        hypothesis->set_class_id(options.class_names(static_cast<int>(row[11])));
        hypothesis->set_score(row[10]);
    }
    output->Swap(&decoded);
    return true;
}

}  // namespace autonomy::perception::lifting
