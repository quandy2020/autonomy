/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/controller/policy_controller/config.hpp"

#include <fstream>
#include <sstream>

#include <google/protobuf/text_format.h>

#include "autonomy/control/controller/policy_controller/diffusion.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {
namespace {

bool ShapeCompatible(const std::vector<int64_t>& expected,
                     const common::network::TensorShape& actual) {
    if (expected.empty()) {
        return true;
    }
    if (expected.size() != actual.Rank()) {
        return false;
    }
    int dynamic_axes = 0;
    for (std::size_t i = 0; i < expected.size(); ++i) {
        const int64_t expect = expected[i];
        const int64_t actual_dim = actual.Dims()[i];
        if (expect <= 0) {
            ++dynamic_axes;
            continue;
        }
        if (actual_dim > 0 && expect != actual_dim) {
            return false;
        }
    }
    return dynamic_axes <= 1;
}

struct ObservationSlotDefaults {
    proto::ObservationModality modality;
    const char* name;
    std::vector<int64_t> shape;
    const char* preprocess;  // nullptr if unused
};

void AppendDefaultShape(proto::ObservationInput* input,
                        const std::vector<int64_t>& shape) {
    for (const int64_t dim : shape) {
        input->add_shape(dim);
    }
}

void ExpandObservationSlot(const proto::ObservationInput& slot,
                           const ObservationSlotDefaults& defaults,
                           proto::PolicyConfig* proto) {
    auto* input = proto->add_inputs();
    input->set_modality(defaults.modality);
    input->set_name(slot.name().empty() ? defaults.name : slot.name());
    if (slot.shape_size() > 0) {
        *input->mutable_shape() = slot.shape();
    } else {
        AppendDefaultShape(input, defaults.shape);
    }
    input->set_required(slot.has_required() ? slot.required() : true);
    if (slot.has_norm()) {
        *input->mutable_norm() = slot.norm();
    }
    if (!slot.preprocess().empty()) {
        input->set_preprocess(slot.preprocess());
    } else if (defaults.preprocess != nullptr) {
        input->set_preprocess(defaults.preprocess);
    }
}

void MaybeExpand(bool enabled, const proto::ObservationInput* options,
                 const ObservationSlotDefaults& defaults,
                 proto::PolicyConfig* proto) {
    if (!enabled) {
        return;
    }
    static const proto::ObservationInput kEmptySlot;
    ExpandObservationSlot(options != nullptr ? *options : kEmptySlot, defaults,
                          proto);
}

// Expand `observation { kinematic: true path: true ... }` into inputs[].
bool ExpandObservationConfig(proto::PolicyConfig* proto, std::string* error) {
    if (proto == nullptr || !proto->has_observation()) {
        return true;
    }
    const auto& config = proto->observation();
    const bool any = config.kinematic() || config.path() || config.costmap() ||
                     config.laser() || config.image() || config.point_cloud() ||
                     config.lidar_bev() || config.raw();
    if (!any) {
        if (error != nullptr) {
            *error =
                "observation has no enabled modality "
                "(set kinematic/path/costmap/laser/image/point_cloud/"
                "lidar_bev/raw to true)";
        }
        return false;
    }
    if (proto->inputs_size() > 0) {
        if (error != nullptr) {
            *error =
                "Use either observation { ... } or inputs { ... }, not both";
        }
        return false;
    }

    MaybeExpand(config.kinematic(),
                config.has_kinematic_input() ? &config.kinematic_input() : nullptr,
                {proto::OBSERVATION_KINEMATIC,
                 "observation_kinematic",
                 {1, 6},
                 nullptr},
                proto);
    MaybeExpand(config.path(),
                config.has_path_input() ? &config.path_input() : nullptr,
                {proto::OBSERVATION_PATH, "observation_path", {1, 40}, nullptr},
                proto);
    MaybeExpand(config.costmap(),
                config.has_costmap_input() ? &config.costmap_input() : nullptr,
                {proto::OBSERVATION_COSTMAP,
                 "observation_costmap",
                 {1, 1, 64, 64},
                 nullptr},
                proto);
    MaybeExpand(config.laser(),
                config.has_laser_input() ? &config.laser_input() : nullptr,
                {proto::OBSERVATION_LASER,
                 "observation_laser",
                 {1, 180},
                 nullptr},
                proto);
    MaybeExpand(config.image(),
                config.has_image_input() ? &config.image_input() : nullptr,
                {proto::OBSERVATION_IMAGE,
                 "observation_image",
                 {1, 3, 224, 224},
                 "letterbox"},
                proto);
    MaybeExpand(config.point_cloud(),
                config.has_point_cloud_input() ? &config.point_cloud_input()
                                              : nullptr,
                {proto::OBSERVATION_POINT_CLOUD,
                 "observation_point_cloud",
                 {1, 2048, 3},
                 nullptr},
                proto);
    MaybeExpand(config.lidar_bev(),
                config.has_lidar_bev_input() ? &config.lidar_bev_input() : nullptr,
                {proto::OBSERVATION_LIDAR_BEV,
                 "observation_lidar_bev",
                 {1, 1, 200, 200},
                 nullptr},
                proto);
    MaybeExpand(config.raw(),
                config.has_raw_input() ? &config.raw_input() : nullptr,
                {proto::OBSERVATION_RAW, "observation_raw", {1, 1}, nullptr},
                proto);
    return true;
}

}  // namespace

bool PolicyConfig::LoadFromFile(const std::string& path, PolicyConfig* out,
                                std::string* error) {
    if (out == nullptr) {
        return false;
    }
    std::ifstream input(path);
    if (!input) {
        if (error != nullptr) {
            *error = "Failed to open PolicyConfig: " + path;
        }
        return false;
    }
    std::ostringstream buffer;
    buffer << input.rdbuf();
    proto::PolicyConfig proto;
    if (!google::protobuf::TextFormat::ParseFromString(buffer.str(), &proto)) {
        proto.Clear();
        std::ifstream binary(path, std::ios::binary);
        if (!proto.ParseFromIstream(&binary)) {
            if (error != nullptr) {
                *error =
                    "Failed to parse PolicyConfig text/binary: " + path;
            }
            return false;
        }
    }
    return LoadFromProto(proto, out, error);
}

bool PolicyConfig::LoadFromProto(const proto::PolicyConfig& proto,
                                 PolicyConfig* out, std::string* error) {
    if (out == nullptr) {
        return false;
    }
    proto::PolicyConfig normalized = proto;
    if (!ExpandObservationConfig(&normalized, error)) {
        return false;
    }
    std::string schema_error;
    ActionSchema schema = ActionSchema::FromConfig(normalized, &schema_error);
    if (schema.tensor_name.empty()) {
        if (error != nullptr) {
            *error = schema_error.empty() ? "Invalid action schema"
                                          : schema_error;
        }
        return false;
    }
    if (normalized.inputs_size() == 0) {
        if (error != nullptr) {
            *error =
                "PolicyConfig has no inputs (set observation { ... } "
                "or inputs { ... })";
        }
        return false;
    }
    out->proto_ = std::move(normalized);
    out->action_schema_ = std::move(schema);
    out->effective_diffusion_ = MakeEffectiveDiffusion(out->proto_);
    const auto mode = out->proto_.mode();
    out->is_diffusion_ =
        mode == proto::INFERENCE_DIFFUSION ||
        out->proto_.action().type() == proto::ACTION_SPACE_DIFFUSION_TWIST ||
        (mode == proto::INFERENCE_UNSPECIFIED && out->proto_.has_diffusion() &&
         out->proto_.diffusion().steps() > 1) ||
        out->action_schema_.type == proto::ACTION_SPACE_DIFFUSION_TWIST;
    return true;
}

common::network::InferenceOptions PolicyConfig::ToInferenceOptions() const {
    common::network::InferenceOptions options;
    const auto& engine = proto_.engine();
    options.backend_id = engine.backend().empty() ? "onnx" : engine.backend();
    options.model_path = engine.model_path();
    options.onnx.execution_provider = engine.provider();
    options.onnx.device_id = engine.device();
    options.onnx.use_io_binding = engine.io_binding();
    if (options.backend_id == "tensorrt") {
        options.tensorrt.device_id = engine.device();
    }
    return options;
}

bool PolicyConfig::ValidateAgainstEngine(
    const std::vector<common::network::ModelTensorInfo>& inputs,
    const std::vector<common::network::ModelTensorInfo>& outputs,
    std::string* error) const {
    auto find_info =
        [](const std::vector<common::network::ModelTensorInfo>& infos,
           const std::string& name) -> const common::network::ModelTensorInfo* {
        for (const auto& info : infos) {
            if (info.name == name) {
                return &info;
            }
        }
        return nullptr;
    };

    for (const auto& input : proto_.inputs()) {
        if (input.name().empty()) {
            if (error != nullptr) {
                *error = "Input name is empty";
            }
            return false;
        }
        if (!input.required()) {
            continue;
        }
        const auto* info = find_info(inputs, input.name());
        if (info == nullptr) {
            if (error != nullptr) {
                *error = "Required input missing in engine: " + input.name();
            }
            return false;
        }
        std::vector<int64_t> expected(input.shape().begin(),
                                      input.shape().end());
        if (!ShapeCompatible(expected, info->shape)) {
            if (error != nullptr) {
                *error = "Input shape mismatch for " + input.name();
            }
            return false;
        }
    }

    std::string prediction_name = action_schema_.tensor_name;
    if (is_diffusion_) {
        const auto& diffusion = effective_diffusion_;
        if (find_info(inputs, diffusion.sample()) == nullptr) {
            if (error != nullptr) {
                *error = "Diffusion sample input missing in engine: " +
                         diffusion.sample();
            }
            return false;
        }
        if (find_info(inputs, diffusion.timestep()) == nullptr) {
            if (error != nullptr) {
                *error = "Diffusion timestep input missing in engine: " +
                         diffusion.timestep();
            }
            return false;
        }
        if (!diffusion.prediction().empty()) {
            prediction_name = diffusion.prediction();
        }
    }

    const auto* output_info = find_info(outputs, prediction_name);
    if (output_info == nullptr) {
        if (error != nullptr) {
            *error = "Action/prediction output missing in engine: " +
                     prediction_name;
        }
        return false;
    }
    const std::size_t need = is_diffusion_ ? action_schema_.SequenceSize()
                                           : action_schema_.slots.size();
    const std::size_t have = output_info->shape.ProductPositiveDims();
    if (have > 0 && have < need) {
        if (error != nullptr) {
            *error = "Action output shorter than required layout / horizon";
        }
        return false;
    }
    return true;
}

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
