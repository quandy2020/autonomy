#include "autolink/autolink.hpp"
#include "autonomy/perception/tasks/scene_graph/room_attribution.hpp"
#include "autonomy/perception/tasks/lifting/lifting_task.hpp"
#include "autonomy/perception/tasks/room_segmentation/room_segmentation_task.hpp"

#include <algorithm>
#include "autonomy/perception/models/boxer/boxer_model.hpp"
#include "autonomy/perception/models/flashocc/flashocc_model.hpp"
#include "autonomy/perception/models/sam2/sam2_model.hpp"
#include "autonomy/perception/models/segformer/segformer_model.hpp"
#include "autonomy/perception/models/yolo/yolo_model.hpp"
#include "autolink/common/file.hpp"
#include "autonomy/perception/engine/options.hpp"
#include "autonomy/perception/models/model_registry.hpp"
#include "autonomy/perception/proto/perception.pb.h"

#include <iostream>
#include <mutex>
#include <optional>
#include <unordered_set>
#include <utility>

namespace {

namespace proto = autonomy::perception::base::proto;
namespace network = autonomy::common::network;
namespace models = autonomy::perception::models;

constexpr size_t kMaxRequestBytes = 64 * 1024 * 1024;

std::optional<network::ElementType> Element(proto::ObjectTensorType type) {
    switch (type) {
        case proto::OBJECT_TENSOR_FLOAT32: return network::ElementType::kFloat32;
        case proto::OBJECT_TENSOR_INT64: return network::ElementType::kInt64;
        case proto::OBJECT_TENSOR_INT32: return network::ElementType::kInt32;
        case proto::OBJECT_TENSOR_UINT8: return network::ElementType::kUint8;
        default: return std::nullopt;
    }
}

struct Stage {
    proto::ObjectGraphStage config;
    std::unique_ptr<models::ModelBase> model;
};

class ObjectInference {
public:
    bool Load(const proto::ObjectInferenceOptions& config, std::string* error) {
        std::unordered_set<std::string> ids;
        std::unordered_set<std::string> available;
        for (const auto& stage : config.stages()) {
            if (stage.id().empty() || stage.id().find('.') != std::string::npos ||
                !ids.insert(stage.id()).second || stage.model_path().empty() ||
                stage.inputs().empty() || stage.outputs().empty()) {
                *error = "Invalid object graph stage.";
                return false;
            }

            std::vector<std::string> inputs;
            for (const auto& entry : stage.inputs()) {
                const auto& source = entry.second;
                if (source.empty() ||
                    (source.find('.') != std::string::npos && !available.count(source))) {
                    *error = "Unknown preceding output: " + source;
                    return false;
                }
                inputs.push_back(entry.first);
            }
            std::vector<std::string> outputs(stage.outputs().begin(), stage.outputs().end());
            auto load = models::LoadGraphModel;
            switch (stage.family()) {
                case proto::MODEL_FAMILY_YOLO: load = models::LoadYoloModel; break;
                case proto::MODEL_FAMILY_SEG_FORMER: load = models::LoadSegFormerModel; break;
                case proto::MODEL_FAMILY_SAM2: load = models::LoadSam2Model; break;
                case proto::MODEL_FAMILY_FLASHOCC: load = models::LoadFlashOccModel; break;
                case proto::MODEL_FAMILY_BOXER: load = models::LoadBoxerModel; break;
                case proto::MODEL_FAMILY_RRCE:
                case proto::MODEL_FAMILY_MOGE:
                case proto::MODEL_FAMILY_UNSPECIFIED: break;
                default:
                    *error = "Unrecognized graph model family.";
                    return false;
            }
            auto model = load(stage.id(), stage.model_path(),
                autonomy::perception::base::BackendId(stage.backend()),
                inputs, outputs, error);
            if (!model)
                return false;
            if (model->inputs().size() != inputs.size()) {
                *error = "Every input must be bound for stage " + stage.id();
                return false;
            }
            for (const auto& out : outputs)
                available.insert(stage.id() + "." + out);
            stages_.push_back({stage, std::move(model)});
        }
        if (stages_.empty()) {
            *error = "At least one inference stage is required.";
            return false;
        }
        if (config.has_boxer_decode()) {
            const auto& final = stages_.back().config;
            const auto& decode = config.boxer_decode();
            if (final.family() != proto::MODEL_FAMILY_BOXER ||
                decode.frame_id().empty() || decode.class_names().empty() ||
                std::find(final.outputs().begin(), final.outputs().end(), decode.boxes_output()) ==
                    final.outputs().end()) {
                *error = "Boxer decoding requires a final Boxer stage, output, frame and class list.";
                return false;
            }
            boxer_decode_ = decode;
        }
        return true;
    }

    void Run(const proto::ObjectInferenceRequest& request,
             proto::ObjectInferenceResponse* response) {
        std::lock_guard<std::mutex> lock(mutex_);
        response->mutable_header()->CopyFrom(request.header());
        if (request.ByteSizeLong() > kMaxRequestBytes) {
            response->set_error("Object inference request is too large.");
            return;
        }
        if (request.has_room_snapshot()) {
            const auto& snapshot = request.room_snapshot();
            if (!boxer_decode_ || request.header().frame_id().empty() ||
                snapshot.frame_id() != request.header().frame_id() ||
                request.map_revision().empty() ||
                snapshot.map_revision() != request.map_revision()) {
                response->set_error("Room snapshot frame or map revision does not match Boxer input.");
                return;
            }
        }
        response->set_map_revision(request.map_revision());
        namespace scene = autonomy::perception::scene_graph;
        std::optional<scene::RoomMapTransform> grid_transform;
        network::TensorMap values;
        if (request.has_occupancy_grid()) {
            if (stages_.size() != 1 ||
                stages_.front().config.family() != proto::MODEL_FAMILY_RRCE ||
                request.inputs_size() != 0 || request.has_room_map()) {
                response->set_error("Use an occupancy grid alone with a single RRCE stage.");
                return;
            }
            const auto& stage = stages_.front();
            const auto metadata = stage.model->inputs();
            if (metadata.size() != 1) {
                response->set_error("RRCE grid mode requires a single graph input.");
                return;
            }
            std::string error;
            network::Tensor tensor;
            grid_transform.emplace();
            if (!autonomy::perception::room_segmentation::PrepareRoomGrid(
                    request.occupancy_grid(), metadata.front(), &tensor,
                    &*grid_transform, &error)) {
                response->set_error(error);
                return;
            }
            values.emplace(stage.config.inputs().begin()->second, std::move(tensor));
            response->mutable_header()->CopyFrom(request.occupancy_grid().header());
        }
        for (const auto& input : request.inputs()) {
            auto type = Element(input.type());
            if (!type || input.name().empty() || input.name().find('.') != std::string::npos ||
                input.data().empty() ||
                input.data().size() % network::ElementTypeByteSize(*type) != 0 ||
                !values.emplace(input.name(), network::Tensor::FromBytes(
                    *type, std::vector<uint8_t>(input.data().begin(), input.data().end()))).second) {
                response->set_error("Invalid or duplicate request tensor: " + input.name());
                return;
            }
        }
        for (const auto& stage : stages_) {
            network::TensorMap inputs;
            for (const auto& entry : stage.config.inputs()) {
                const auto value = values.find(entry.second);
                if (value == values.end()) {
                    response->set_error("Missing tensor: " + entry.second);
                    return;
                }
                inputs.emplace(entry.first, value->second);
            }
            network::TensorMap outputs;
            std::string error;
            if (!stage.model->Infer(inputs, &outputs, &error)) {
                response->set_error(stage.config.id() + ": " + error);
                return;
            }
            for (const auto& port : stage.config.outputs()) {
                auto output = outputs.find(port);
                if (output == outputs.end()) {
                    response->set_error("Missing model output: " + port);
                    return;
                }
                values.emplace(stage.config.id() + "." + port, std::move(output->second));
            }
        }
        const auto& final = stages_.back().config;
        if (final.family() == proto::MODEL_FAMILY_RRCE) {
            const auto& map = request.room_map();
            const auto vertices = values.find(final.id() + ".vertices");
            const auto counts = values.find(final.id() + ".vertex_counts");
            if ((!grid_transform && (!request.has_room_map() ||
                                      map.model_to_grid_size() != 6 ||
                                      request.header().frame_id().empty())) ||
                vertices == values.end() || counts == values.end()) {
                response->set_error("RRCE needs map geometry, vertices and vertex_counts.");
                return;
            }
            if (!grid_transform) {
                grid_transform = scene::RoomMapTransform{
                    {}, map.metres_per_cell(), {map.origin_x(), map.origin_y()}, map.yaw_rad()};
                std::copy(map.model_to_grid().begin(), map.model_to_grid().end(),
                          grid_transform->model_to_grid.begin());
            }
            std::vector<scene::RoomGeometry> rooms;
            std::string error;
            if (!autonomy::perception::room_segmentation::DecodeRooms(
                    vertices->second, counts->second, *grid_transform, &rooms, &error)) {
                response->set_error(error);
                return;
            }
            for (const auto& room : rooms) {
                auto* output = response->add_rooms();
                for (const auto& point : room.contour) {
                    auto* vertex = output->add_contour();
                    vertex->set_x(point.x);
                    vertex->set_y(point.y);
                }
                output->mutable_center()->set_x(room.center.x);
                output->mutable_center()->set_y(room.center.y);
                output->set_width_m(room.width_m);
                output->set_height_m(room.height_m);
                output->set_yaw_rad(room.yaw_rad);
            }
            return;
        }
        if (boxer_decode_) {
            const auto& name = boxer_decode_->boxes_output();
            const auto metadata = stages_.back().model->supported_outputs();
            const auto port = std::find_if(metadata.begin(), metadata.end(),
                [&](const auto& output) { return output.name == name; });
            std::string error;
            if (port == metadata.end()) {
                response->set_error("Missing Boxer output metadata.");
                return;
            }
            if (!autonomy::perception::lifting::DecodeBoxes(
                    values.at(final.id() + "." + name), *port, *boxer_decode_,
                    request.header(), response->mutable_detections_3d(), &error)) {
                response->clear_detections_3d();
                response->set_error(error);
                return;
            }
            if (request.has_room_snapshot()) {
                std::unordered_set<std::string> ids;
                std::vector<scene::RoomAnnotation> annotations;
                for (const auto& room : request.room_snapshot().rooms()) {
                    if (room.room_id().empty() || !ids.insert(room.room_id()).second ||
                        room.contour_size() < 3) {
                        response->clear_detections_3d();
                        response->set_error("Invalid room snapshot ID or contour.");
                        return;
                    }
                    scene::RoomAnnotation annotation{room.room_id(), {}};
                    annotation.contour.reserve(room.contour_size());
                    for (const auto& vertex : room.contour())
                        annotation.contour.push_back({vertex.x(), vertex.y()});
                    annotations.push_back(std::move(annotation));
                }
                const auto& detections = response->detections_3d().detections();
                for (int i = 0; i < detections.size(); ++i) {
                    const auto& center = detections.Get(i).bbox().center().position();
                    const auto room = scene::FindRoom({center.x(), center.y()}, annotations);
                    if (room) {
                        auto* membership = response->add_room_memberships();
                        membership->set_detection_index(static_cast<uint32_t>(i));
                        membership->set_room_id(*room);
                    }
                }
            }
            return;
        }
        size_t result_bytes = 0;
        for (const auto& name : final.outputs()) {
            const auto& value = values.at(final.id() + "." + name);
            proto::ObjectTensorType type = proto::OBJECT_TENSOR_UNSPECIFIED;
            for (const auto candidate : {proto::OBJECT_TENSOR_FLOAT32,
                                         proto::OBJECT_TENSOR_INT64,
                                         proto::OBJECT_TENSOR_INT32,
                                         proto::OBJECT_TENSOR_UINT8}) {
                if (Element(candidate) == value.element_type())
                    type = candidate;
            }
            if (type == proto::OBJECT_TENSOR_UNSPECIFIED ||
                value.byte_size() > kMaxRequestBytes - result_bytes) {
                response->clear_outputs();
                response->set_error("Unsupported output type or output size.");
                return;
            }
            result_bytes += value.byte_size();
            auto* output = response->add_outputs();
            output->set_name(name);
            output->set_type(type);
            output->set_data(reinterpret_cast<const char*>(value.bytes()), value.byte_size());
        }
    }

private:
    std::mutex mutex_;
    std::vector<Stage> stages_;
    std::optional<proto::BoxerDecodeOptions> boxer_decode_;
};

}  // namespace

int main(int argc, char** argv) {
    proto::ObjectInferenceOptions config;
    if (argc != 2 || !autolink::common::GetProtoFromFile(argv[1], &config) ||
        config.service().empty() || config.service().front() != '/') {
        std::cerr << "Usage: autonomy.perception_object <options.pb.txt>\n";
        return 2;
    }
    ObjectInference inference;
    std::string error;
    if (!inference.Load(config, &error)) {
        std::cerr << "Cannot load object inference: " << error << '\n';
        return 2;
    }
    if (!autolink::Init(argv[0]))
        return 2;
    auto node = autolink::CreateNode("perception_object_inference");
    if (!node)
        return 2;
    auto service = node->CreateService<proto::ObjectInferenceRequest, proto::ObjectInferenceResponse>(
        config.service(),
        [&](const std::shared_ptr<proto::ObjectInferenceRequest>& request,
            std::shared_ptr<proto::ObjectInferenceResponse>& response) {
            inference.Run(*request, response.get());
        });
    if (!service)
        return 2;
    autolink::WaitForShutdown();
    return 0;
}
