/* Copyright 2026 The Openbot Authors. */
/** @file native_defm_head_test.cpp
 * @brief Consume actual expert depth through native DeFM, RAW history and the trained ONNX step.
 * @details Uses real exported weights and real expert image pixels. Offline
 * fixture timestamps/calibration are controlled test metadata, not SLAM health
 * evidence. No navigation, trajectory execution or command is authorized here.
 */
#include <chrono>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include "nlohmann/json.hpp"
#include "autonomy/common/network/backend/engine.hpp"
#include "autonomy/perception/models/defm/defm_encoder.hpp"
#include "autonomy/control/controller/policy_controller/observation/defm_features.hpp"
#include "autonomy/control/controller/policy_controller/observation/buffer.hpp"
#include "autonomy/control/controller/policy_controller/observation/encoder.hpp"
#include "autonomy/control/controller/policy_controller/observation/tensor_frame_encoder.hpp"
#include "autonomy/control/controller/policy_controller/pipeline.hpp"

namespace {
namespace network = autonomy::common::network;
namespace features = autonomy::perception::defm;
namespace policy = autonomy::control::controller::policy_controller;
namespace proto = autonomy::control::proto;
using Image = automsgs::msgs::sensor_msgs::Image;
/** @brief Fail with the exact inference/input contract, not a fabricated safe result. */
void Require(bool value, const std::string& reason) { if (!value) throw std::runtime_error(reason); }

/** @brief Require native float32 tensor metadata and every declared dimension. */
bool Matches(const network::ModelTensorInfo& tensor, const char* name, const std::vector<int64_t>& shape) {
    return tensor.name == name && tensor.element_type == network::ElementType::kFloat32 && tensor.shape.Dims() == shape;
}

/** @brief Read actual binary automsgs fixture data from this controlled input directory. */
std::shared_ptr<const Image> ReadImage(const std::filesystem::path& directory, const std::string& name) {
    const std::filesystem::path relative(name);
    Require(!relative.is_absolute() && relative == relative.filename(), "native fixture image path escapes its directory");
    std::ifstream input(directory/relative,std::ios::binary); auto image = std::make_shared<Image>();
    Require(input.good() && image->ParseFromIstream(&input), "missing or invalid actual expert Image fixture");
    return image;
}

/** @brief Actual static model loading and exact shared-history input preparation. */
void Consume(const std::string& head_path, const std::string& defm_path, const std::filesystem::path& fixture_path) {
    std::ifstream file(fixture_path); Require(file.good(), "real expert/weight fixture is missing");
    nlohmann::json fixture; file >> fixture;
    Require(fixture.at("schema") == "autonomy.defm.sand.native_fixture.v1" &&
        fixture.at("actual_held_out_expert_images").get<bool>() &&
        fixture.at("controlled_offline_frame_timestamps").get<bool>() &&
        !fixture.at("actual_localization_or_motion_verified").get<bool>(), "wrong native fixture or hidden motion claim");
    network::InferenceOptions inference; inference.model_path = head_path; inference.backend_id = "onnx";
    std::string error; auto head = network::Engine::CreateEngine(inference,&error);
    Require(head != nullptr,error);
    const auto inputs = head->GetInputInfos(); const auto outputs = head->GetOutputInfos();
    Require(inputs.size() == 4 && outputs.size() == 1 && Matches(outputs[0],"action",{1,24}),
        "native learned head has the wrong input/output contract");
    unsigned matched = 0;
    for (const auto& input : inputs) {
        matched += Matches(input,"observation_depth_tokens",{1,4,256,384});
        matched += Matches(input,"observation_goal",{1,3});
        matched += Matches(input,"sample",{1,24}); matched += Matches(input,"timestep",{1});
    }
    Require(matched == 4,"native model metadata does not match the exported float32 token-conditioned step");
    features::DefmEncoderOptions options; options.inference.model_path = defm_path; options.inference.backend_id = "onnx";
    const auto& camera = fixture.at("camera");
    options.calibration.camera = {camera.at("width").get<int>(),camera.at("height").get<int>(),
        camera.at("fx").get<double>(),camera.at("fy").get<double>(),camera.at("cx").get<double>(),camera.at("cy").get<double>(),.1,8};
    options.calibration.frame_id = fixture.at("frame_id").get<std::string>();
    options.calibration.revision = fixture.at("calibration_revision").get<std::uint64_t>();
    options.calibration.rectified = true; options.cache_capacity = 4;
    options.maximum_age = options.decoding.valid_for = 10;  // Explicit offline test lease, not a production age change.
    auto encoder = features::DefmEncoder::Create(options,&error); Require(encoder != nullptr,error);
    proto::ObservationInput input; input.set_name("observation_depth_tokens");
    input.set_modality(proto::OBSERVATION_RAW); input.set_required(true);
    for (int dimension : {1,4,256,384}) input.add_shape(dimension);
    const auto& cases = fixture.at("cases"); Require(!cases.empty() && cases.size() <= 8,"native fixture case budget exceeded");
    const double now = fixture.at("now_sec").get<double>();
    proto::PolicyControllerOptions pipeline_options;
    pipeline_options.set_missing_modality_policy(proto::MISSING_MODALITY_FAIL);
    policy::PolicyPipeline pipeline; pipeline.Configure(pipeline_options);
    proto::PolicyConfig config;
    config.set_name("actual_token_head_input_validation"); config.set_mode(proto::INFERENCE_DIFFUSION);
    config.mutable_engine()->set_backend("onnx"); config.mutable_engine()->set_model_path(head_path);
    config.mutable_action()->set_type(proto::ACTION_SPACE_SAND_PLANNER);
    config.mutable_action()->set_name("action");
    config.mutable_diffusion()->set_steps(8); config.mutable_diffusion()->set_horizon(8);
    config.mutable_diffusion()->set_dim(3); config.mutable_diffusion()->set_prediction_type(proto::DIFFUSION_PRED_VELOCITY);
    *config.add_inputs() = input;
    auto* goal_input = config.add_inputs(); goal_input->set_name("observation_goal");
    goal_input->set_modality(proto::OBSERVATION_RAW); goal_input->set_required(true);
    goal_input->add_shape(1); goal_input->add_shape(3);
    Require(pipeline.LoadConfig(config,&error),error);
    const auto active_model = pipeline.policy_manager().Active();
    Require(active_model != nullptr,"the real Pipeline did not activate the actual trained token model");
    double maximum_error = 0;
    for (const auto& sample : cases) {
        Require(sample.at("images").size() == 4,"real history fixture is incomplete");
        std::vector<std::shared_ptr<const features::DefmDepthFeatures>> history;
        for (const auto& image_name : sample.at("images")) {
            auto actual = encoder->Encode(ReadImage(fixture_path.parent_path(),image_name.get<std::string>()),now,&error);
            Require(actual != nullptr,error); history.push_back(std::move(actual));
        }
        const auto calls = encoder->inference_count();
        policy::TensorFrame frame;
        Require(policy::MakeDefmHistoryTensorFrame(history,input,now,&frame,&error),error);
        Require(encoder->inference_count() == calls && frame.shape == std::vector<int64_t>({1,4,256,384}),
            "native history adapter recomputed features or changed trained axes");
        policy::ObservationBuffer buffer; buffer.SetRawTensor(frame);
        std::vector<float> tokens;
        Require(policy::EncodeRawTensor(buffer.Snapshot(),input,&tokens,&error) && tokens == frame.data,
            "existing RAW input changed the actual shared token history");
        Require(pipeline.SetDepthFeatureHistory(history,input.name(),now,&error),error);
        std::vector<float> pipeline_tokens;
        Require(policy::EncodeRawTensor(pipeline.observation_buffer().Snapshot(),input,&pipeline_tokens,&error) &&
            pipeline_tokens == tokens && encoder->inference_count() == calls,
            "active PolicyPipeline changed the shared history or recomputed its backbone");
        network::TensorMap values;
        values.emplace("observation_depth_tokens",network::Tensor::FromFloat32(std::move(tokens)));
        values.emplace("observation_goal",network::Tensor::FromFloat32(sample.at("goal").get<std::vector<float>>()));
        values.emplace("sample",network::Tensor::FromFloat32(sample.at("sample").get<std::vector<float>>()));
        values.emplace("timestep",network::Tensor::FromFloat32(sample.at("timestep").get<std::vector<float>>()));
        network::TensorMap predicted;
        const bool ran = head->Run(values,&predicted); Require(ran,head->GetLastError());
        const auto expected = sample.at("expected_v_prediction").get<std::vector<float>>();
        Require(expected.size() == 24 && predicted.size() == 1 && predicted.count("action"),"native head produced malformed v-prediction");
        const auto actual = predicted.at("action").ToFloat32(); Require(actual.size() == expected.size(),"native step output count differs");
        for (std::size_t index = 0; index < actual.size(); ++index) {
            const double difference = std::abs(actual[index]-expected[index]);
            Require(std::isfinite(actual[index]) && difference <= 5e-4+5e-4*std::abs(expected[index]),
                "actual native DeFM/RAW/head output disagrees with the held-out Torch reference");
            maximum_error = std::max(maximum_error,difference);
        }
        network::FloatTensorMap policy_inputs,policy_outputs;
        for (const auto& tensor : values) policy_inputs.emplace(tensor.first,tensor.second.ToFloat32());
        policy_inputs["observation_depth_tokens"] = pipeline_tokens;
        Require(active_model->RunInference(policy_inputs,&policy_outputs,&error),error);
        Require(policy_outputs.count("action") && policy_outputs.at("action").size() == expected.size(),
            "actual PolicyModel produced a malformed token-conditioned prediction");
        for (std::size_t index = 0; index < expected.size(); ++index)
            Require(std::isfinite(policy_outputs.at("action")[index]) &&
                std::abs(policy_outputs.at("action")[index]-expected[index]) <= 5e-4+5e-4*std::abs(expected[index]),
                "active native policy inference differs from the held-out expert/Torch reference");
        auto missing = history; missing.pop_back();
        Require(!pipeline.SetDepthFeatureHistory(missing,input.name(),now,&error) &&
            !policy::EncodeRawTensor(pipeline.observation_buffer().Snapshot(),input,&pipeline_tokens,&error),
            "a denied new history cannot leave previously valid tokens usable by the Pipeline");
        Require(pipeline.SetDepthFeatureHistory(history,input.name(),now,&error),error);
        // Expired context cannot be reused just because the trained network can
        // numerically run another forward. This test never dispatches a command.
        Require(!policy::MakeDefmHistoryTensorFrame(history,input,1010.01,&frame,&error) && frame.data.empty(),
            "native head input cannot renew its oldest original acquisition");
    }
    proto::PolicyControllerOptions zero_options;
    zero_options.set_missing_modality_policy(proto::MISSING_MODALITY_ZERO_FILL);
    policy::PolicyPipeline zero_pipeline; zero_pipeline.Configure(zero_options);
    Require(zero_pipeline.LoadConfig(config,&error),error);
    Require(!zero_pipeline.SetDepthFeatureHistory({},input.name(),now,&error),
        "zero-filled observation policy cannot authorize a missing DeFM history");
    std::cout << "PASS actual_native_defm_head cases=" << cases.size() << " max_abs_error=" << maximum_error
              << " policy_input_verified=true motion_verified=false" << std::endl;
}
}  // namespace

/** @brief Run actual native model consumption with required real artifacts. */
int main(int argc,char** argv) {
    try { Require(argc == 4,"provide actual head ONNX, frozen DeFM ONNX and native fixture JSON"); Consume(argv[1],argv[2],argv[3]); return 0; }
    catch (const std::exception& error) { std::cerr << "FAIL actual_native_defm_head: " << error.what() << std::endl; return 1; }
}
