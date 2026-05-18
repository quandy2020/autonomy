// Copyright 2025 The Openbot Authors. Apache 2.0.
//
// Monocular depth demo (ONNX). Tune PreprocessOptions for your export.
//
// Usage: demo_depth_angthing_v3 <model.onnx> <image> [vis.png]

#include "autonomy/common/network/network.hpp"
#include "autonomy/common/network/detail/preprocess/dims.hpp"
#include "autonomy/common/network/detail/preprocess/policy.hpp"

#include <glog/logging.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>

namespace network = autonomy::common::network;

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <model.onnx> <image> [vis.png]\n";
        return 1;
    }
    google::InitGoogleLogging(argv[0]);

    cv::Mat image = cv::imread(argv[2]);
    if (image.empty()) {
        std::cerr << "Failed to read image: " << argv[2] << '\n';
        return 1;
    }

    std::string error;
    auto engine = network::Engine::CreateEngine(argv[1], "onnx", &error);
    if (!engine) {
        std::cerr << "Load failed: " << error << '\n';
        return 1;
    }

    const auto& inputs = engine->GetInputInfos();
    const auto& outputs = engine->GetOutputInfos();
    if (inputs.empty() || outputs.empty()) {
        std::cerr << "Invalid model I/O.\n";
        return 1;
    }

    int height = 518;
    int width = 518;
    network::GetSpatialSize(inputs[0], 518, &height, &width);

    const network::PreprocessOptions preprocess =
        network::MakeBound(504, 14, height, width);

    network::RunResult result;
    if (!network::RunPipeline(engine.get(), image, preprocess, &result, &error)) {
        std::cerr << "RunPipeline failed: " << error << '\n';
        return 1;
    }

    std::string name;
    const std::vector<float>* data = nullptr;
    if (!network::Find(result.outputs, outputs, &name, &data, "depth", &error)) {
        std::cerr << "Find failed: " << error << '\n';
        return 1;
    }

    const network::ModelTensorInfo* meta = &outputs[0];
    for (const auto& info : outputs) {
        if (info.name == name) {
            meta = &info;
            break;
        }
    }

    cv::Mat depth;
    if (!network::ToMat(*data, *meta, result.meta, &depth, &error)) {
        std::cerr << "ToMat failed: " << error << '\n';
        return 1;
    }

    cv::Mat vis;
    if (!network::Colorize(depth, &vis)) {
        std::cerr << "Colorize failed.\n";
        return 1;
    }

    const std::string path = argc >= 4 ? argv[3] : "depth_vis.png";
    if (!cv::imwrite(path, vis)) {
        std::cerr << "Failed to write " << path << '\n';
        return 1;
    }

    std::cout << "Output: " << name << '\n';
    std::cout << "Saved " << path << " (" << depth.cols << "x" << depth.rows << ")\n";
    return 0;
}
