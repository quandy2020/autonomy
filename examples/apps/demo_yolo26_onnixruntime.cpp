// Copyright 2025 The Openbot Authors. Apache 2.0.
// Usage: demo_yolo26_onnixruntime <model.onnx> <image> [output.jpg]

#include "autonomy/common/network/network.hpp"

#include <glog/logging.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>
#include <vector>

namespace network = autonomy::common::network;

void DrawBoxes(cv::Mat* image, const std::vector<network::Detection>& boxes) {
    for (const network::Detection& box : boxes) {
        const int x1 = static_cast<int>(std::min(box.x1, box.x2));
        const int y1 = static_cast<int>(std::min(box.y1, box.y2));
        const int x2 = static_cast<int>(std::max(box.x1, box.x2));
        const int y2 = static_cast<int>(std::max(box.y1, box.y2));
        cv::rectangle(*image, {x1, y1, std::max(x2, x1 + 2), std::max(y2, y1 + 2)}, {0, 255, 0},
                      2);
        char label[48];
        std::snprintf(label, sizeof(label), "%d %.2f", box.class_id, box.confidence);
        cv::putText(*image, label, {x1, std::max(0, y1 - 2)}, cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    {0, 255, 0}, 1);
    }
}

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    FLAGS_logtostderr = true;

    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <model.onnx> <image> [output.jpg]\n";
        return 1;
    }

    std::string error;
    auto engine = network::Engine::CreateEngine(argv[1], "onnx", &error);
    if (!engine) {
        std::cerr << "Load failed: " << error << '\n';
        return 1;
    }

    const auto& outputs = engine->GetOutputInfos();
    if (outputs.empty()) {
        std::cerr << "Invalid model I/O.\n";
        return 1;
    }

    cv::Mat image = cv::imread(argv[2]);
    if (image.empty()) {
        std::cerr << "Failed to read image.\n";
        return 1;
    }

    const network::PreprocessOptions preprocess =
        network::Make<network::ResizePolicy::kLetterbox>(640, 640);

    network::RunResult result;
    if (!network::RunPipeline(engine.get(), image, preprocess, &result, &error)) {
        std::cerr << "RunPipeline failed: " << error << '\n';
        return 1;
    }

    const auto output_it = result.outputs.find(outputs[0].name);
    if (output_it == result.outputs.end()) {
        std::cerr << "Missing output.\n";
        return 1;
    }

    network::BoxOptions opts;
    opts.num_classes = 80;
    opts.conf_threshold = 0.55f;
    opts.nms_iou = 0.45f;

    std::vector<network::Detection> boxes;
    if (!network::Decode(output_it->second, outputs[0], result.meta, opts, &boxes, &error)) {
        std::cerr << "Decode failed: " << error << '\n';
        return 1;
    }

    DrawBoxes(&image, boxes);

    const std::string path = argc >= 4 ? argv[3] : "det_out.jpg";
    if (!cv::imwrite(path, image)) {
        std::cerr << "Failed to write " << path << '\n';
        return 1;
    }
    std::cout << "Saved " << boxes.size() << " box(es) to " << path << '\n';
    return 0;
}
