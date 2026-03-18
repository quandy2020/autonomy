// Copyright 2025 The Openbot Authors. Apache 2.0.
// Usage: demo_yolo26_onnixruntime <model.onnx> <image> [output.jpg]

#include "autonomy/common/network/onnx_engine.hpp"
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include <unordered_map>
#include <vector>

namespace {

constexpr float kConfThreshold = 0.55f;
constexpr float kNmsThreshold = 0.45f;
constexpr float kMinBoxSize = 2.0f;
constexpr int kInputSize = 640;
constexpr int kNumClasses = 80;

struct Detection {
  float x1{}, y1{}, x2{}, y2{};
  float confidence{};
  int class_id{};
};

inline float Sigmoid(float x) {
  return x >= 0.f
      ? 1.f / (1.f + std::exp(-x))
      : std::exp(x) / (1.f + std::exp(x));
}

float IoU(const Detection& a, const Detection& b) {
  const float inter_x1 = std::max(a.x1, b.x1);
  const float inter_y1 = std::max(a.y1, b.y1);
  const float inter_x2 = std::min(a.x2, b.x2);
  const float inter_y2 = std::min(a.y2, b.y2);
  const float inter_w = std::max(0.f, inter_x2 - inter_x1);
  const float inter_h = std::max(0.f, inter_y2 - inter_y1);
  const float inter_area = inter_w * inter_h;
  const float area_a = (a.x2 - a.x1) * (a.y2 - a.y1);
  const float area_b = (b.x2 - b.x1) * (b.y2 - b.y1);
  const float union_area = area_a + area_b - inter_area;
  return (union_area > 1e-6f) ? (inter_area / union_area) : 0.f;
}

void NMS(std::vector<Detection>* detections) {
  std::vector<Detection>& d = *detections;
  std::sort(d.begin(), d.end(), [](const Detection& a, const Detection& b) {
    return a.confidence > b.confidence;
  });
  for (size_t i = 0; i < d.size(); ++i) {
    if (d[i].confidence < 0.f) continue;
    for (size_t j = i + 1; j < d.size(); ++j) {
      if (d[j].confidence < 0.f) continue;
      if (d[i].class_id != d[j].class_id) continue;
      if (IoU(d[i], d[j]) > kNmsThreshold) {
        d[j].confidence = -1.f;
      }
    }
  }
  d.erase(
      std::remove_if(d.begin(), d.end(),
                     [](const Detection& x) { return x.confidence < 0.f; }),
      d.end());
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <model.onnx> <image> [output.jpg]\n";
    return 1;
  }
  google::InitGoogleLogging(argv[0]);

  // Load model
  autonomy::common::network::OnnxEngine engine;
  if (!engine.LoadFromFile(argv[1])) {
    std::cerr << "Load failed: " << engine.GetLastError() << "\n";
    return 1;
  }
  auto input_names = engine.GetInputNames();
  auto output_names = engine.GetOutputNames();
  if (input_names.empty() || output_names.empty()) {
    std::cerr << "Invalid model (no input/output).\n";
    return 1;
  }
  auto input_info = engine.GetInputInfo(0);
  if (!input_info.has_value() || input_info->shape.size() != 4) {
    std::cerr << "Expected input shape [N,C,H,W].\n";
    return 1;
  }
  const int input_h = input_info->shape[2] > 0
                          ? static_cast<int>(input_info->shape[2])
                          : kInputSize;
  const int input_w = input_info->shape[3] > 0
                          ? static_cast<int>(input_info->shape[3])
                          : kInputSize;

  // Load image and letterbox
  cv::Mat image = cv::imread(argv[2]);
  if (image.empty()) {
    std::cerr << "Failed to read image.\n";
    return 1;
  }
  const int orig_h = image.rows;
  const int orig_w = image.cols;
  const double gain = std::min(static_cast<double>(input_h) / orig_h,
                               static_cast<double>(input_w) / orig_w);
  const int new_w = static_cast<int>(std::round(orig_w * gain));
  const int new_h = static_cast<int>(std::round(orig_h * gain));
  const int pad_left = static_cast<int>(std::round((input_w - new_w) / 2.0));
  const int pad_top = static_cast<int>(std::round((input_h - new_h) / 2.0));

  cv::Mat letterbox(input_h, input_w, image.type());
  letterbox.setTo(cv::Scalar(114, 114, 114));
  cv::Mat resized;
  cv::resize(image, resized, cv::Size(new_w, new_h));
  resized.copyTo(letterbox(cv::Rect(pad_left, pad_top, new_w, new_h)));

  // HWC -> NCHW, [0,255] -> [0,1]
  cv::Mat rgb;
  cv::cvtColor(letterbox, rgb, cv::COLOR_BGR2RGB);
  std::vector<float> input_data(3 * input_h * input_w);
  for (int c = 0; c < 3; ++c) {
    for (int y = 0; y < input_h; ++y) {
      for (int x = 0; x < input_w; ++x) {
        input_data[c * input_h * input_w + y * input_w + x] =
            static_cast<float>(rgb.at<cv::Vec3b>(y, x)[c]) / 255.f;
      }
    }
  }

  // Infer
  std::unordered_map<std::string, std::vector<float>> inputs, outputs;
  inputs[input_names[0]] = std::move(input_data);
  if (!engine.Infer(inputs, &outputs)) {
    std::cerr << "Inference failed: " << engine.GetLastError() << "\n";
    return 1;
  }

  // Parse output: layout (1,84,N) or (1,N,84)
  const std::vector<float>& output = outputs[output_names[0]];
  const int stride = 4 + kNumClasses;
  if (output.size() % stride != 0) {
    std::cerr << "Unexpected output size.\n";
    return 1;
  }
  int num_proposals = static_cast<int>(output.size() / stride);
  bool row_major = false;
  auto out_info = engine.GetOutputInfo(0);
  if (out_info.has_value() && out_info->shape.size() == 3) {
    if (out_info->shape[2] == stride) row_major = true;
    if (out_info->shape[1] == stride && out_info->shape[2] > 0) {
      num_proposals = static_cast<int>(out_info->shape[2]);
    } else if (out_info->shape[2] == stride && out_info->shape[1] > 0) {
      num_proposals = static_cast<int>(out_info->shape[1]);
    }
  }

  auto get_output = [&](int n, int ch) {
    return row_major ? output[n * stride + ch] : output[ch * num_proposals + n];
  };

  // Detect format from best-scoring proposal
  int best_proposal = 0;
  float best_score = 0.f;
  for (int n = 0; n < num_proposals; ++n) {
    float max_s = 0.f;
    for (int c = 0; c < kNumClasses; ++c) {
      max_s = std::max(max_s, Sigmoid(get_output(n, 4 + c)));
    }
    if (max_s > best_score) {
      best_score = max_s;
      best_proposal = n;
    }
  }
  const float b0 = get_output(best_proposal, 0);
  const float b1 = get_output(best_proposal, 1);
  const float b2 = get_output(best_proposal, 2);
  const float b3 = get_output(best_proposal, 3);
  const bool is_normalized = std::max({b0, b1, b2, b3}) <= 1.5f;
  const bool is_xyxy = (b2 > b0 + 2.f && b3 > b1 + 2.f);

  // Decode boxes and collect detections
  std::vector<Detection> detections;
  detections.reserve(256);
  for (int n = 0; n < num_proposals; ++n) {
    double cx = get_output(n, 0);
    double cy = get_output(n, 1);
    double w = get_output(n, 2);
    double h = get_output(n, 3);
    if (is_normalized) {
      cx *= input_w;
      cy *= input_h;
      w *= input_w;
      h *= input_h;
    }

    double x1, y1, x2, y2;
    if (is_xyxy) {
      x1 = (cx - pad_left) / gain;
      y1 = (cy - pad_top) / gain;
      x2 = (w - pad_left) / gain;
      y2 = (h - pad_top) / gain;
    } else {
      const double center_x = (cx - pad_left) / gain;
      const double center_y = (cy - pad_top) / gain;
      const double half_w = (w * 0.5) / gain;
      const double half_h = (h * 0.5) / gain;
      x1 = center_x - half_w;
      y1 = center_y - half_h;
      x2 = center_x + half_w;
      y2 = center_y + half_h;
    }

    int class_id = 0;
    float confidence = Sigmoid(get_output(n, 4));
    for (int c = 1; c < kNumClasses; ++c) {
      float s = Sigmoid(get_output(n, 4 + c));
      if (s > confidence) {
        confidence = s;
        class_id = c;
      }
    }
    if (confidence < kConfThreshold) continue;

    const double x1_f = std::clamp(std::min(x1, x2), 0.0, static_cast<double>(orig_w));
    const double x2_f = std::clamp(std::max(x1, x2), 0.0, static_cast<double>(orig_w));
    const double y1_f = std::clamp(std::min(y1, y2), 0.0, static_cast<double>(orig_h));
    const double y2_f = std::clamp(std::max(y1, y2), 0.0, static_cast<double>(orig_h));
    if (x2_f - x1_f < kMinBoxSize || y2_f - y1_f < kMinBoxSize) continue;

    detections.push_back({static_cast<float>(x1_f), static_cast<float>(y1_f),
                          static_cast<float>(x2_f), static_cast<float>(y2_f),
                          confidence, class_id});
  }
  NMS(&detections);

  // Draw on original image
  for (const Detection& d : detections) {
    float x1 = std::min(d.x1, d.x2);
    float y1 = std::min(d.y1, d.y2);
    float x2 = std::max(d.x1, d.x2);
    float y2 = std::max(d.y1, d.y2);
    if (x2 - x1 < 1.f) x2 = x1 + 2.f;
    if (y2 - y1 < 1.f) y2 = y1 + 2.f;
    cv::rectangle(image, cv::Point2f(x1, y1), cv::Point2f(x2, y2),
                  cv::Scalar(0, 255, 0), 2);
    char label[64];
    std::snprintf(label, sizeof(label), "%d %.2f", d.class_id, d.confidence);
    cv::putText(image, label, cv::Point(x1, y1 - 2), cv::FONT_HERSHEY_SIMPLEX,
                0.5, cv::Scalar(0, 255, 0), 1);
  }

  const std::string output_path =
      (argc >= 4) ? argv[3] : "yolo8_out.jpg";
  if (!cv::imwrite(output_path, image)) {
    std::cerr << "Failed to write " << output_path << "\n";
    return 1;
  }
  std::cout << "Saved " << detections.size() << " detection(s) to "
            << output_path << "\n";
  return 0;
}
