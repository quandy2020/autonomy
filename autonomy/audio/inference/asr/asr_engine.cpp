/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy/audio/inference/asr/asr_engine.hpp"

#include "autonomy/common/logging.hpp"

#include "autolink/common/file.hpp"

#include <cstring>
#include <utility>

#if defined(AUTONOMY_HAS_SHERPA_ONNX)
#include "sherpa-onnx/c-api/c-api.h"
#endif

namespace autonomy {
namespace audio {
namespace {

class StubAsrEngine final : public AsrEngine {
public:
    bool available() const override { return false; }

    bool Recognize(const AsrUtterance& /*utterance*/, proto::AsrResult* /*result*/,
                   std::string* error) override {
        if (error != nullptr) {
            *error =
                "ASR backend unavailable (build with BUILD_SHERPA_ONNX and "
                "install sherpa-onnx + models).";
        }
        return false;
    }
};

#if defined(AUTONOMY_HAS_SHERPA_ONNX)

class SherpaAsrEngine final : public AsrEngine {
public:
    explicit SherpaAsrEngine(const SherpaOnnxOfflineRecognizer* recognizer,
                             std::string language)
        : recognizer_(recognizer), language_(std::move(language)) {}

    ~SherpaAsrEngine() override {
        if (recognizer_ != nullptr) {
            SherpaOnnxDestroyOfflineRecognizer(recognizer_);
            recognizer_ = nullptr;
        }
    }

    bool available() const override { return recognizer_ != nullptr; }

    bool Recognize(const AsrUtterance& utterance, proto::AsrResult* result,
                   std::string* error) override {
        if (recognizer_ == nullptr || result == nullptr) {
            if (error != nullptr) {
                *error = "Sherpa ASR recognizer is not initialized.";
            }
            return false;
        }
        if (utterance.samples.empty() || utterance.sample_rate <= 0) {
            if (error != nullptr) {
                *error = "Empty ASR utterance.";
            }
            return false;
        }

        const SherpaOnnxOfflineStream* stream =
            SherpaOnnxCreateOfflineStream(recognizer_);
        if (stream == nullptr) {
            if (error != nullptr) {
                *error = "SherpaOnnxCreateOfflineStream failed.";
            }
            return false;
        }

        SherpaOnnxAcceptWaveformOffline(stream, utterance.sample_rate,
                                        utterance.samples.data(),
                                        static_cast<int32_t>(utterance.samples.size()));
        SherpaOnnxDecodeOfflineStream(recognizer_, stream);
        const SherpaOnnxOfflineRecognizerResult* decoded =
            SherpaOnnxGetOfflineStreamResult(stream);

        bool ok = false;
        if (decoded != nullptr && decoded->text != nullptr &&
            decoded->text[0] != '\0') {
            result->set_text(decoded->text);
            result->set_is_final(true);
            result->set_language(language_);
            // Offline C-API does not always expose a scalar confidence.
            result->set_confidence(1.0F);
            ok = true;
        } else if (error != nullptr) {
            *error = "Sherpa returned empty text.";
        }

        SherpaOnnxDestroyOfflineRecognizerResult(decoded);
        SherpaOnnxDestroyOfflineStream(stream);
        return ok;
    }

private:
    const SherpaOnnxOfflineRecognizer* recognizer_ = nullptr;
    std::string language_;
};

std::unique_ptr<AsrEngine> CreateSherpaEngine(const proto::AudioOptions& options,
                                              std::string* error) {
    auto Resolve = [&](const std::string& explicit_path,
                       const std::string& filename) {
        if (!explicit_path.empty()) {
            return explicit_path;
        }
        if (options.asr_model_dir().empty()) {
            return std::string();
        }
        const std::string base = options.asr_model_dir();
        if (!base.empty() && base.back() == '/') {
            return base + filename;
        }
        return base + "/" + filename;
    };

    const std::string tokens =
        Resolve(options.asr_tokens(), "tokens.txt");
    const std::string encoder =
        Resolve(options.asr_encoder(), "encoder.onnx");
    const std::string decoder =
        Resolve(options.asr_decoder(), "decoder.onnx");
    const std::string joiner =
        Resolve(options.asr_joiner(), "joiner.onnx");

    for (const auto& path : {tokens, encoder, decoder, joiner}) {
        if (path.empty() || !autolink::common::PathExists(path)) {
            if (error != nullptr) {
                *error = "Sherpa ASR model file missing: " + path;
            }
            return nullptr;
        }
    }

    SherpaOnnxOfflineModelConfig model_config;
    memset(&model_config, 0, sizeof(model_config));
    model_config.tokens = tokens.c_str();
    model_config.num_threads = 1;
    model_config.provider = "cpu";
    model_config.debug = 0;
    model_config.model_type = "";
    model_config.modeling_unit = "cjkchar";
    model_config.bpe_vocab = "";
    model_config.telespeech_ctc = "";
    model_config.transducer.encoder = encoder.c_str();
    model_config.transducer.decoder = decoder.c_str();
    model_config.transducer.joiner = joiner.c_str();

    SherpaOnnxOfflineRecognizerConfig config;
    memset(&config, 0, sizeof(config));
    config.feat_config.sampling_rate = 16000;
    config.feat_config.feature_dim = 80;
    config.model_config = model_config;
    config.decoding_method = "greedy_search";
    config.max_active_paths = 4;

    const SherpaOnnxOfflineRecognizer* recognizer =
        SherpaOnnxCreateOfflineRecognizer(&config);
    if (recognizer == nullptr) {
        if (error != nullptr) {
            *error = "SherpaOnnxCreateOfflineRecognizer failed.";
        }
        return nullptr;
    }

    std::string language = options.asr_language().empty()
                               ? "zh"
                               : options.asr_language();
    return std::make_unique<SherpaAsrEngine>(recognizer, std::move(language));
}

#endif  // AUTONOMY_HAS_SHERPA_ONNX

}  // namespace

std::unique_ptr<AsrEngine> CreateAsrEngine(const proto::AudioOptions& options,
                                           std::string* error) {
#if defined(AUTONOMY_HAS_SHERPA_ONNX)
    auto engine = CreateSherpaEngine(options, error);
    if (engine != nullptr) {
        AINFO << "ASR engine: sherpa-onnx offline transducer.";
        return engine;
    }
    AWARN << "Failed to create Sherpa ASR engine"
          << (error != nullptr && !error->empty() ? (": " + *error) : ".");
#else
    (void)options;
    if (error != nullptr) {
        *error =
            "Built without sherpa-onnx (enable BUILD_SHERPA_ONNX and install "
            "the library + Zipformer model).";
    }
#endif
    return std::make_unique<StubAsrEngine>();
}

}  // namespace audio
}  // namespace autonomy
