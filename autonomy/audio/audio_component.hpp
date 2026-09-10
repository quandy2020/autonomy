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

#ifndef AUTONOMY_AUDIO_AUDIO_COMPONENT_HPP_
#define AUTONOMY_AUDIO_AUDIO_COMPONENT_HPP_

#include "autonomy/audio/common/audio_info.hpp"
#include "autonomy/audio/inference/asr/asr_engine.hpp"
#include "autonomy/audio/inference/direction_detection.hpp"
#include "autonomy/audio/inference/moving_detection.hpp"
#include "autonomy/audio/proto/asr.pb.h"
#include "autonomy/audio/proto/audio.pb.h"
#include "autonomy/audio/proto/audio_detection.pb.h"
#include "autonomy/audio/proto/microphone.pb.h"

#include "autolink/component/component.hpp"

#include <automsgs/msgs/std_msgs/string.pb.h>

#include <memory>

namespace autonomy {
namespace audio {

class AudioComponent final
    : public autolink::Component<proto::AudioData> {
public:
    ~AudioComponent() override;

    bool Init() override;

    bool Proc(const std::shared_ptr<proto::AudioData>& audio_data) override;

protected:
    void Clear() override;

private:
    proto::AudioOptions options_;
    AudioInfo audio_info_;
    DirectionDetection direction_detection_;
    MovingDetection moving_detection_;
    std::unique_ptr<AsrEngine> asr_engine_;

    std::shared_ptr<autolink::Writer<proto::AudioDetection>> audio_writer_;
    std::shared_ptr<autolink::Writer<proto::AsrResult>> asr_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::std_msgs::String>>
        speech_writer_;
};

}  // namespace audio
}  // namespace autonomy

#endif  // AUTONOMY_AUDIO_AUDIO_COMPONENT_HPP_
