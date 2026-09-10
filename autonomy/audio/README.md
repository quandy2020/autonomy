# Audio — mic-array direction / motion + optional ASR

```text
/mic/cabin/audio (AudioData)
        │
        ▼
   AudioComponent
        ├─ direction (GCC-PHAT) ─┐
        ├─ moving (FFT trend)  ─┼─► /audio/detection
        └─ ASR (sherpa-onnx)   ─┬─► /audio/asr          (AsrResult)
                                └─► /speech_recognition (std_msgs/String)
```

| Topic | Type | Role |
| --- | --- | --- |
| `/mic/cabin/audio` | `autonomy.audio.proto.AudioData` | Mic-array (+ optional ASR channel) |
| `/audio/detection` | `autonomy.audio.proto.AudioDetection` | Direction / motion |
| `/audio/asr` | `autonomy.audio.proto.AsrResult` | Recognized text |
| `/speech_recognition` | `std_msgs/String` | Text-only compatibility |

### Message layering

| Layer | Package | Messages |
| --- | --- | --- |
| Generic PCM | `automsgs.msgs.audio_msgs` | `AudioInfo`, `AudioDataStamped` |
| Array + ASR | `autonomy.audio.proto` | `AudioData`, `AudioDetection`, `AsrResult`, `AudioOptions` |

## Enable Sherpa-ONNX ASR

```bash
# 1) Install sherpa-onnx (headers + libsherpa-onnx-c-api)
# 2) Download a Zipformer offline transducer (encoder/decoder/joiner/tokens)
cmake -DBUILD_SHERPA_ONNX=ON ...
```

Then in `conf/audio.pb.txt`:

```text
enable_asr: true
asr_model_dir: "/path/to/sherpa-onnx-zipformer-..."
# or set asr_encoder / asr_decoder / asr_joiner / asr_tokens explicitly
```

Without the library, `enable_asr` logs a warning and direction/moving still run.

## Layout

```text
audio/
  inference/asr/asr_engine.*   # stub + optional Sherpa
  proto/asr.proto              # AsrResult
  ...
```

Downstream NLU → `task` goals is **not** included; subscribe to `/audio/asr`
or `/speech_recognition` from a separate intent node.
