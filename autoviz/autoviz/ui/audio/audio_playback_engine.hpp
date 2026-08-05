/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QObject>

#include <cstdint>
#include <memory>
#include <mutex>

#ifdef AUTOVIZ_USE_QT_MULTIMEDIA
class QAudioSink;
class QIODevice;
#endif

namespace autoviz {
namespace audio_panel {

/** Streams pcm-s16 samples to the default audio output device. */
class AudioPlaybackEngine : public QObject {
  Q_OBJECT

 public:
  explicit AudioPlaybackEngine(QObject* parent = nullptr);
  ~AudioPlaybackEngine() override;

  bool playbackAvailable() const;
  bool configure(int sample_rate, int channels);
  void setVolume(double volume);
  void setMuted(bool muted);
  void enqueueSamples(const std::int16_t* samples, std::size_t sample_count);
  void stop();
  bool isActive() const;

 private:
  void ensureStarted();

#ifdef AUTOVIZ_USE_QT_MULTIMEDIA
  std::unique_ptr<QAudioSink> sink_;
  QIODevice* io_device_ = nullptr;
#endif
  int sample_rate_ = 0;
  int channels_ = 0;
  double volume_ = 0.8;
  bool muted_ = false;
  mutable std::mutex mutex_;
};

}  // namespace audio_panel
}  // namespace autoviz
