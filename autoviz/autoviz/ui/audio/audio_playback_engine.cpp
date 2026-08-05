/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/audio/audio_playback_engine.hpp"

#include <algorithm>

#ifdef AUTOVIZ_USE_QT_MULTIMEDIA
#include <QAudioDevice>
#include <QAudioFormat>
#include <QAudioSink>
#include <QIODevice>
#include <QMediaDevices>
#endif

namespace autoviz {
namespace audio_panel {

AudioPlaybackEngine::AudioPlaybackEngine(QObject* parent) : QObject(parent) {}

AudioPlaybackEngine::~AudioPlaybackEngine() { stop(); }

bool AudioPlaybackEngine::playbackAvailable() const {
#ifdef AUTOVIZ_USE_QT_MULTIMEDIA
  return true;
#else
  return false;
#endif
}

bool AudioPlaybackEngine::configure(int sample_rate, int channels) {
  if (sample_rate <= 0 || channels <= 0) {
    return false;
  }
  stop();
  sample_rate_ = sample_rate;
  channels_ = channels;
  return true;
}

void AudioPlaybackEngine::setVolume(double volume) {
  volume_ = std::clamp(volume, 0.0, 1.0);
#ifdef AUTOVIZ_USE_QT_MULTIMEDIA
  std::lock_guard<std::mutex> lock(mutex_);
  if (sink_ != nullptr) {
    sink_->setVolume(muted_ ? 0.0f : static_cast<float>(volume_));
  }
#endif
}

void AudioPlaybackEngine::setMuted(bool muted) {
  muted_ = muted;
  setVolume(volume_);
}

void AudioPlaybackEngine::ensureStarted() {
#ifdef AUTOVIZ_USE_QT_MULTIMEDIA
  if (sink_ != nullptr || sample_rate_ <= 0 || channels_ <= 0) {
    return;
  }
  QAudioFormat format;
  format.setSampleRate(sample_rate_);
  format.setChannelCount(channels_);
  format.setSampleFormat(QAudioFormat::Int16);
  const QAudioDevice device = QMediaDevices::defaultAudioOutput();
  if (device.isNull() || !device.isFormatSupported(format)) {
    return;
  }
  sink_ = std::make_unique<QAudioSink>(device, format, this);
  sink_->setVolume(muted_ ? 0.0f : static_cast<float>(volume_));
  io_device_ = sink_->start();
  if (io_device_ == nullptr) {
    sink_.reset();
  }
#else
  (void)sample_rate_;
  (void)channels_;
#endif
}

void AudioPlaybackEngine::enqueueSamples(const std::int16_t* samples,
                                         std::size_t sample_count) {
  if (samples == nullptr || sample_count == 0) {
    return;
  }
#ifdef AUTOVIZ_USE_QT_MULTIMEDIA
  std::lock_guard<std::mutex> lock(mutex_);
  ensureStarted();
  if (sink_ == nullptr || io_device_ == nullptr ||
      sink_->state() == QAudio::StoppedState) {
    return;
  }
  const qint64 bytes_to_write =
      static_cast<qint64>(sample_count * sizeof(std::int16_t));
  const qint64 written =
      io_device_->write(reinterpret_cast<const char*>(samples), bytes_to_write);
  if (written < bytes_to_write) {
    // Drop overflow samples to keep latency bounded during live playback.
  }
#else
  (void)samples;
  (void)sample_count;
#endif
}

void AudioPlaybackEngine::stop() {
#ifdef AUTOVIZ_USE_QT_MULTIMEDIA
  std::lock_guard<std::mutex> lock(mutex_);
  io_device_ = nullptr;
  if (sink_ != nullptr) {
    sink_->stop();
    sink_.reset();
  }
#endif
}

bool AudioPlaybackEngine::isActive() const {
#ifdef AUTOVIZ_USE_QT_MULTIMEDIA
  std::lock_guard<std::mutex> lock(mutex_);
  return sink_ != nullptr && sink_->state() == QAudio::ActiveState;
#else
  return false;
#endif
}

}  // namespace audio_panel
}  // namespace autoviz
