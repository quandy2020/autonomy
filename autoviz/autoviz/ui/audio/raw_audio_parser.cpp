/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/audio/raw_audio_parser.hpp"

#include <algorithm>
#include <cstring>

#include <QByteArray>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>

namespace autoviz {
namespace audio_panel {
namespace {

constexpr char kPcmS16Format[] = "pcm-s16";

std::int64_t TimestampNsFromJson(const QJsonObject& object) {
  const QJsonObject timestamp = object.value(QStringLiteral("timestamp")).toObject();
  if (timestamp.isEmpty()) {
    return 0;
  }
  const qint64 sec = timestamp.value(QStringLiteral("sec")).toVariant().toLongLong();
  const qint64 nsec = timestamp.value(QStringLiteral("nsec")).toVariant().toLongLong();
  return sec * 1000000000LL + nsec;
}

QByteArray DecodeDataField(const QJsonValue& data_value) {
  if (data_value.isString()) {
    return QByteArray::fromBase64(data_value.toString().toUtf8());
  }
  if (!data_value.isArray()) {
    return {};
  }
  const QJsonArray array = data_value.toArray();
  QByteArray bytes;
  bytes.reserve(array.size());
  for (const QJsonValue& byte_value : array) {
    bytes.append(static_cast<char>(byte_value.toInt() & 0xFF));
  }
  return bytes;
}

}  // namespace

bool IsRawAudioMessageType(const std::string& message_type) {
  return message_type == "foxglove.RawAudio" ||
         message_type.find("RawAudio") != std::string::npos;
}

bool ParseRawAudioPayload(const std::string& message_type,
                          const std::string& payload, ParsedRawAudio* audio,
                          std::string* error_message) {
  if (audio == nullptr) {
    return false;
  }
  audio->pcm_samples.clear();
  audio->timestamp_ns = 0;
  audio->sample_rate = 0;
  audio->number_of_channels = 0;
  audio->format.clear();

  if (payload.empty()) {
    if (error_message != nullptr) {
      *error_message = "Empty payload";
    }
    return false;
  }

  if (payload.front() != '{') {
    if (error_message != nullptr) {
      *error_message = "Expected foxglove.RawAudio JSON payload";
    }
    return false;
  }

  const QJsonDocument document =
      QJsonDocument::fromJson(QByteArray::fromStdString(payload));
  if (!document.isObject()) {
    if (error_message != nullptr) {
      *error_message = "Invalid JSON object";
    }
    return false;
  }

  const QJsonObject root = document.object();
  const QString format = root.value(QStringLiteral("format")).toString();
  if (!format.isEmpty() && format != QLatin1String(kPcmS16Format)) {
    if (error_message != nullptr) {
      *error_message = "Unsupported audio format (only pcm-s16 is supported)";
    }
    return false;
  }

  const int sample_rate = root.value(QStringLiteral("sample_rate")).toInt();
  const int channels = root.value(QStringLiteral("number_of_channels")).toInt();
  if (sample_rate <= 0 || channels <= 0) {
    if (error_message != nullptr) {
      *error_message = "Invalid sample_rate or number_of_channels";
    }
    return false;
  }

  const QByteArray bytes = DecodeDataField(root.value(QStringLiteral("data")));
  if (bytes.isEmpty() || (bytes.size() % 2) != 0) {
    if (error_message != nullptr) {
      *error_message = "Missing or invalid PCM data";
    }
    return false;
  }

  const std::size_t sample_count = static_cast<std::size_t>(bytes.size()) / 2;
  if ((sample_count % static_cast<std::size_t>(channels)) != 0) {
    if (error_message != nullptr) {
      *error_message = "PCM byte length does not match channel count";
    }
    return false;
  }

  audio->timestamp_ns = TimestampNsFromJson(root);
  audio->sample_rate = sample_rate;
  audio->number_of_channels = channels;
  audio->format = format.isEmpty() ? kPcmS16Format : format.toStdString();
  audio->pcm_samples.resize(sample_count);
  std::memcpy(audio->pcm_samples.data(), bytes.constData(), bytes.size());
  return true;
}

}  // namespace audio_panel
}  // namespace autoviz
