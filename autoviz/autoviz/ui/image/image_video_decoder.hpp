/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QImage>
#include <QString>

namespace autoviz {
namespace image {

/** Decode H.264/H.265/VP9 compressed video packets to RGB frames. */
class VideoStreamDecoder {
 public:
  VideoStreamDecoder();
  ~VideoStreamDecoder();

  VideoStreamDecoder(const VideoStreamDecoder&) = delete;
  VideoStreamDecoder& operator=(const VideoStreamDecoder&) = delete;

  bool isAvailable() const;
  void reset();
  QImage decodePacket(const QString& format, const std::byte* data, std::size_t size);

 private:
  struct Impl;
  Impl* impl_ = nullptr;
};

bool isVideoMessageType(const std::string& message_type);
bool isVideoFormat(const QString& format);

}  // namespace image
}  // namespace autoviz
