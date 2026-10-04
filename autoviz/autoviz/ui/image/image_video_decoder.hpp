/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file image_video_decoder.hpp
 * @brief Decode compressed video packets (H.264 / H.265 / VP9) to RGB frames.
 *
 * Used by @ref ImagePanel when the main or overlay channel carries a video
 * message type rather than a single still image.
 *
 * @see ImagePanel
 * @see isVideoMessageType()
 */

#pragma once

#include <QImage>
#include <QString>

namespace autoviz {
namespace image {

/**
 * @class VideoStreamDecoder
 * @brief Stateful bitstream decoder that reconstructs RGB @c QImage frames.
 *
 * Non-copyable; holds opaque codec state in @c Impl. Call @ref reset() when
 * switching streams or seeking.
 *
 * @note @ref isAvailable() reports whether the build linked a usable decoder.
 */
class VideoStreamDecoder {
 public:
  /**
   * @brief Constructs an empty decoder (lazy codec init on first packet).
   */
  VideoStreamDecoder();

  /**
   * @brief Releases codec resources held by @c impl_.
   */
  ~VideoStreamDecoder();

  VideoStreamDecoder(const VideoStreamDecoder&) = delete;
  VideoStreamDecoder& operator=(const VideoStreamDecoder&) = delete;

  /**
   * @brief Whether a video decoder backend is linked and usable.
   *
   * @return @c true when @ref decodePacket() can succeed for supported formats.
   */
  bool isAvailable() const;

  /**
   * @brief Clears codec state (e.g. after seek or channel change).
   */
  void reset();

  /**
   * @brief Feeds one compressed packet and returns a decoded RGB frame if ready.
   *
   * @param format Codec / container hint (e.g. @c h264, @c h265, @c vp9).
   * @param data Packet bytes.
   * @param size Packet length in bytes.
   * @return Decoded frame, or a null @c QImage if no picture is ready / on error.
   */
  QImage decodePacket(const QString& format, const std::byte* data, std::size_t size);

 private:
  /** Opaque codec / FFmpeg (or similar) implementation. */
  struct Impl;

  /** Owned implementation pointer (@c nullptr until first use / if unavailable). */
  Impl* impl_ = nullptr;
};

/**
 * @brief Returns @c true when @p message_type is a compressed video schema.
 *
 * @param message_type Fully-qualified message type name.
 * @return Whether @ref VideoStreamDecoder should handle the payload.
 * @see isVideoFormat()
 */
bool isVideoMessageType(const std::string& message_type);

/**
 * @brief Returns @c true when @p format names a supported video codec.
 *
 * @param format Short format string from the message (e.g. @c h264).
 * @return Whether the decoder recognizes the format.
 */
bool isVideoFormat(const QString& format);

}  // namespace image
}  // namespace autoviz
