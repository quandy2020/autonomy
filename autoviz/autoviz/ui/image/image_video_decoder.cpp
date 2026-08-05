/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/image/image_video_decoder.hpp"

#include <QByteArray>

namespace autoviz {
namespace image {

namespace {

QString NormalizeFormat(QString format) {
  format = format.trimmed().toLower();
  if (format == QLatin1String("libx264") || format == QLatin1String("avc1")) {
    return QStringLiteral("h264");
  }
  if (format == QLatin1String("hevc") || format == QLatin1String("libx265")) {
    return QStringLiteral("h265");
  }
  return format;
}

}  // namespace

bool isVideoMessageType(const std::string& message_type) {
  return message_type == "foxglove.CompressedVideo" ||
         message_type.find("CompressedVideo") != std::string::npos;
}

bool isVideoFormat(const QString& format) {
  const QString normalized = NormalizeFormat(format);
  return normalized == QLatin1String("h264") ||
         normalized == QLatin1String("h265") ||
         normalized == QLatin1String("hevc") ||
         normalized == QLatin1String("vp9") ||
         normalized == QLatin1String("av1");
}

#ifdef AUTOVIZ_USE_FFMPEG

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

struct VideoStreamDecoder::Impl {
  AVCodecContext* codec = nullptr;
  AVFrame* frame = nullptr;
  AVFrame* rgb_frame = nullptr;
  AVPacket* packet = nullptr;
  SwsContext* sws = nullptr;
  QString active_format;
};

VideoStreamDecoder::VideoStreamDecoder() : impl_(new Impl()) {
  impl_->frame = av_frame_alloc();
  impl_->rgb_frame = av_frame_alloc();
  impl_->packet = av_packet_alloc();
}

VideoStreamDecoder::~VideoStreamDecoder() {
  reset();
  if (impl_->packet != nullptr) {
    av_packet_free(&impl_->packet);
  }
  if (impl_->frame != nullptr) {
    av_frame_free(&impl_->frame);
  }
  if (impl_->rgb_frame != nullptr) {
    av_frame_free(&impl_->rgb_frame);
  }
  delete impl_;
}

bool VideoStreamDecoder::isAvailable() const { return true; }

void VideoStreamDecoder::reset() {
  if (impl_->sws != nullptr) {
    sws_freeContext(impl_->sws);
    impl_->sws = nullptr;
  }
  if (impl_->codec != nullptr) {
    avcodec_free_context(&impl_->codec);
    impl_->codec = nullptr;
  }
  impl_->active_format.clear();
}

QImage VideoStreamDecoder::decodePacket(const QString& format, const std::byte* data,
                                        std::size_t size) {
  if (data == nullptr || size == 0) {
    return {};
  }
  const QString normalized = NormalizeFormat(format);
  if (!isVideoFormat(normalized)) {
    return {};
  }
  if (impl_->codec == nullptr || impl_->active_format != normalized) {
    reset();
    const AVCodec* codec = nullptr;
    if (normalized == QLatin1String("h264")) {
      codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    } else if (normalized == QLatin1String("h265") ||
               normalized == QLatin1String("hevc")) {
      codec = avcodec_find_decoder(AV_CODEC_ID_HEVC);
    } else if (normalized == QLatin1String("vp9")) {
      codec = avcodec_find_decoder(AV_CODEC_ID_VP9);
    } else if (normalized == QLatin1String("av1")) {
      codec = avcodec_find_decoder(AV_CODEC_ID_AV1);
    }
    if (codec == nullptr) {
      return {};
    }
    impl_->codec = avcodec_alloc_context3(codec);
    if (impl_->codec == nullptr ||
        avcodec_open2(impl_->codec, codec, nullptr) < 0) {
      reset();
      return {};
    }
    impl_->active_format = normalized;
  }

  av_packet_unref(impl_->packet);
  impl_->packet->data = const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(data));
  impl_->packet->size = static_cast<int>(size);
  if (avcodec_send_packet(impl_->codec, impl_->packet) < 0) {
    return {};
  }
  if (avcodec_receive_frame(impl_->codec, impl_->frame) < 0) {
    return {};
  }

  impl_->sws = sws_getCachedContext(
      impl_->sws, impl_->frame->width, impl_->frame->height,
      static_cast<AVPixelFormat>(impl_->frame->format), impl_->frame->width,
      impl_->frame->height, AV_PIX_FMT_RGB24, SWS_BILINEAR, nullptr, nullptr,
      nullptr);
  if (impl_->sws == nullptr) {
    return {};
  }

  const int buffer_size =
      av_image_get_buffer_size(AV_PIX_FMT_RGB24, impl_->frame->width,
                               impl_->frame->height, 1);
  QByteArray buffer(buffer_size, Qt::Uninitialized);
  av_image_fill_arrays(impl_->rgb_frame->data, impl_->rgb_frame->linesize,
                       reinterpret_cast<uint8_t*>(buffer.data()), AV_PIX_FMT_RGB24,
                       impl_->frame->width, impl_->frame->height, 1);
  sws_scale(impl_->sws, impl_->frame->data, impl_->frame->linesize, 0,
            impl_->frame->height, impl_->rgb_frame->data,
            impl_->rgb_frame->linesize);
  QImage image(reinterpret_cast<const uchar*>(buffer.constData()),
               impl_->frame->width, impl_->frame->height, impl_->rgb_frame->linesize[0],
               QImage::Format_RGB888);
  return image.copy();
}

#else

struct VideoStreamDecoder::Impl {};

VideoStreamDecoder::VideoStreamDecoder() = default;
VideoStreamDecoder::~VideoStreamDecoder() = default;
bool VideoStreamDecoder::isAvailable() const { return false; }
void VideoStreamDecoder::reset() {}
QImage VideoStreamDecoder::decodePacket(const QString&, const std::byte*, std::size_t) {
  return {};
}

#endif

}  // namespace image
}  // namespace autoviz
