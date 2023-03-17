/*
 *  Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <aws/core/utils/logging/LogMacros.h>
#include <h264_encoder_core/h264_encoder.h>

#include <cstdio>
#include <dlfcn.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

using namespace Aws::Client;
using namespace Aws::Utils::Logging;



namespace Aws {
namespace Utils {
namespace Encoding {

constexpr char kOutputWidthKey[] = "output_width";
constexpr char kOutputHeightKey[] = "output_height";
constexpr char kFpsNumeratorKey[] = "fps_numerator";
constexpr char kFpsDenominatorKey[] = "fps_denominator";
constexpr char kCodecKey[] = "codec";
constexpr char kBitrateKey[] = "bitrate";

constexpr char kDefaultHardwareCodec[] = "h264_omx";
constexpr char kDefaultSoftwareCodec[] = "libx264";
constexpr float kFragmentDuration = 1.0f; /* set fragment duration to 1.0 second */
constexpr int kDefaultMaxBFrames = 0;
constexpr int kDefaultFpsNumerator = 24;
constexpr int kDefaultFpsDenominator = 1;
constexpr int kDefaultBitrate = 2048000;

class H264EncoderImpl
{
public:
  H264EncoderImpl()
  : src_width_(-1),
    src_height_(-1),
    src_encoding_(AV_PIX_FMT_NONE),
    src_stride_(-1),
    dst_width_(-1),
    dst_height_(-1),
    convert_ctx_(nullptr),
    bitrate_(-1),
    fps_num_(-1),
    fps_den_(-1),
    param_(nullptr),
    pic_in_(nullptr)
  {
  }

  /* Setup param_ 
   * Function will fail if param_ is not nullptr
   */
  AwsError set_param(AVCodec * codec) {
    if (nullptr != param_) {
      AWS_LOG_ERROR(__func__, "Unable to setup codec context. param_ must be null");
      return AWS_ERR_FAILURE;
    }
    param_ = avcodec_alloc_context3(codec);
    if (nullptr == param_) {
      AWS_LOG_ERROR(__func__, "Could not allocate video codec context");
      return AWS_ERR_MEMORY;
    }
    /* put sample parameters */
    param_->bit_rate = bitrate_;
    /* resolution must be a multiple of two */
    param_->width = dst_width_;
    param_->height = dst_height_;
    /* frames per second */
    param_->time_base = (AVRational){fps_den_, fps_num_};
    frame_duration_ = (1e6 * fps_den_) / fps_num_;
    param_->gop_size = static_cast<int>(ceil(kFragmentDuration * fps_num_ / fps_den_));
    param_->keyint_min = param_->gop_size - 1;
    param_->max_b_frames = kDefaultMaxBFrames;
    param_->pix_fmt = AV_PIX_FMT_YUV420P;

    param_->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    param_->flags2 &= ~AV_CODEC_FLAG2_LOCAL_HEADER;

    return AWS_ERR_OK;
  }
  
  // Attempts to open a codec
  AwsError open_codec(AVCodec * codec, AVDictionary * opts) {
    if (nullptr == codec) {
      AWS_LOG_ERROR(__func__, "Invalid codec");
      return AWS_ERR_FAILURE;
    }

    AWS_LOGSTREAM_INFO(__func__, "Attempting to open codec: " << codec->name);

    if (AWS_ERR_OK != set_param(codec) || avcodec_open2(param_, codec, &opts) < 0 ) {
      AWS_LOG_ERROR(__func__, "Could not open codec");
      if (nullptr != param_) {
	      avcodec_close(param_);
	      av_free(param_);
	      param_ = nullptr;
      }
      return AWS_ERR_FAILURE;
    }
   
    return AWS_ERR_OK;
  }

  AwsError Initialize(const int src_width, const int src_height, const AVPixelFormat src_encoding,
                      const std::string & codec_name, const int dst_width, const int dst_height,
                      const int fps_num, const int fps_den, const int64_t bitrate)
  {
    if (src_width <= 0) {
      AWS_LOGSTREAM_ERROR(__func__, "Invalid video source width " << src_width << "!");
      return AWS_ERR_PARAM;
    }
    if (src_height <= 0) {
      AWS_LOGSTREAM_ERROR(__func__, "Invalid video source height " << src_height << "!");
      return AWS_ERR_PARAM;
    }
    if (dst_width <= 0) {
      AWS_LOGSTREAM_ERROR(__func__, "Invalid output video width " << dst_width << "!");
      return AWS_ERR_PARAM;
    }
    if (dst_height <= 0) {
      AWS_LOGSTREAM_ERROR(__func__, "Invalid video source height " << dst_height << "!");
      return AWS_ERR_PARAM;
    }
    if (fps_num <= 0) {
      AWS_LOGSTREAM_ERROR(__func__, "Invalid FPS numerator " << fps_num << "!");
      return AWS_ERR_PARAM;
    }
    if (fps_den <= 0) {
      AWS_LOGSTREAM_ERROR(__func__, "Invalid FPS denominator " << fps_den << "!");
      return AWS_ERR_PARAM;
    }
    if (bitrate <= 0) {
      AWS_LOGSTREAM_ERROR(__func__, "Invalid bit rate " << bitrate << "!");
      return AWS_ERR_PARAM;
    }

    src_width_ = src_width;
    src_height_ = src_height;
    src_encoding_ = src_encoding;
    if (src_encoding_ == AV_PIX_FMT_RGB24) {
      src_stride_ = 3 * src_width_;  // 3 color channels (red, green, blue)
    } else if (src_encoding_ == AV_PIX_FMT_BGR24) {
      src_stride_ = 3 * src_width_;  // 3 color channels (blue, green, red)
    } else if (src_encoding_ == AV_PIX_FMT_RGBA) {
      src_stride_ = 4 * src_width_;  // 4 color channels (red, green, blue, alpha)
    } else if (src_encoding_ == AV_PIX_FMT_BGRA) {
      src_stride_ = 4 * src_width_;  // 4 color channels (blue, green, red, alpha)
    } else {
      AWS_LOG_ERROR(__func__, "Trying to work with unsupported encoding!");
      return AWS_ERR_PARAM;
    }

    dst_width_ = dst_width;
    dst_height_ = dst_height;
    fps_num_ = fps_num;
    fps_den_ = fps_den;
    bitrate_ = bitrate;

    avcodec_register_all();

    /* find the mpeg1 video encoder */
    AVCodec * codec = nullptr;
    AVDictionary * opts = nullptr;
    if (codec_name.empty()) {
      codec = avcodec_find_encoder_by_name(kDefaultHardwareCodec);
      if (AWS_ERR_OK != open_codec(codec, opts)) {
        codec = avcodec_find_encoder_by_name(kDefaultSoftwareCodec);
	      av_dict_set(&opts, "preset", "veryfast", 0);
        av_dict_set(&opts, "tune", "zerolatency", 0);

        if (AWS_ERR_OK != open_codec(codec, opts)) {
          AWS_LOGSTREAM_ERROR(__func__, kDefaultHardwareCodec << " and " << kDefaultSoftwareCodec
                                                              << " codecs were not available!");
          return AWS_ERR_NOT_FOUND;
        }
      }
    } else {
      codec = avcodec_find_encoder_by_name(codec_name.c_str());
      if (AWS_ERR_OK != open_codec(codec, opts)) {
        AWS_LOGSTREAM_ERROR(__func__, codec_name << " codec not found!");
        return AWS_ERR_NOT_FOUND;
      }
    }
    AWS_LOGSTREAM_INFO(__func__, "Encoding using " << codec->name << " codec");
    


    dst_width_ = param_->width;
    dst_height_ = param_->height;

    pic_in_ = av_frame_alloc();
    if (nullptr == pic_in_) {
      AWS_LOG_ERROR(__func__, "Could not allocate video frame");
      return AWS_ERR_MEMORY;
    }
    pic_in_->format = param_->pix_fmt;
    pic_in_->width = param_->width;
    pic_in_->height = param_->height;
    pic_in_->pts = 0;

    /* the image can be allocated by any means and av_image_alloc() is
     * just the most convenient way if av_malloc() is to be used */
    int ret = av_image_alloc(pic_in_->data, pic_in_->linesize, param_->width, param_->height,
                             param_->pix_fmt, 32);
    if (ret < 0) {
      AWS_LOGSTREAM_ERROR(__func__,
                          "Could not allocate raw picture buffer"
                          " (av_image_alloc() returned: "
                            << ret << ")");
      return AWS_ERR_MEMORY;
    }

    convert_ctx_ = sws_getContext(src_width_, src_height_, src_encoding_, dst_width_, dst_height_,
                                  AV_PIX_FMT_YUV420P, SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);

    return AWS_ERR_OK;
  }

  ~H264EncoderImpl()
  {
    if (nullptr != convert_ctx_) {
      sws_freeContext(convert_ctx_);
    }

    if (nullptr != param_) {
      avcodec_close(param_);
      av_free(param_);
    }

    if (nullptr != pic_in_) {
      av_freep(&pic_in_->data[0]);
      av_frame_free(&pic_in_);
    }
  }

  AwsError Encode(const uint8_t * img_data, H264EncoderResult & res)
  {
    if (nullptr == img_data) {
      return AWS_ERR_NULL_PARAM;
    }

    AVPacket pkt;

    /* Convert from image encoding to YUV420P */
    const uint8_t * buf_in[4] = {img_data, nullptr, nullptr, nullptr};
    sws_scale(convert_ctx_, (const uint8_t * const *)buf_in, &src_stride_, 0, src_height_,
              pic_in_->data, pic_in_->linesize);

    /* Encode */
    av_init_packet(&pkt);
    pkt.data = nullptr;  // packet data will be allocated by the encoder
    pkt.size = 0;

    int got_output = 0;

    int ret = avcodec_encode_video2(param_, &pkt, pic_in_, &got_output);
    ++pic_in_->pts;
    if (ret < 0) {
      AWS_LOGSTREAM_ERROR(__func__,
                          "Error encoding frame (avcodec_encode_video2() returned: " << ret << ")");
      return AWS_ERR_FAILURE;
    }
    if (got_output) {
      res.Reset();
      res.frame_data.insert(res.frame_data.end(), pkt.data, pkt.data + pkt.size);
      res.frame_pts = frame_duration_ * (0 <= pkt.pts ? pkt.pts : 0);
      res.frame_dts = frame_duration_ * (0 <= pkt.dts ? pkt.dts : 0);
      res.frame_duration = frame_duration_;
      res.key_frame = pkt.flags & AV_PKT_FLAG_KEY;
      av_free_packet(&pkt);

      return AWS_ERR_OK;
    }

    return AWS_ERR_EMPTY;
  }

  std::vector<uint8_t> GetExtraData() const
  {
    if (param_ != nullptr && param_->extradata != nullptr && param_->extradata_size > 0) {
      return std::vector<uint8_t>(param_->extradata, param_->extradata + param_->extradata_size);
    } else {
      return std::vector<uint8_t>();
    }
  }

private:
  int src_width_;
  int src_height_;
  AVPixelFormat src_encoding_;
  int src_stride_;
  int dst_width_;
  int dst_height_;
  int bitrate_;
  struct SwsContext * convert_ctx_;

  int fps_num_;
  int fps_den_;
  uint64_t frame_duration_;  // in microseconds
  AVCodecContext * param_;
  AVFrame * pic_in_;
};

H264Encoder::H264Encoder() {}

H264Encoder::~H264Encoder() {}

AwsError H264Encoder::Initialize(const int src_width, const int src_height,
                                 const AVPixelFormat src_encoding,
                                 const ParameterReaderInterface & dst_params)
{
  int dst_width, dst_height;
  bool dims_set = (dst_params.ReadParam(ParameterPath(kOutputWidthKey), dst_width) == Aws::AWS_ERR_OK &&
                   dst_params.ReadParam(ParameterPath(kOutputHeightKey), dst_height) == Aws::AWS_ERR_OK);
  if (!dims_set) {
    dst_width = src_width;
    dst_height = src_height;
  }

  int fps_num, fps_den;
  bool fps_set = (dst_params.ReadParam(ParameterPath(kFpsNumeratorKey), fps_num) == Aws::AWS_ERR_OK &&
                  dst_params.ReadParam(ParameterPath(kFpsDenominatorKey), fps_den) == Aws::AWS_ERR_OK);
  if (!fps_set) {
    AWS_LOG_WARN(__func__, "fps not set");
    fps_num = kDefaultFpsNumerator;
    fps_den = kDefaultFpsDenominator;
  }

  std::string codec;
  dst_params.ReadParam(ParameterPath(kCodecKey), codec);

  int bitrate = kDefaultBitrate;
  dst_params.ReadParam(ParameterPath(kBitrateKey), bitrate);

  impl_ = std::unique_ptr<H264EncoderImpl>(new H264EncoderImpl());

  return impl_->Initialize(src_width, src_height, src_encoding, codec, dst_width, dst_height,
                           fps_num, fps_den, bitrate);
}

AwsError H264Encoder::Encode(const uint8_t * img_data, H264EncoderResult & res) const
{
  return impl_->Encode(img_data, res);
}

std::vector<uint8_t> H264Encoder::GetExtraData() const { return impl_->GetExtraData(); }

}  // namespace Encoding
}  // namespace Utils
}  // namespace Aws
