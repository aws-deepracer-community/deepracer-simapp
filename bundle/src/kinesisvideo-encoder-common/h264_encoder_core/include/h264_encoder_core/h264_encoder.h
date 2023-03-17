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
#pragma once

#include <stdint.h>

#include <memory>
#include <vector>

extern "C" {
#include <libavutil/pixfmt.h>
}

#include <aws_common/sdk_utils/parameter_reader.h>

namespace Aws {
namespace Utils {
namespace Encoding {

struct H264EncoderResult
{
  H264EncoderResult() { Reset(); }

  void Reset()
  {
    frame_data.clear();
    frame_pts = 0;
    frame_dts = 0;
    frame_duration = 0;
    key_frame = false;
  }

  std::vector<uint8_t> frame_data;
  uint64_t frame_pts;
  uint64_t frame_dts;
  uint64_t frame_duration;
  bool key_frame;
};

class H264EncoderImpl;

class H264Encoder
{
public:
  H264Encoder();

  ~H264Encoder();

  /**
   * Initialize the H264Encoder instance
   * @param src_width the width of the source video stream
   * @param src_height the height of the source video stream
   * @param src_encoding the encoding of the source video stream
   * @param dst_params parameter reader used for reading the desired configuration of the encoder
   * output
   * @return AWS_ERR_OK if initialization was successful
   */
  AwsError Initialize(const int src_width, const int src_height, const AVPixelFormat src_encoding,
                      const Aws::Client::ParameterReaderInterface & dst_params);

  /**
   * Encode one frame
   * @param img_data frame data
   * @return resulting struct containing H264 encoded data
   */
  AwsError Encode(const uint8_t * img_data, H264EncoderResult & res) const;

  /**
   * Get the extra data from the FFmpeg encoder (get "codec private data" from here)
   * @return vector containing the extra data
   */
  std::vector<uint8_t> GetExtraData() const;

private:
  std::unique_ptr<H264EncoderImpl> impl_;
};

}  // namespace Encoding
}  // namespace Utils
}  // namespace Aws
