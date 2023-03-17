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

#include <aws_common/sdk_utils/aws_error.h>
#include <gtest/gtest.h>
#include <h264_encoder_core/h264_encoder.h>
#include <math.h>

#include <array>
#include <map>

constexpr int kDefaultSrcWidth = 410;
constexpr int kDefaultSrcHeight = 308;
constexpr AVPixelFormat kDefaultSrcEncoding = AV_PIX_FMT_RGB24;
constexpr int kBytesPerPixel = 3;  // 3 color channels (red, green, blue)
constexpr int kDefaultDstWidth = 1230;
constexpr int kDefaultDstHeight = 924;
constexpr int kDefaultFpsNumerator = 30;
constexpr int kDefaultFpsDenominator = 1;
constexpr int kDefaultBitrate = 2048000;
constexpr char kDefaultCodec[] = "libx264";

using namespace Aws;
using namespace Aws::Client;
using namespace Aws::Utils::Encoding;

/**
 * Parameter reader that sets the output using provided std::mapS.
 */
class TestParameterReader : public ParameterReaderInterface
{
public:
  TestParameterReader() {}

  TestParameterReader(int output_width, int output_height, int fps_num, int fps_denom, int bitrate,
                      const std::string & codec)
  {
    int_map_ = {{"output_width", output_width},
                {"output_height", output_height},
                {"fps_numerator", fps_num},
                {"fps_denominator", fps_denom},
                {"bitrate", bitrate}};
    string_map_ = {{"codec", codec}};
  }

  AwsError ReadParam(const ParameterPath & param_path, int & out) const
  {
    AwsError result = AWS_ERR_NOT_FOUND;
    std::string name = FormatParameterPath(param_path);
    if (int_map_.count(name) > 0) {
      out = int_map_.at(name);
      result = AWS_ERR_OK;
    }
    return result;
  }

  AwsError ReadParam(const ParameterPath & param_path, bool & out) const
  {
    return AWS_ERR_NOT_FOUND;
  }

  AwsError ReadParam(const ParameterPath & param_path, std::string & out) const
  {
    AwsError result = AWS_ERR_NOT_FOUND;
    std::string name = FormatParameterPath(param_path);
    if (string_map_.count(name) > 0) {
      out = string_map_.at(name);
      result = AWS_ERR_OK;
    }
    return result;
  }

  AwsError ReadParam(const ParameterPath & param_path, Aws::String & out) const
  {
    AwsError result = AWS_ERR_NOT_FOUND;
    std::string name = FormatParameterPath(param_path);
    if (string_map_.count(name) > 0) {
      out = string_map_.at(name).c_str();
      result = AWS_ERR_OK;
    }
    return result;
  }

  AwsError ReadParam(const ParameterPath & param_path, std::map<std::string, std::string> & out) const
  {
    return AWS_ERR_NOT_FOUND;
  }

  AwsError ReadParam(const ParameterPath & param_path, std::vector<std::string> & out) const
  {
    return AWS_ERR_NOT_FOUND;
  }

  AwsError ReadParam(const ParameterPath & param_path, double & out) const {
    return AWS_ERR_NOT_FOUND;
  }

private:
  std::string FormatParameterPath(const ParameterPath & param_path) const
  {
    return param_path.get_resolved_path('/', '/');
  }

  std::map<std::string, int> int_map_;
  std::map<std::string, std::string> string_map_;
};

/**
 * Tests the creation, initialization, and destruction of the H264 Encoder with empty parameter
 * server
 */
TEST(H264EncoderCoreSuite, InitWithEmptyParamServer)
{
  TestParameterReader param_reader;

  auto encoder = std::unique_ptr<H264Encoder>(new H264Encoder());

  AwsError result =
    encoder->Initialize(kDefaultSrcWidth, kDefaultSrcHeight, kDefaultSrcEncoding, param_reader);

  EXPECT_EQ(result, AWS_ERR_OK);
  EXPECT_FALSE(encoder->GetExtraData().empty());
}

/**
 * Tests the creation, initialization, and destruction of the H264 Encoder with valid parameters
 */
TEST(H264EncoderCoreSuite, InitWithFullParamServer)
{
  TestParameterReader param_reader(kDefaultDstWidth, kDefaultDstHeight, kDefaultFpsNumerator,
                                   kDefaultFpsDenominator, kDefaultBitrate, kDefaultCodec);

  auto encoder = std::unique_ptr<H264Encoder>(new H264Encoder());

  AwsError result =
    encoder->Initialize(kDefaultSrcWidth, kDefaultSrcHeight, kDefaultSrcEncoding, param_reader);

  EXPECT_EQ(result, AWS_ERR_OK);
  EXPECT_FALSE(encoder->GetExtraData().empty());
}

/**
 * Tests the creation, initialization, and destruction of the H264 Encoder with invalid parameters
 */
TEST(H264EncoderCoreSuite, InitWithInvalidParamServer)
{
  struct TestParamCollection
  {
    int src_width;
    int src_height;
    AVPixelFormat src_enc;
    int dst_width;
    int dst_height;
    int fps_num;
    int fps_denom;
    int bitrate;
    const char * codec;
  };

  const std::array<TestParamCollection, 9> params_sweep = {
    {{
       -1,                      // SrcWidth set to some invalid value
       kDefaultSrcHeight,       // SrcHeight
       kDefaultSrcEncoding,     // SrcEncoding
       kDefaultDstWidth,        // DstWidth
       kDefaultDstHeight,       // DstHeight
       kDefaultFpsNumerator,    // FpsNumerator
       kDefaultFpsDenominator,  // FpsDenominator
       kDefaultBitrate,         // Bitrate
       kDefaultCodec            // Codec
     },
     {
       kDefaultSrcWidth,        // SrcWidth
       -1,                      // SrcHeight set to some invalid value
       kDefaultSrcEncoding,     // SrcEncoding
       kDefaultDstWidth,        // DstWidth
       kDefaultDstHeight,       // DstHeight
       kDefaultFpsNumerator,    // FpsNumerator
       kDefaultFpsDenominator,  // FpsDenominator
       kDefaultBitrate,         // Bitrate
       kDefaultCodec            // Codec
     },
     {
       kDefaultSrcWidth,                // SrcWidth
       kDefaultSrcHeight,               // SrcHeight
       static_cast<AVPixelFormat>(-1),  // SrcEncoding set to some invalid value
       kDefaultDstWidth,                // DstWidth
       kDefaultDstHeight,               // DstHeight
       kDefaultFpsNumerator,            // FpsNumerator
       kDefaultFpsDenominator,          // FpsDenominator
       kDefaultBitrate,                 // Bitrate
       kDefaultCodec                    // Codec
     },
     {
       kDefaultSrcWidth,        // SrcWidth
       kDefaultSrcHeight,       // SrcHeight
       kDefaultSrcEncoding,     // SrcEncoding
       -1,                      // DstWidth set to some invalid value
       kDefaultDstHeight,       // DstHeight
       kDefaultFpsNumerator,    // FpsNumerator
       kDefaultFpsDenominator,  // FpsDenominator
       kDefaultBitrate,         // Bitrate
       kDefaultCodec            // Codec
     },
     {
       kDefaultSrcWidth,        // SrcWidth
       kDefaultSrcHeight,       // SrcHeight
       kDefaultSrcEncoding,     // SrcEncoding
       kDefaultDstWidth,        // DstWidth
       -1,                      // DstHeight set to some invalid value
       kDefaultFpsNumerator,    // FpsNumerator
       kDefaultFpsDenominator,  // FpsDenominator
       kDefaultBitrate,         // Bitrate
       kDefaultCodec            // Codec
     },
     {
       kDefaultSrcWidth,        // SrcWidth
       kDefaultSrcHeight,       // SrcHeight
       kDefaultSrcEncoding,     // SrcEncoding
       kDefaultDstWidth,        // DstWidth
       kDefaultDstHeight,       // DstHeight
       -1,                      // FpsNumerator set to some invalid value
       kDefaultFpsDenominator,  // FpsDenominator
       kDefaultBitrate,         // Bitrate
       kDefaultCodec            // Codec
     },
     {
       kDefaultSrcWidth,      // SrcWidth
       kDefaultSrcHeight,     // SrcHeight
       kDefaultSrcEncoding,   // SrcEncoding
       kDefaultDstWidth,      // DstWidth
       kDefaultDstHeight,     // DstHeight
       kDefaultFpsNumerator,  // FpsNumerator
       -1,                    // FpsDenominator set to some invalid value
       kDefaultBitrate,       // Bitrate
       kDefaultCodec          // Codec
     },
     {
       kDefaultSrcWidth,        // SrcWidth
       kDefaultSrcHeight,       // SrcHeight
       kDefaultSrcEncoding,     // SrcEncoding
       kDefaultDstWidth,        // DstWidth
       kDefaultDstHeight,       // DstHeight
       kDefaultFpsNumerator,    // FpsNumerator
       kDefaultFpsDenominator,  // FpsDenominator
       -1,                      // Bitrate set to some invalid value
       kDefaultCodec            // Codec
     },
     {
       kDefaultSrcWidth,        // SrcWidth
       kDefaultSrcHeight,       // SrcHeight
       kDefaultSrcEncoding,     // SrcEncoding
       kDefaultDstWidth,        // DstWidth
       kDefaultDstHeight,       // DstHeight
       kDefaultFpsNumerator,    // FpsNumerator
       kDefaultFpsDenominator,  // FpsDenominator
       kDefaultBitrate,         // Bitrate
       "fake codec name"        // Codec set to some invalid value
     }}};

  for (const auto & p : params_sweep) {
    TestParameterReader param_reader(p.dst_width, p.dst_height, p.fps_num, p.fps_denom, p.bitrate,
                                     p.codec);

    auto encoder = std::unique_ptr<H264Encoder>(new H264Encoder());

    AwsError result = encoder->Initialize(p.src_width, p.src_height, p.src_enc, param_reader);

    EXPECT_TRUE(AWS_ERR_PARAM == result || AWS_ERR_NOT_FOUND == result);
  }
}

static void RainbowColor(const float h, uint8_t & r_out, uint8_t & g_out, uint8_t & b_out)
{
  int i = 6.0f * h;
  float f = 6.0f * h - i;
  float t = f;
  float q = 1.0f - f;

  float r, g, b;
  switch (i % 6) {
    case 0:
      r = 1.0f;
      g = t;
      b = 0.0f;
      break;
    case 1:
      r = q;
      g = 1.0f;
      b = 0.0f;
      break;
    case 2:
      r = 0.0f;
      g = 1.0f;
      b = t;
      break;
    case 3:
      r = 0.0f;
      g = q;
      b = 1.0f;
      break;
    case 4:
      r = t;
      g = 0.0f;
      b = 1.0f;
      break;
    case 5:
      r = 1.0f;
      g = 0.0f;
      b = q;
      break;
  }

  r_out = std::lround(255.0f * r);
  g_out = std::lround(255.0f * g);
  b_out = std::lround(255.0f * b);
}

/**
 * Tests the creation, initialization, and destruction of the H264 Encoder with valid parameters
 */
TEST(H264EncoderCoreSuite, Encode)
{
  TestParameterReader param_reader(kDefaultDstWidth, kDefaultDstHeight, kDefaultFpsNumerator,
                                   kDefaultFpsDenominator, kDefaultBitrate, kDefaultCodec);

  auto encoder = std::unique_ptr<H264Encoder>(new H264Encoder());

  AwsError result =
    encoder->Initialize(kDefaultSrcWidth, kDefaultSrcHeight, kDefaultSrcEncoding, param_reader);
  ASSERT_EQ(result, AWS_ERR_OK);

  uint8_t * img_buffer = new uint8_t[kBytesPerPixel * kDefaultSrcWidth * kDefaultSrcHeight];
  FILE * debug_file = fopen("frames.bin", "wb");

  // encode (1 / kDefaultFpsDenominator) seconds of video
  for (int i = 0; i < kDefaultFpsNumerator; ++i) {
    // prepare a dummy image
    int shift = static_cast<float>(i) / (kDefaultFpsNumerator - 1) * kDefaultSrcWidth;
    for (int y = 0; y < kDefaultSrcHeight; ++y) {
      for (int x = 0; x < kDefaultSrcWidth; ++x) {
        uint8_t r, g, b;
        RainbowColor(static_cast<float>((x + shift) % kDefaultSrcWidth) / kDefaultSrcWidth, r, g,
                     b);
        img_buffer[kBytesPerPixel * y * kDefaultSrcWidth + kBytesPerPixel * x + 0] = r;
        img_buffer[kBytesPerPixel * y * kDefaultSrcWidth + kBytesPerPixel * x + 1] = g;
        img_buffer[kBytesPerPixel * y * kDefaultSrcWidth + kBytesPerPixel * x + 2] = b;
      }
    }

    H264EncoderResult encoder_output;
    AwsError result = encoder->Encode(img_buffer, encoder_output);

    EXPECT_TRUE(AWS_ERR_OK == result || AWS_ERR_EMPTY == result);
    if (AWS_ERR_OK == result) {
      EXPECT_EQ(encoder_output.frame_dts % encoder_output.frame_duration, 0);
      EXPECT_EQ(encoder_output.frame_pts % encoder_output.frame_duration, 0);
      EXPECT_EQ(static_cast<uint64_t>(1.0e6 * kDefaultFpsDenominator / kDefaultFpsNumerator),
                encoder_output.frame_duration);
    }

    if (0 == i) {
      fwrite(encoder->GetExtraData().data(), 1, encoder->GetExtraData().size(), debug_file);
    }
    fwrite(encoder_output.frame_data.data(), 1, encoder_output.frame_data.size(), debug_file);
  }

  fclose(debug_file);
  // you can dump the debug frames by executing: ffmpeg -i frames.bin -frames:v 10 -f image2
  // frame%03d.png
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
