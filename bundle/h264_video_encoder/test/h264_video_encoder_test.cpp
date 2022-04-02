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

#include <aws_ros1_common/sdk_utils/ros1_node_parameter_reader.h>
#include <aws/core/utils/logging/AWSLogging.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <gtest/gtest.h>
#include <h264_encoder_core/h264_encoder.h>
#include <h264_encoder_core/h264_encoder_node_config.h>
#include <image_transport/image_transport.h>
#include <kinesis_video_msgs/KinesisImageMetadata.h>
#include <kinesis_video_msgs/KinesisVideoFrame.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

using namespace Aws::Utils::Encoding;
using namespace Aws::Utils::Logging;

namespace Aws {
namespace Kinesis {

void InitializeEncoder(const sensor_msgs::ImageConstPtr & msg,
                       std::unique_ptr<H264Encoder> & encoder,
                       const Aws::Client::ParameterReaderInterface & param_reader);

void ImageCallback(const sensor_msgs::ImageConstPtr & msg, const H264Encoder * encoder,
                   uint64_t & frame_num, kinesis_video_msgs::KinesisImageMetadata & metadata,
                   ros::Publisher & pub);

void InitializeCommunication(ros::NodeHandle & nh,
                             ros::Subscriber & metadata_sub,
                             image_transport::Subscriber & image_sub,
                             ros::Publisher & pub,
                             std::unique_ptr<H264Encoder> & encoder,
                             uint64_t & frame_num,
                             kinesis_video_msgs::KinesisImageMetadata & metadata,
                             Aws::Client::Ros1NodeParameterReader & param_reader
                            );

}  // namespace Kinesis
}  // namespace Aws

constexpr static char kDefaultPublicationTopicName[] = "/video/encoded";
constexpr static char kDefaultSubscriptionTopicName[] = "/raspicam_node/image";
constexpr static char kDefaultMetadataTopicName[] = "/image_metadata";
constexpr static int kDefaultWidth = 410;
constexpr static int kDefaultHeight = 308;
constexpr static int kBytesPerPixel = 3;  // 3 color channels (red, green, blue)
constexpr int kNumTestFrames = 30;


class H264EncoderNodeSuite : public ::testing::Test
{
public:

  constexpr static const std::string & kDefaultEncoding = sensor_msgs::image_encodings::RGB8;

  H264EncoderNodeSuite() {}

protected:
  void SetUp() override
  {
    ros::Time::init();

    default_msg = boost::make_shared<sensor_msgs::Image>();
    default_msg->header.seq = 0;
    default_msg->header.frame_id = "";
    default_msg->header.stamp = ros::Time::now();
    default_msg->height = kDefaultHeight;
    default_msg->width = kDefaultWidth;
    default_msg->encoding = kDefaultEncoding;
    default_msg->step = kBytesPerPixel * kDefaultWidth;
  }

  void KinesisVideoCallback(const kinesis_video_msgs::KinesisVideoFrame::ConstPtr & frame)
  {
    if (frame->index > 0) {
      EXPECT_GT(frame->index, prev_frame.index);
      EXPECT_EQ(prev_frame.duration, frame->duration);
      EXPECT_EQ(0, (frame->decoding_ts - prev_frame.decoding_ts) % frame->duration);
      EXPECT_EQ(0, (frame->presentation_ts - prev_frame.presentation_ts) % frame->duration);
      EXPECT_GT(frame->presentation_ts, prev_frame.presentation_ts);
    }

    prev_frame = *frame;
    if (0 == prev_frame.index) {
      fwrite(prev_frame.codec_private_data.data(), 1,
             prev_frame.codec_private_data.size(), debug_file);
    }
    fwrite(prev_frame.frame_data.data(), 1, prev_frame.frame_data.size(), debug_file);
  }

  sensor_msgs::ImagePtr default_msg;
  std::unique_ptr<H264Encoder> encoder;
  Aws::Client::Ros1NodeParameterReader param_reader;

  kinesis_video_msgs::KinesisVideoFrame prev_frame;
  FILE * debug_file;
};

/**
 * Tests the callback of the H264 Encoder Node that performs the initialization
 */
TEST_F(H264EncoderNodeSuite, EncoderInit)
{
  std::unique_ptr<H264Encoder> invalid_encoder;
  sensor_msgs::ImagePtr invalid_msg = boost::make_shared<sensor_msgs::Image>();
  invalid_msg->encoding = "InvalidEncoding";
  Aws::Kinesis::InitializeEncoder(invalid_msg, invalid_encoder, param_reader);
  EXPECT_EQ(invalid_encoder, nullptr);
  Aws::Kinesis::InitializeEncoder(default_msg, encoder, param_reader);
  EXPECT_NE(encoder, nullptr);
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

void CreateImageMsg(sensor_msgs::ImagePtr & msg, int frame_num)
{
  ++msg->header.seq;
  msg->header.stamp = ros::Time::now();

  // prepare a dummy image
  int shift = static_cast<float>(frame_num) / (kNumTestFrames - 1) * kDefaultWidth;
  for (int y = 0; y < kDefaultHeight; ++y) {
    for (int x = 0; x < kDefaultWidth; ++x) {
      uint8_t r, g, b;
      RainbowColor(static_cast<float>((x + shift) % kDefaultWidth) / kDefaultWidth, r, g, b);
      msg->data[kBytesPerPixel * y * kDefaultWidth + kBytesPerPixel * x + 0] = r;
      msg->data[kBytesPerPixel * y * kDefaultWidth + kBytesPerPixel * x + 1] = g;
      msg->data[kBytesPerPixel * y * kDefaultWidth + kBytesPerPixel * x + 2] = b;
    }
  }
}

/**
 * Tests the callback of the H264 Encoder Node that performs the encoding
 */
TEST_F(H264EncoderNodeSuite, EncoderCallback)
{
  Aws::Kinesis::InitializeEncoder(default_msg, encoder, param_reader);
  EXPECT_NE(encoder, nullptr);

  ros::NodeHandle pub_node;
  ros::Publisher pub =
    pub_node.advertise<kinesis_video_msgs::KinesisVideoFrame>(kDefaultPublicationTopicName, 100);

  ros::NodeHandle sub_node;
  boost::function<void(const kinesis_video_msgs::KinesisVideoFrame::ConstPtr &)> callback;
  callback = [this](const kinesis_video_msgs::KinesisVideoFrame::ConstPtr & frame) -> void {
    this->KinesisVideoCallback(frame);
  };
  ros::Subscriber sub = sub_node.subscribe(kDefaultPublicationTopicName, 100, callback);

  default_msg->data.resize(kBytesPerPixel * kDefaultWidth * kDefaultHeight);
  debug_file = fopen("frames_encodercallback.bin", "wb");

  // let's encode 30 frames
  uint64_t prev_frame_index = 0, frame_index = 0;
  kinesis_video_msgs::KinesisImageMetadata metadata;
  for (int i = 0; i < kNumTestFrames; ++i) {
    CreateImageMsg(default_msg, i);
    Aws::Kinesis::ImageCallback(default_msg, encoder.get(), frame_index, metadata, pub);
    ros::spinOnce();

    EXPECT_GE(frame_index, prev_frame_index);
    prev_frame_index = frame_index;
  }

  fclose(debug_file);
  // you can dump the debug frames by executing: ffmpeg -i frames.bin -frames:v 10 -f image2
  // frame%03d.png
}


/**
 * Tests that InitializeCommunicaiton sets up the correct subscribers and publisher
 */

TEST_F(H264EncoderNodeSuite, InitializeCommunicaiton)
{
  ros::NodeHandle nh("~");
  ros::Publisher pub;
  image_transport::Subscriber image_sub;
  ros::Subscriber metadata_sub;
  uint64_t frame_num = 0;
  kinesis_video_msgs::KinesisImageMetadata metadata;
  Aws::Client::Ros1NodeParameterReader param_reader;
  Aws::Kinesis::InitializeCommunication(nh, metadata_sub, image_sub, pub,
                                        encoder, frame_num, metadata, param_reader);

  EXPECT_EQ(kDefaultPublicationTopicName, pub.getTopic());
  EXPECT_EQ(kDefaultSubscriptionTopicName, image_sub.getTopic());
  EXPECT_EQ(kDefaultMetadataTopicName, metadata_sub.getTopic());

  // Test that callback function is properly set up
  ros::NodeHandle sub_node;
  boost::function<void(const kinesis_video_msgs::KinesisVideoFrame::ConstPtr &)> callback;
  callback = [this](const kinesis_video_msgs::KinesisVideoFrame::ConstPtr & frame) -> void {
    this->KinesisVideoCallback(frame);
  };
  ros::Subscriber sub = sub_node.subscribe(kDefaultPublicationTopicName, 100, callback);

  // setup the raw image source
  default_msg->data.resize(kBytesPerPixel * kDefaultWidth * kDefaultHeight);
  ros::NodeHandle pub_node("~");
  ros::Publisher image_pub =
    pub_node.advertise<sensor_msgs::Image>(kDefaultSubscriptionTopicName, 100);

  // let's encode 30 frames of the raw image
  constexpr int kNumTestFrames = 30;
  debug_file = fopen("frames_intialize_communication.bin", "wb");
  uint64_t prev_frame_index = 0, frame_index = 0;
  for (int i = 0; i < kNumTestFrames; ++i) {
    CreateImageMsg(default_msg, i);
    image_pub.publish(default_msg);
    ros::spinOnce();
    EXPECT_GE(frame_index, prev_frame_index);
    prev_frame_index = frame_index;
  }

  fclose(debug_file);
}


int main(int argc, char ** argv)
{

  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_h264_video_encoder");
  return RUN_ALL_TESTS();
}
