// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include <string.h>

#include <aws_ros2_common/sdk_utils/ros2_node_parameter_reader.h>
#include <aws/core/utils/logging/AWSLogging.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <gtest/gtest.h>
#include <h264_encoder_core/h264_encoder.h>
#include <h264_encoder_core/h264_encoder_node_config.h>
#include <image_transport/image_transport.hpp>
#include <kinesis_video_msgs/msg/kinesis_image_metadata.hpp>
#include <kinesis_video_msgs/msg/kinesis_video_frame.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/image_encodings.hpp>

using namespace Aws::Utils::Encoding;
using namespace Aws::Utils::Logging;

namespace Aws {
namespace Kinesis {

void InitializeEncoder(sensor_msgs::msg::Image::ConstSharedPtr msg,
                       std::unique_ptr<H264Encoder> & encoder,
                       const Aws::Client::ParameterReaderInterface & param_reader);

void ImageCallback(sensor_msgs::msg::Image::ConstSharedPtr msg, const H264Encoder * encoder,
                   uint64_t & frame_num, kinesis_video_msgs::msg::KinesisImageMetadata & metadata,
                   std::shared_ptr<rclcpp::Publisher<kinesis_video_msgs::msg::KinesisVideoFrame>> pub);

void InitializeCommunication(rclcpp::Node::SharedPtr node,
                             std::shared_ptr<rclcpp::Subscription<kinesis_video_msgs::msg::KinesisImageMetadata>> & metadata_sub,
                             image_transport::Subscriber & image_sub,
                             std::shared_ptr<rclcpp::Publisher<kinesis_video_msgs::msg::KinesisVideoFrame>> & pub,
                             std::unique_ptr<H264Encoder> & encoder,
                             uint64_t & frame_num,
                             kinesis_video_msgs::msg::KinesisImageMetadata & metadata,
                             Aws::Client::Ros2NodeParameterReader & param_reader
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
static const std::string & kDefaultEncoding = sensor_msgs::image_encodings::RGB8;

class H264EncoderNodeSuite : public ::testing::Test
{
public:


  H264EncoderNodeSuite() {}

protected:
  void SetUp() override
  {
    default_msg = std::make_shared<sensor_msgs::msg::Image>();
    default_msg->header.frame_id = "";
    default_msg->header.stamp = rclcpp::Clock().now();
    default_msg->height = kDefaultHeight;
    default_msg->width = kDefaultWidth;
    default_msg->encoding = kDefaultEncoding;
    default_msg->step = kBytesPerPixel * kDefaultWidth;
  }

  void KinesisVideoCallback(const std::shared_ptr<const kinesis_video_msgs::msg::KinesisVideoFrame> frame)
  {
    if (prev_frame.index > 0) {
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

  sensor_msgs::msg::Image::SharedPtr default_msg;
  std::unique_ptr<H264Encoder> encoder;

  kinesis_video_msgs::msg::KinesisVideoFrame prev_frame;
  FILE * debug_file;
};

/**
 * Tests the callback of the H264 Encoder Node that performs the initialization
 */
TEST_F(H264EncoderNodeSuite, EncoderInit)
{
  auto node = rclcpp::Node::make_shared("node");
  Aws::Client::Ros2NodeParameterReader param_reader(node);
  std::unique_ptr<H264Encoder> invalid_encoder;
  sensor_msgs::msg::Image::SharedPtr invalid_msg = std::make_shared<sensor_msgs::msg::Image>();
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

void CreateImageMsg(sensor_msgs::msg::Image::SharedPtr msg, int frame_num)
{
  msg->header.stamp = rclcpp::Clock().now();

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
  
  auto pub_node = rclcpp::Node::make_shared("pub_node");
  auto pub =
    pub_node->create_publisher<kinesis_video_msgs::msg::KinesisVideoFrame>(kDefaultPublicationTopicName, 100);

  Aws::Client::Ros2NodeParameterReader param_reader(pub_node);
  Aws::Kinesis::InitializeEncoder(default_msg, encoder, param_reader);
  EXPECT_NE(encoder, nullptr);

  auto sub_node = rclcpp::Node::make_shared("sub_node");
auto callback = [this](const std::shared_ptr<const kinesis_video_msgs::msg::KinesisVideoFrame> frame) -> void {
    this->KinesisVideoCallback(frame);
  };
  auto sub = sub_node->create_subscription<kinesis_video_msgs::msg::KinesisVideoFrame>(kDefaultPublicationTopicName, 100, callback);

  default_msg->data.resize(kBytesPerPixel * kDefaultWidth * kDefaultHeight);
  debug_file = fopen("frames_encodercallback.bin", "wb");

  // let's encode 30 frames
  uint64_t prev_frame_index = 0, frame_index = 0;
  kinesis_video_msgs::msg::KinesisImageMetadata metadata;
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(pub_node);
  executor.add_node(sub_node);
  for (int i = 0; i < kNumTestFrames; ++i) {
    CreateImageMsg(default_msg, i);
    Aws::Kinesis::ImageCallback(default_msg, encoder.get(), frame_index, metadata, pub);
    executor.spin_some();
    //rclcpp::spin_some(pub_node);
    //rclcpp::spin_some(sub_node);

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
  auto init_node = rclcpp::Node::make_shared("init_node");
  std::shared_ptr<rclcpp::Publisher<kinesis_video_msgs::msg::KinesisVideoFrame>> pub;
  image_transport::Subscriber image_sub;
  std::shared_ptr<rclcpp::Subscription<kinesis_video_msgs::msg::KinesisImageMetadata>> metadata_sub;
  uint64_t frame_num = 0;
  kinesis_video_msgs::msg::KinesisImageMetadata metadata;
  Aws::Client::Ros2NodeParameterReader param_reader(init_node);
  AWS_LOG_ERROR(__func__, "Initialized communication\n");
  Aws::Kinesis::InitializeCommunication(init_node, metadata_sub, image_sub, pub,
                                        encoder, frame_num, metadata, param_reader);

  EXPECT_EQ(std::string(kDefaultMetadataTopicName), std::string(metadata_sub->get_topic_name()));
  EXPECT_EQ(std::string(kDefaultPublicationTopicName), std::string(pub->get_topic_name()));
  EXPECT_EQ(std::string(kDefaultSubscriptionTopicName), std::string(image_sub.getTopic()));

  // Test that callback function is properly set up
  auto sub_node = rclcpp::Node::make_shared("sub_node");
  auto callback = [this](const std::shared_ptr<const kinesis_video_msgs::msg::KinesisVideoFrame> frame) -> void {
    this->KinesisVideoCallback(frame);
  };
  auto callback_sub = sub_node->create_subscription<kinesis_video_msgs::msg::KinesisVideoFrame>(kDefaultPublicationTopicName, 100, callback);

  // setup the raw image source
  default_msg->data.resize(kBytesPerPixel * kDefaultWidth * kDefaultHeight);
  auto pub_node = rclcpp::Node::make_shared("pub_node");
  auto image_pub =
    pub_node->create_publisher<sensor_msgs::msg::Image>(kDefaultSubscriptionTopicName, 100);

  // let's encode 30 frames of the raw image
  constexpr int kNumTestFrames = 30;
  debug_file = fopen("frames_intialize_communication.bin", "wb");
  uint64_t prev_frame_index = 0, frame_index = 0;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(init_node);
  executor.add_node(sub_node);
  executor.add_node(pub_node);
  for (int i = 0; i < kNumTestFrames; ++i) {
    CreateImageMsg(default_msg, i);
    image_pub->publish(*default_msg);
    executor.spin_some();
    EXPECT_GE(frame_index, prev_frame_index);
    prev_frame_index = frame_index;
  }

  fclose(debug_file);
}


int main(int argc, char ** argv)
{

  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
