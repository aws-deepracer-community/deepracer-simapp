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

#include <aws/core/utils/logging/AWSLogging.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <aws_ros2_common/sdk_utils/logging/aws_ros_logger.h>
#include <aws_ros2_common/sdk_utils/ros2_node_parameter_reader.h>
#include <h264_encoder_core/h264_encoder.h>
#include <h264_encoder_core/h264_encoder_node_config.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>
#include <kinesis_video_msgs/msg/kinesis_image_metadata.hpp>
#include <kinesis_video_msgs/msg/kinesis_video_frame.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <map>
#include <string>
#include <vector>

using namespace Aws::Utils::Encoding;
using namespace Aws::Utils::Logging;

namespace Aws {
namespace Kinesis {

const std::map<std::string, AVPixelFormat> SNSR_IMG_ENC_to_LIBAV_PIXEL_FRMT = {
  {sensor_msgs::image_encodings::RGB8, AV_PIX_FMT_RGB24},
  {sensor_msgs::image_encodings::BGR8, AV_PIX_FMT_BGR24},
  {sensor_msgs::image_encodings::RGBA8, AV_PIX_FMT_RGBA},
  {sensor_msgs::image_encodings::BGRA8, AV_PIX_FMT_BGRA}};

/**
 * Initialize the H264Encoder
 * @param msg the message from the image sensor through image transport
 * @param encoder reference to pointer that owns the H264Encoder instance. if
 *  the pointer is null, it will be modified to the address of the new H264Encoder instance
 * @param param_reader parameter reader used for reading the desired configuration of the encoder
 * output
 */
void InitializeEncoder(sensor_msgs::msg::Image::ConstSharedPtr msg,
                       std::unique_ptr<H264Encoder> & encoder,
                       const Aws::Client::ParameterReaderInterface & param_reader)
{
  auto encoding_iter = SNSR_IMG_ENC_to_LIBAV_PIXEL_FRMT.find(msg->encoding);
  if (encoding_iter == SNSR_IMG_ENC_to_LIBAV_PIXEL_FRMT.end()) {
    AWS_LOGSTREAM_ERROR(__func__,
                        "Trying to work with unsupported encoding " << msg->encoding << "!");
    return;
  }

  encoder = std::unique_ptr<H264Encoder>(new H264Encoder());
  if (nullptr != encoder) {
    encoder->Initialize(msg->width, msg->height, encoding_iter->second, param_reader);
  }
}

void ImageCallback(sensor_msgs::msg::Image::ConstSharedPtr msg, const H264Encoder * encoder,
                   uint64_t & frame_num, kinesis_video_msgs::msg::KinesisImageMetadata & metadata,
                   std::shared_ptr<rclcpp::Publisher<kinesis_video_msgs::msg::KinesisVideoFrame>> pub)
{
  thread_local H264EncoderResult encoder_output;

  AwsError retcode = encoder->Encode(msg->data.data(), encoder_output);
  if (retcode != AWS_ERR_OK) {
    if (retcode == AWS_ERR_NULL_PARAM) {
      AWS_LOG_ERROR(__func__, "Encoder received empty data!");
    } else if (retcode == AWS_ERR_FAILURE) {
      AWS_LOG_ERROR(__func__, "Unknown encoding error occurred");
    } else if (retcode == AWS_ERR_EMPTY) {
      AWS_LOG_WARN(__func__, "Encoder returned empty frame");
    }
    return;
  }

  kinesis_video_msgs::msg::KinesisVideoFrame frame;
  frame.index = frame_num;
  frame.flags = (encoder_output.key_frame) ? kKeyFrameFlag : kBPFrameFlag;
  frame.decoding_ts = encoder_output.frame_dts;
  frame.presentation_ts = encoder_output.frame_pts;
  frame.duration = encoder_output.frame_duration / 2;  // duration recommended to be set shorter
  frame.codec_private_data = encoder->GetExtraData();
  frame.frame_data = encoder_output.frame_data;
  frame.metadata.swap(metadata.metadata);

  pub->publish(frame);

  ++frame_num;
}

void InitializeCommunication(rclcpp::Node::SharedPtr node,
                             std::shared_ptr<rclcpp::Subscription<kinesis_video_msgs::msg::KinesisImageMetadata>> & metadata_sub,
                             image_transport::Subscriber& image_sub,
                             std::shared_ptr<rclcpp::Publisher<kinesis_video_msgs::msg::KinesisVideoFrame>> & pub,
                             std::unique_ptr<H264Encoder>& encoder,
                             uint64_t & frame_num,
                             kinesis_video_msgs::msg::KinesisImageMetadata & metadata,
                             Aws::Client::Ros2NodeParameterReader & param_reader)
{
  //
  // reading parameters
  //
  H264EncoderNodeParams params;
  GetH264EncoderNodeParams(param_reader, params);


  pub = node->create_publisher<kinesis_video_msgs::msg::KinesisVideoFrame>(params.publication_topic,
                                                            params.queue_size);

  //
  // subscribing to topic with callback
  //
  auto image_callback = [&](const sensor_msgs::msg::Image::ConstSharedPtr msg) -> void {
    if (0 < pub->get_subscription_count()) {
      if (nullptr == encoder) {
        InitializeEncoder(msg, encoder, param_reader);
      }
      if (nullptr != encoder) {
        ImageCallback(msg, encoder.get(), frame_num, metadata, pub);
      }
    } else {
      frame_num = 0;
    }
  };

  image_transport::ImageTransport it(node);
  image_sub =
    it.subscribe(params.subscription_topic, params.queue_size, image_callback);
  AWS_LOGSTREAM_INFO(__func__, "subscribed to " << params.subscription_topic << "...");

    auto metadata_callback = [&](const std::shared_ptr<const kinesis_video_msgs::msg::KinesisImageMetadata> msg) -> void {
    metadata.metadata.insert(metadata.metadata.end(), msg->metadata.begin(), msg->metadata.end());
  };
  metadata_sub = node->create_subscription<kinesis_video_msgs::msg::KinesisImageMetadata>(params.metadata_topic, params.queue_size, metadata_callback);
  AWS_LOGSTREAM_INFO(__func__, "subscribed to " << params.metadata_topic << " for metadata...");
}

AwsError RunEncoderNode(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("h264_video_encoder", node_options);

 
  Aws::Utils::Logging::InitializeAWSLogging(Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>(
    "h264_video_encoder", Aws::Utils::Logging::LogLevel::Trace, std::weak_ptr<rclcpp::Node>(node)));
  AWS_LOG_INFO(__func__, "Starting H264 Video Node...");

  std::shared_ptr<rclcpp::Publisher<kinesis_video_msgs::msg::KinesisVideoFrame>> pub;
  std::shared_ptr<rclcpp::Subscription<kinesis_video_msgs::msg::KinesisImageMetadata>> metadata_sub;
  image_transport::Subscriber image_sub;
  std::unique_ptr<H264Encoder> encoder;
  uint64_t frame_num = 0;
  kinesis_video_msgs::msg::KinesisImageMetadata metadata;
  Aws::Client::Ros2NodeParameterReader param_reader(node);

  InitializeCommunication(node, metadata_sub, image_sub, pub,
                          encoder, frame_num, metadata, param_reader);
  
  //
  // run the node
  //
  rclcpp::spin(node);
  AWS_LOG_INFO(__func__, "Shutting down H264 Video Node...");
  Aws::Utils::Logging::ShutdownAWSLogging();
  return AWS_ERR_OK;
}

}  // namespace Kinesis
}  // namespace Aws
