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
#include <aws_ros1_common/sdk_utils/logging/aws_ros_logger.h>
#include <aws_ros1_common/sdk_utils/ros1_node_parameter_reader.h>
#include <h264_encoder_core/h264_encoder.h>
#include <h264_encoder_core/h264_encoder_node_config.h>
#include <image_transport/image_transport.h>
#include <kinesis_video_msgs/KinesisImageMetadata.h>
#include <kinesis_video_msgs/KinesisVideoFrame.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

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
void InitializeEncoder(const sensor_msgs::ImageConstPtr & msg,
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

void ImageCallback(const sensor_msgs::ImageConstPtr & msg, const H264Encoder * encoder,
                   uint64_t & frame_num, kinesis_video_msgs::KinesisImageMetadata & metadata,
                   ros::Publisher & pub)
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

  kinesis_video_msgs::KinesisVideoFrame frame;
  frame.index = frame_num;
  frame.flags = (encoder_output.key_frame) ? kKeyFrameFlag : kBPFrameFlag;
  frame.decoding_ts = encoder_output.frame_dts;
  frame.presentation_ts = encoder_output.frame_pts;
  frame.duration = encoder_output.frame_duration / 2;  // duration recommended to be set shorter
  frame.codec_private_data = encoder->GetExtraData();
  frame.frame_data = encoder_output.frame_data;
  frame.metadata.swap(metadata.metadata);

  pub.publish(frame);

  constexpr int kDbgMsgThrottlePeriod = 10;  // 10 seconds throttling period
  ROS_DEBUG_THROTTLE(kDbgMsgThrottlePeriod, "Published Frame #%lu (timestamp: %lu)\n", frame_num,
                     encoder_output.frame_pts);

  ++frame_num;
}

void InitializeCommunication(ros::NodeHandle & nh,
                             ros::Subscriber& metadata_sub,
                             image_transport::Subscriber& image_sub,
                             ros::Publisher& pub,
                             std::unique_ptr<H264Encoder>& encoder,
                             uint64_t & frame_num,
                             kinesis_video_msgs::KinesisImageMetadata & metadata,
                             Aws::Client::Ros1NodeParameterReader & param_reader)
{
  //
  // reading parameters
  //
  H264EncoderNodeParams params;
  GetH264EncoderNodeParams(param_reader, params);


  pub = nh.advertise<kinesis_video_msgs::KinesisVideoFrame>(params.publication_topic,
                                                            params.queue_size);

  //
  // subscribing to topic with callback
  //
  boost::function<void(const sensor_msgs::ImageConstPtr &)> image_callback;
  image_callback = [&](const sensor_msgs::ImageConstPtr & msg) -> void {
    if (0 < pub.getNumSubscribers()) {
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

  image_transport::ImageTransport it(nh);
  image_sub =
    it.subscribe(params.subscription_topic, params.queue_size, image_callback);
  AWS_LOGSTREAM_INFO(__func__, "subscribed to " << params.subscription_topic << "...");

  boost::function<void(const kinesis_video_msgs::KinesisImageMetadata::ConstPtr &)>
    metadata_callback;
  metadata_callback = [&](const kinesis_video_msgs::KinesisImageMetadata::ConstPtr & msg) -> void {
    metadata.metadata.insert(metadata.metadata.end(), msg->metadata.begin(), msg->metadata.end());
  };
  metadata_sub =
    nh.subscribe(params.metadata_topic, params.queue_size, metadata_callback);
  AWS_LOGSTREAM_INFO(__func__, "subscribed to " << params.metadata_topic << " for metadata...");
}

AwsError RunEncoderNode(int argc, char ** argv)
{
  ros::init(argc, argv, "h264_video_encoder");
  ros::NodeHandle nh("~");
  
  Aws::Utils::Logging::InitializeAWSLogging(
    Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>("h264_video_encoder"));
  AWS_LOG_INFO(__func__, "Starting H264 Video Node...");

  ros::Publisher pub;
  image_transport::Subscriber image_sub;
  ros::Subscriber metadata_sub;
  std::unique_ptr<H264Encoder> encoder;
  uint64_t frame_num = 0;
  kinesis_video_msgs::KinesisImageMetadata metadata;
  Aws::Client::Ros1NodeParameterReader param_reader;

  InitializeCommunication(nh, metadata_sub, image_sub, pub,
                          encoder, frame_num, metadata, param_reader);
  
  //
  // run the node
  //
  ros::spin();
  AWS_LOG_INFO(__func__, "Shutting down H264 Video Node...");
  Aws::Utils::Logging::ShutdownAWSLogging();
  return AWS_ERR_OK;
}

}  // namespace Kinesis
}  // namespace Aws
