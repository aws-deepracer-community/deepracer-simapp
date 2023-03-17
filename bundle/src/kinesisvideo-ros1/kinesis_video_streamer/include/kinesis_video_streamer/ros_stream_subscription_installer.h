/*
 * Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */
#pragma once
#include <aws/core/utils/logging/LogMacros.h>
#include <image_transport/image_transport.h>
#include <kinesis_manager/kinesis_stream_manager.h>
#include <kinesis_manager/stream_subscription_installer.h>
#include <kinesis_video_msgs/KinesisVideoFrame.h>
#include <kinesis_video_streamer/subscriber_callbacks.h>


namespace Aws {
namespace Kinesis {

/**
 * Stream type and the appropriate subscriber callback functions are inferred from the topic type
 * the node subscribes to.
 */
typedef enum kinesis_stream_ros_input_type_e {
  KINESIS_STREAM_INPUT_TYPE_KINESIS_VIDEO_FRAME =
    1, /* Implies a video stream with kinesis_video_msgs::KinesisVideoFrame message transport. */
  KINESIS_STREAM_INPUT_TYPE_IMAGE_TRANSPORT =
    2, /* Implies a video stream with sensor_msgs::Image message transport. */
  KINESIS_STREAM_INPUT_TYPE_REKOGNITION_ENABLED_KINESIS_VIDEO_FRAME =
    3 /* KinesisVideoFrame with AWS Rekognition results published to a ROS topic. */
} KinesisStreamRosInputType;

typedef void (*KinesisVideoFrameTransportCallbackFn)(
  KinesisStreamManagerInterface & stream_manager, std::string stream_name,
  const kinesis_video_msgs::KinesisVideoFrame::ConstPtr & frame_msg);

typedef void (*RekognitionEnabledKinesisVideoFrameTransportCallbackFn)(
  KinesisStreamManagerInterface & stream_manager, std::string stream_name,
  const kinesis_video_msgs::KinesisVideoFrame::ConstPtr & frame_msg,
  const ros::Publisher & publisher);

typedef void (*ImageTransportCallbackFn)(const KinesisStreamManagerInterface & stream_manager,
                                         std::string stream_name,
                                         const sensor_msgs::ImageConstPtr & image);

/**
 * Sets up subscriptions for ROS topics, supporting image transport and KinesisVideoFrame.
 */
class RosStreamSubscriptionInstaller : public StreamSubscriptionInstaller
{
public:
  RosStreamSubscriptionInstaller(ros::NodeHandle & handle)
  : handle_(&handle), StreamSubscriptionInstaller()
  {
  }

  /**
   * Initializes the subscription installer with the default callbacks defined at
   * subscriber_callbacks.h. Alternatively, the individual Setup* functions can be called to use
   * custom callbacks.
   * @return true on success
   */
  virtual bool SetDefaultCallbacks()
  {
    bool status = true;
    ImageTransportCallbackFn image_transport_callback;
    KinesisVideoFrameTransportCallbackFn kinesis_video_frame_transport_callback;
    RekognitionEnabledKinesisVideoFrameTransportCallbackFn rekognition_video_frame_callback;

    /* Set up subscription callbacks */
    image_transport_callback = &ImageTransportCallback;
    kinesis_video_frame_transport_callback = &KinesisVideoFrameTransportCallback;
    rekognition_video_frame_callback = &RekognitionEnabledKinesisVideoFrameTransportCallback;

    if (!this->SetupImageTransport(image_transport_callback) ||
        !this->SetupKinesisVideoFrameTransport(kinesis_video_frame_transport_callback) ||
        !this->SetupRekognitionEnabledKinesisVideoFrameTransport(
          rekognition_video_frame_callback)) {
      AWS_LOG_FATAL(__func__, "Failed to set up subscription callbacks.");
      status = false;
    }
    return status;
  }
  /**
   * Subscriber installation for sensor_msgs::Image
   * @param callback
   */
  bool SetupImageTransport(const ImageTransportCallbackFn callback);
  /**
   * Subscriber installation for kinesis_video_msgs::KinesisVideoFrame
   * @param callback
   */
  bool SetupKinesisVideoFrameTransport(const KinesisVideoFrameTransportCallbackFn callback);
  /**
   *
   * @param callback
   * @return
   */
  bool SetupRekognitionEnabledKinesisVideoFrameTransport(
    const RekognitionEnabledKinesisVideoFrameTransportCallbackFn callback);
  /**
   * Shuts down all ROS subscribers for topic_name.
   * @param topic_name
   */
  void Uninstall(const std::string & topic_name) override;
  /**
   * Updates the pointer to the stream manager which will be passed along to subscription callback
   * functions.
   * @param stream_manager
   */
  void set_stream_manager(KinesisStreamManagerInterface * stream_manager)
  {
    stream_manager_ = stream_manager;
  }

private:
  Aws::Kinesis::KinesisStreamManagerInterface * stream_manager_;
  ros::NodeHandle * handle_;
  std::vector<ros::Subscriber> standard_subscribers_;
  std::vector<image_transport::Subscriber> image_transport_subscribers_;
  std::map<std::string, ros::Publisher> publishers_; /* Publishers associated with a subscriber. Key
                                                        is the *subscription* topic name. */
};

}  // namespace Kinesis
}  // namespace Aws