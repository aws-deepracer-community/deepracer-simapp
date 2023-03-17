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
#include <aws/core/utils/logging/LogMacros.h>
#include <kinesis_video_streamer/ros_stream_subscription_installer.h>
#include <std_msgs/String.h>

namespace Aws {
namespace Kinesis {

bool RosStreamSubscriptionInstaller::SetupImageTransport(const ImageTransportCallbackFn callback)
{
  if (!callback) {
    AWS_LOG_ERROR(__func__, "Invalid callback was provided");
    return false;
  }
  SubscriberSetupFn image_transport_setup_closure =
    [this, callback](const StreamSubscriptionDescriptor & descriptor) -> bool {
    image_transport::ImageTransport it(*handle_);
    boost::function<void(const sensor_msgs::ImageConstPtr &)> callback_wrapper;
    callback_wrapper = [this, callback,
                        descriptor](const sensor_msgs::ImageConstPtr & image) -> void {
      callback(*this->stream_manager_, descriptor.stream_name, image);
    };
    image_transport_subscribers_.push_back(
      it.subscribe(descriptor.topic_name, descriptor.message_queue_size, callback_wrapper));
    return true;
  };
  installers_.insert({KINESIS_STREAM_INPUT_TYPE_IMAGE_TRANSPORT, image_transport_setup_closure});
  return true;
}

bool RosStreamSubscriptionInstaller::SetupKinesisVideoFrameTransport(
  const KinesisVideoFrameTransportCallbackFn callback)
{
  if (!callback) {
    AWS_LOG_ERROR(__func__, "Invalid callback was provided");
    return false;
  }
  SubscriberSetupFn kinesis_video_frame_setup_closure =
    [this, callback](const StreamSubscriptionDescriptor & descriptor) -> bool {
    boost::function<void(const kinesis_video_msgs::KinesisVideoFrame::ConstPtr &)> callback_wrapper;
    callback_wrapper = [this, callback, descriptor](
                         const kinesis_video_msgs::KinesisVideoFrame::ConstPtr & frame) -> void {
      callback(*this->stream_manager_, descriptor.stream_name, frame);
    };
    standard_subscribers_.push_back(handle_->subscribe(
      descriptor.topic_name.c_str(), descriptor.message_queue_size, callback_wrapper));
    return true;
  };
  installers_.insert(
    {KINESIS_STREAM_INPUT_TYPE_KINESIS_VIDEO_FRAME, kinesis_video_frame_setup_closure});
  return true;
}

bool RosStreamSubscriptionInstaller::SetupRekognitionEnabledKinesisVideoFrameTransport(
  const RekognitionEnabledKinesisVideoFrameTransportCallbackFn callback)
{
  if (!callback) {
    AWS_LOG_ERROR(__func__, "Invalid callback was provided");
    return false;
  }
  SubscriberSetupFn rekognition_kinesis_video_frame_setup_closure =
    [this, callback](const StreamSubscriptionDescriptor & descriptor) -> bool {
    if (descriptor.rekognition_topic_name.empty()) {
      AWS_LOG_ERROR(__func__, "Can't set up subscription: Rekognition topic name is empty.");
      return false;
    }
    boost::function<void(const kinesis_video_msgs::KinesisVideoFrame::ConstPtr &)> callback_wrapper;
    ros::Publisher publisher = handle_->advertise<std_msgs::String>(
      descriptor.rekognition_topic_name, descriptor.message_queue_size);
    callback_wrapper = [this, callback, descriptor, publisher](
                         const kinesis_video_msgs::KinesisVideoFrame::ConstPtr & frame) -> void {
      callback(*this->stream_manager_, descriptor.stream_name, frame, publisher);
    };
    standard_subscribers_.push_back(handle_->subscribe(
      descriptor.topic_name.c_str(), descriptor.message_queue_size, callback_wrapper));
    publishers_[descriptor.topic_name] = publisher;
    return true;
  };
  installers_.insert({KINESIS_STREAM_INPUT_TYPE_REKOGNITION_ENABLED_KINESIS_VIDEO_FRAME,
                      rekognition_kinesis_video_frame_setup_closure});
  return true;
}

void RosStreamSubscriptionInstaller::Uninstall(const std::string & topic_name)
{
  if (topic_name.empty()) {
    return;
  }
  for (auto subscriber = standard_subscribers_.begin(); subscriber != standard_subscribers_.end();
       subscriber++) {
    if (subscriber->getTopic() == topic_name) {
      subscriber->shutdown();
      if (0 < publishers_.count(topic_name)) {
        publishers_.at(topic_name).shutdown();
      }
    }
  }
  for (auto subscriber = image_transport_subscribers_.begin();
       subscriber != image_transport_subscribers_.end(); subscriber++) {
    if (subscriber->getTopic() == topic_name) {
      subscriber->shutdown();
    }
  }
}

}  // namespace Kinesis
}  // namespace Aws
