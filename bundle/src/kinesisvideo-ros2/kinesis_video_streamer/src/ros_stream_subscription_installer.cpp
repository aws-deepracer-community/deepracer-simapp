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
#include <image_transport/subscriber.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/node.hpp>

namespace Aws {
namespace Kinesis {

bool RosStreamSubscriptionInstaller::SetupImageTransport(const ImageTransportCallbackFn callback)
{
    if (!callback) {
        AWS_LOG_ERROR(__func__, "Invalid callback was provided at SetupImageTransport");
        return false;
    }
    SubscriberSetupFn image_transport_setup_closure =
        [this, callback](const StreamSubscriptionDescriptor & descriptor) -> bool {
            auto callback_wrapper = [this, callback, descriptor]
                    (const sensor_msgs::msg::Image::ConstSharedPtr &image) -> void {
                callback(*this->stream_manager_, descriptor.stream_name, image);
            };
            if (auto node_handle = handle_.lock()) {
                image_transport::ImageTransport it(node_handle);
                image_transport_subscribers_.push_back(
                        it.subscribe(descriptor.topic_name, descriptor.message_queue_size, callback_wrapper));
            } else {
                AWS_LOG_ERROR(__func__, "Cannot set up subscription - the node handle has been destroyed.");
                return false;
            }
            return true;
        };
    installers_.insert({KINESIS_STREAM_INPUT_TYPE_IMAGE_TRANSPORT, image_transport_setup_closure});
    return true;
}

bool RosStreamSubscriptionInstaller::SetupKinesisVideoFrameTransport(
    const KinesisVideoFrameTransportCallbackFn callback)
{
    if (!callback) {
        AWS_LOG_ERROR(__func__, "Invalid callback was provided at SetupKinesisVideoFrameTransport");
        return false;
    }
    SubscriberSetupFn kinesis_video_frame_setup_closure =
        [this, callback](const StreamSubscriptionDescriptor & descriptor) -> bool {
            auto callback_wrapper = [this, callback, descriptor]
                    (const kinesis_video_msgs::msg::KinesisVideoFrame::ConstSharedPtr frame) -> void {
                callback(*this->stream_manager_, descriptor.stream_name, frame);
            };
            if (auto node_handle = handle_.lock()) {
                const auto qos = rclcpp::QoS(
                    rclcpp::KeepLast(descriptor.message_queue_size))
                    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
                    .history(rclcpp::HistoryPolicy::KeepLast)
                    .durability(rclcpp::DurabilityPolicy::Volatile);
            
                standard_subscribers_.push_back(
                    node_handle->create_subscription<kinesis_video_msgs::msg::KinesisVideoFrame>(
                        descriptor.topic_name,
                        qos,
                        callback_wrapper));
            } else {
                AWS_LOG_ERROR(__func__, "Cannot set up subscription - the node handle has been destroyed.");
                return false;
            }
            return true;
        };
    installers_.insert({KINESIS_STREAM_INPUT_TYPE_KINESIS_VIDEO_FRAME, kinesis_video_frame_setup_closure});
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
            if (auto node_handle = handle_.lock()) {
                auto publisher = node_handle->create_publisher<std_msgs::msg::String>(
                        descriptor.rekognition_topic_name, descriptor.message_queue_size);
                auto callback_wrapper = [this, callback, descriptor, publisher](
                        const kinesis_video_msgs::msg::KinesisVideoFrame::ConstSharedPtr frame) -> void {
                    callback(*this->stream_manager_, descriptor.stream_name, frame, publisher);
                };
                /* Use sensor data default QoS settings (volatile, best effort - typical for streaming) */
                const auto qos = rclcpp::QoS(
                    rclcpp::KeepLast(descriptor.message_queue_size))
                    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
                    .history(rclcpp::HistoryPolicy::KeepLast)
                    .durability(rclcpp::DurabilityPolicy::Volatile);

                standard_subscribers_.push_back(
                    node_handle->create_subscription<kinesis_video_msgs::msg::KinesisVideoFrame>(
                        descriptor.topic_name,
                        qos,
                        callback_wrapper));
                publishers_[descriptor.topic_name] = publisher;
            } else {
                AWS_LOG_ERROR(__func__, "Cannot set up subscription - the node handle has been destroyed.");
                return false;
            }
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
  // TODO: Figure out a way to get the subscribed topic name given an rclcpp::Subscription object.
}

} // namespace Kinesis
} // namespace Aws
