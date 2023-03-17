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
#include <kinesis_video_streamer/subscriber_callbacks.h>
#include <std_msgs/String.h>

#ifndef PUTFRAME_LOG_INTERVAL_IN_SECONDS
#define PUTFRAME_LOG_INTERVAL_IN_SECONDS (10)
#endif

namespace Aws {
namespace Kinesis {

void RekognitionEnabledKinesisVideoFrameTransportCallback(
  KinesisStreamManagerInterface & stream_manager, std::string stream_name,
  const kinesis_video_msgs::KinesisVideoFrame::ConstPtr & frame_msg,
  const ros::Publisher & publisher)
{
  KinesisVideoFrameTransportCallback(stream_manager, stream_name, frame_msg);
  Aws::Vector<Model::Record> records;
  KinesisManagerStatus status = stream_manager.FetchRekognitionResults(stream_name, &records);
  if (KINESIS_MANAGER_STATUS_FAILED(status) &&
      KINESIS_MANAGER_STATUS_GET_RECORDS_THROTTLED != status) {
    AWS_LOGSTREAM_WARN(__func__, stream_name.c_str()
                                   << " FetchRekognitionResults failed. Error code: " << status);
    return;
  }
  for (auto item = records.begin(); item != records.end(); item++) {
    std_msgs::String message;
    const char * data = reinterpret_cast<char *>(item->GetData().GetUnderlyingData());
    size_t length = item->GetData().GetLength();
    message.data = std::string(data, length);
    publisher.publish(message);
  }
}

void KinesisVideoFrameTransportCallback(
  KinesisStreamManagerInterface & stream_manager, std::string stream_name,
  const kinesis_video_msgs::KinesisVideoFrame::ConstPtr & frame_msg)
{
  if (!frame_msg->codec_private_data.empty()) {
    KinesisManagerStatus update_codec_data_result =
      stream_manager.ProcessCodecPrivateDataForStream(stream_name, frame_msg->codec_private_data);
    if (KINESIS_MANAGER_STATUS_FAILED(update_codec_data_result)) {
        AWS_LOGSTREAM_WARN(
        __func__, stream_name.c_str()
                    << " failed updating codec data, error code: " << update_codec_data_result
                    << ". Continuing streaming as a best effort, but you might not be able to "
                       "decode and render the stream.");
    } else {
      ROS_DEBUG_THROTTLE(PUTFRAME_LOG_INTERVAL_IN_SECONDS,
                         "%s Updated codec data successfully. Frame index: %du",
                         stream_name.c_str(), frame_msg->index);
    }
  }

  Frame frame;
  frame.trackId = DEFAULT_TRACK_ID;
  frame.size = frame_msg->frame_data.size();
  frame.frameData = reinterpret_cast<PBYTE>((void *)(frame_msg->frame_data.data()));
  frame.duration = frame_msg->duration *
                   HUNDREDS_OF_NANOS_IN_A_MICROSECOND; /* Duration is specified in microseconds, but
                                                          Kinesis expects 100ns units. */
  frame.index = frame_msg->index;
  UINT64 generated_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                 std::chrono::system_clock::now().time_since_epoch())
                                 .count() /
                               DEFAULT_TIME_UNIT_IN_NANOS;
  frame.decodingTs = frame_msg->decoding_ts ? frame_msg->decoding_ts : generated_timestamp;
  frame.presentationTs =
    frame_msg->presentation_ts ? frame_msg->presentation_ts : generated_timestamp;
  frame.flags = (FRAME_FLAGS)frame_msg->flags;

  KinesisManagerStatus status = stream_manager.PutFrame(stream_name, frame);
  if (KINESIS_MANAGER_STATUS_FAILED(status)) {
    AWS_LOGSTREAM_WARN(__func__, stream_name.c_str() << " PutFrame failed. Error code: " << status);
  } else {
    ROS_DEBUG_THROTTLE(PUTFRAME_LOG_INTERVAL_IN_SECONDS, "%s PutFrame succeeded. Frame index: %du",
                       stream_name.c_str(), frame.index);
  }

  if (!frame_msg->metadata.empty()) {
    status = KINESIS_MANAGER_STATUS_SUCCESS;
    for (auto iter = frame_msg->metadata.begin(); iter != frame_msg->metadata.end(); ++iter) {
      status = static_cast<KinesisManagerStatus>(
        status | stream_manager.PutMetadata(stream_name, iter->key, iter->value));
    }
    if (KINESIS_MANAGER_STATUS_FAILED(status)) {
      AWS_LOGSTREAM_WARN(__func__, stream_name.c_str()
                                     << " PutMetadata failed. Error code: " << status);
    } else {
      ROS_DEBUG_THROTTLE(PUTFRAME_LOG_INTERVAL_IN_SECONDS,
                         "%s PutMetadata succeeded. Frame index: %du", stream_name.c_str(),
                         frame.index);
    }
  }
}

void ImageTransportCallback(const KinesisStreamManagerInterface & stream_manager,
                            std::string stream_name, const sensor_msgs::ImageConstPtr & image)
{
  Frame frame;
  frame.trackId = DEFAULT_TRACK_ID;
  frame.size = image->step * image->height;
  /* Overflow check (since 'size', 'step' and 'height' are all 32 bit integers). */
  if (image->step != 0 && frame.size / image->step != image->height) {
    AWS_LOGSTREAM_WARN(
      __func__,
      stream_name.c_str()
        << " Integer overflow detected - image size is too big. Aborting imageTransportCallback");
    return;
  }
  frame.frameData = reinterpret_cast<PBYTE>((void *)(image->data.data()));
  frame.duration = 0;
  frame.index = image->header.seq;
  UINT64 generated_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                 std::chrono::system_clock::now().time_since_epoch())
                                 .count() /
                               DEFAULT_TIME_UNIT_IN_NANOS;
  /* Image uses standard ROS Header type which contains a (seconds, nseconds) timestamp structure.
   * Need to convert to 100ns unit. */
  std::chrono::seconds timestamp_in_seconds(image->header.stamp.sec);
  UINT64 image_timestamp =
    (std::chrono::microseconds(timestamp_in_seconds).count() * HUNDREDS_OF_NANOS_IN_A_MICROSECOND) +
    (image->header.stamp.nsec / DEFAULT_TIME_UNIT_IN_NANOS);
  frame.decodingTs = image_timestamp ? image_timestamp : generated_timestamp;
  frame.presentationTs = image_timestamp ? image_timestamp : generated_timestamp;
  frame.flags = FRAME_FLAG_NONE;

  KinesisManagerStatus status = stream_manager.PutFrame(stream_name, frame);
  if (KINESIS_MANAGER_STATUS_FAILED(status)) {
    AWS_LOGSTREAM_WARN(__func__, stream_name.c_str() << " PutFrame failed. Error code: " << status);
  } else {
    ROS_DEBUG_THROTTLE(PUTFRAME_LOG_INTERVAL_IN_SECONDS, "%s PutFrame succeeded. Frame index: %du",
                       stream_name.c_str(), frame.index);
  }
}

}  // namespace Kinesis
}  // namespace Aws
