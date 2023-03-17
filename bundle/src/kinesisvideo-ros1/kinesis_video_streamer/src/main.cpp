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
#include <aws/core/Aws.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <aws_common/sdk_utils/aws_error.h>
#include <aws_common/sdk_utils/client_configuration_provider.h>
#include <aws_ros1_common/sdk_utils/logging/aws_ros_logger.h>
#include <aws_ros1_common/sdk_utils/ros1_node_parameter_reader.h>
#include <kinesis_video_streamer/ros_stream_subscription_installer.h>
#include <kinesis_video_streamer/streamer.h>
#include <kinesis_video_streamer/subscriber_callbacks.h>
#include <ros/ros.h>

using namespace Aws::Client;
using namespace Aws::Kinesis;


#ifndef RETURN_CODE_MASK
#define RETURN_CODE_MASK (0xff) /* Process exit code is in range (0, 255) */
#endif

#ifndef UNKNOWN_ERROR_KINESIS_VIDEO_EXIT_CODE
#define UNKNOWN_ERROR_KINESIS_VIDEO_EXIT_CODE (0xf0)
#endif

constexpr char kNodeName[] = "kinesis_video_streamer";

int shutdown(Aws::SDKOptions options, int return_code) {
  AWS_LOG_INFO(__func__, "Shutting down Kinesis Video Node...");
  Aws::Utils::Logging::ShutdownAWSLogging();
  Aws::ShutdownAPI(options);
  return return_code & RETURN_CODE_MASK;
}

int main(int argc, char * argv[])
{
  int return_code = UNKNOWN_ERROR_KINESIS_VIDEO_EXIT_CODE;

  ros::init(argc, argv, kNodeName);
  StreamerNode streamer("~");

  Aws::Utils::Logging::InitializeAWSLogging(
    Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>(kNodeName));
  Aws::SDKOptions options;
  Aws::InitAPI(options);

  KinesisManagerStatus status = streamer.Initialize();
  if (!KINESIS_MANAGER_STATUS_SUCCEEDED(status)) {
    return shutdown(options, status);
  }
  
  status = streamer.InitializeStreamSubscriptions();
  if (!KINESIS_MANAGER_STATUS_SUCCEEDED(status)) {
    return shutdown(options, status);
  }
  
  AWS_LOG_INFO(__func__, "Starting Kinesis Video Node...");
  streamer.Spin();
  
  return shutdown(options, return_code);
}
