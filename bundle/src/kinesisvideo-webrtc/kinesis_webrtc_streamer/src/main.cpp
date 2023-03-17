/*
 * Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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
#include <aws/core/utils/logging/AWSLogging.h>
#include <aws/core/utils/logging/LogLevel.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <aws_ros1_common/sdk_utils/logging/aws_ros_logger.h>
#include <aws_ros1_common/sdk_utils/ros1_node_parameter_reader.h>
#include <ros/ros.h>

#include "kinesis_webrtc_streamer/streamer_params.h"
#include "kinesis_webrtc_streamer/webrtc_node.h"
#include "read_option.h"

 #ifndef RETURN_CODE_MASK
 #define RETURN_CODE_MASK (0xff) /* Process exit code is in range (0, 255) */
 #endif
 
 #ifndef UNKNOWN_ERROR_KINESIS_WEBRTC_EXIT_CODE
 #define UNKNOWN_ERROR_KINESIS_WEBRTC_EXIT_CODE (0xf0)
 #endif

constexpr char kNodeName[] = "kinesis_webrtc_streamer";

int shutdown(Aws::SDKOptions options, int return_code) {
  AWS_LOG_INFO(__func__, "Shutting down Kinesis Video Node...");
  Aws::Utils::Logging::ShutdownAWSLogging();
  Aws::ShutdownAPI(options);
  return return_code & RETURN_CODE_MASK;
}

int main(int argc, char ** argv)
{
  int return_code = UNKNOWN_ERROR_KINESIS_WEBRTC_EXIT_CODE;
  ros::init(argc, argv, kNodeName);

  Aws::SDKOptions sdk_options;
  Aws::InitAPI(sdk_options);

  auto webrtc_node = std::make_shared<Aws::Kinesis::WebRtcNode>(kNodeName);
  
  auto param_reader = std::make_shared<Aws::Client::Ros1NodeParameterReader>();

  Aws::Utils::Logging::InitializeAWSLogging(
    Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>(kNodeName));

  Aws::Kinesis::KinesisWebRtcManagerStatus status = webrtc_node->Initialize(param_reader);
  if (Aws::Kinesis::KinesisWebRtcManagerStatusSucceeded(status)) {
    AWS_LOGSTREAM_INFO(__func__, "Successfully initialized " << kNodeName);
    int spinner_thread_count;
    Aws::Kinesis::ReadOption<int>(
      param_reader,
      {ros::this_node::getNamespace() + Aws::Kinesis::kWebRtcStreamerNamespacePrefix},
      Aws::Kinesis::kSpinnerThreadCountKey,
      Aws::Kinesis::kSpinnerThreadCountDefault,
      spinner_thread_count
    );


    ros::MultiThreadedSpinner spinner(spinner_thread_count);

    AWS_LOG_INFO(__func__, "Starting Kinesis WebRTC Node...");
    spinner.spin();
    AWS_LOG_INFO(__func__, "Shutting down Kinesis WebRTC Node...");
  } else {
    AWS_LOGSTREAM_ERROR(__func__, "Failed to initialize " << kNodeName);
  }
  return_code = static_cast<int>(status);

  return shutdown(sdk_options, return_code);
}
