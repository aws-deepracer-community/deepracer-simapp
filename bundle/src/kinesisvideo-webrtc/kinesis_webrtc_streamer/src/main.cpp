/*
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <aws/core/Aws.h>
#include <aws/core/utils/logging/AWSLogging.h>
#include <aws/core/utils/logging/LogLevel.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <aws_ros2_common/sdk_utils/logging/aws_ros_logger.h>
#include <aws_ros2_common/sdk_utils/ros2_node_parameter_reader.h>
#include <rclcpp/rclcpp.hpp>

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
  rclcpp::init(argc, argv);

  Aws::SDKOptions sdk_options;
  Aws::InitAPI(sdk_options);

  auto webrtc_node = std::make_shared<Aws::Kinesis::WebRtcNode>(kNodeName);
  
  auto param_reader = std::make_shared<Aws::Client::Ros2NodeParameterReader>(webrtc_node);

  // Initialize AWS logging with the ROS logger
  auto aws_logger = Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>(
    kNodeName, 
    Aws::Utils::Logging::LogLevel::Info, 
    std::weak_ptr<rclcpp::Node>(webrtc_node)
  );
  Aws::Utils::Logging::InitializeAWSLogging(aws_logger);

  Aws::Kinesis::KinesisWebRtcManagerStatus status = webrtc_node->Initialize(param_reader);
  if (Aws::Kinesis::KinesisWebRtcManagerStatusSucceeded(status)) {
    RCLCPP_INFO(webrtc_node->get_logger(), "Successfully initialized %s", kNodeName);
    
    int spinner_thread_count;
    Aws::Kinesis::ReadOption<int>(
      param_reader,
      {Aws::Kinesis::kWebRtcStreamerNamespacePrefix},
      Aws::Kinesis::kSpinnerThreadCountKey,
      Aws::Kinesis::kSpinnerThreadCountDefault,
      spinner_thread_count
    );

    AWS_LOG_INFO(__func__, "Starting Kinesis WebRTC Node...");
    
    // Create a multithreaded executor
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), spinner_thread_count);
    executor.add_node(webrtc_node);
    executor.spin();
    
    AWS_LOG_INFO(__func__, "Shutting down Kinesis WebRTC Node...");
  } else {
    RCLCPP_ERROR(webrtc_node->get_logger(), "Failed to initialize %s", kNodeName);
  }
  return_code = static_cast<int>(status);

  rclcpp::shutdown();
  return shutdown(sdk_options, return_code);
}
