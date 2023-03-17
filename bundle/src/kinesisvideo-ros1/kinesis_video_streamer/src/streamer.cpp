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

#include <log4cplus/configurator.h>

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
const char * kSpinnerThreadCountOverrideParameter = "spinner_thread_count";

namespace Aws {
namespace Kinesis {
  
StreamerNode::StreamerNode(const std::string & ns = std::string()) : ros::NodeHandle(ns)
{
  parameter_reader_ = std::make_shared<Ros1NodeParameterReader>();
  subscription_installer_ = std::make_shared<RosStreamSubscriptionInstaller>(*this);

  /* Log4cplus setup for the Kinesis Producer SDK */
  std::string log4cplus_config;
  parameter_reader_->ReadParam(
    GetKinesisVideoParameter(kStreamParameters.log4cplus_config), log4cplus_config);
  if (!log4cplus_config.empty()) {
    log4cplus::PropertyConfigurator::doConfigure(log4cplus_config);
  } else {
    log4cplus::BasicConfigurator configurator;
    configurator.configure();
  }
}

KinesisManagerStatus StreamerNode::Initialize()
{
  ClientConfigurationProvider configuration_provider(parameter_reader_);
  ClientConfiguration aws_sdk_config =
    configuration_provider.GetClientConfiguration();
  /* Set up subscription callbacks */
  if (!subscription_installer_->SetDefaultCallbacks()) {
    AWS_LOG_FATAL(__func__, "Failed to set up subscription callbacks.");
    return KINESIS_MANAGER_STATUS_ERROR_BASE;
  }
  auto kinesis_client = std::unique_ptr<KinesisClient>(
    Aws::New<Aws::Kinesis::KinesisClientFacade>(__func__, aws_sdk_config));
  stream_manager_ = std::make_shared<KinesisStreamManager>(
    parameter_reader_.get(), &stream_definition_provider_, subscription_installer_.get(),
    std::move(kinesis_client));
  subscription_installer_->set_stream_manager(stream_manager_.get());
  /* Initialization of video producer */
  KinesisManagerStatus initialize_video_producer_result =
    stream_manager_->InitializeVideoProducer(aws_sdk_config.region.c_str());
  if (KINESIS_MANAGER_STATUS_FAILED(initialize_video_producer_result)) {
    fprintf(stderr, "Failed to initialize video producer");
    AWS_LOGSTREAM_FATAL(__func__, "Failed to initialize video producer. Error code: "
                                    << initialize_video_producer_result);
    return initialize_video_producer_result;
  }
  
  return KINESIS_MANAGER_STATUS_SUCCESS;
}

KinesisManagerStatus StreamerNode::InitializeStreamSubscriptions() 
{
  /* Set up subscriptions and get ready to start streaming */
  KinesisManagerStatus streamer_setup_result = stream_manager_->KinesisVideoStreamerSetup();
  if (KINESIS_MANAGER_STATUS_SUCCEEDED(streamer_setup_result)) {
    AWS_LOG_DEBUG(__func__, "KinesisVideoStreamerSetup completed successfully.");
  } else {
    fprintf(stderr, "Failed to setup the kinesis video streamer");
    AWS_LOGSTREAM_ERROR(__func__, "KinesisVideoStreamerSetup failed with error code : "
                                    << streamer_setup_result << ". Exiting");
    return streamer_setup_result;
  }
  
  return KINESIS_MANAGER_STATUS_SUCCESS;
}

void StreamerNode::Spin()
{
  uint32_t spinner_thread_count = kDefaultNumberOfSpinnerThreads;
  int spinner_thread_count_input;
  if (Aws::AwsError::AWS_ERR_OK ==
      parameter_reader_->ReadParam(ParameterPath(kSpinnerThreadCountOverrideParameter),
                                 spinner_thread_count_input)) {
    spinner_thread_count = static_cast<uint32_t>(spinner_thread_count_input);
  }
  ros::MultiThreadedSpinner spinner(spinner_thread_count);
  spinner.spin();
}

void StreamerNode::set_subscription_installer(std::shared_ptr<RosStreamSubscriptionInstaller> subscription_installer)
{
  subscription_installer_ = subscription_installer;
}

}  // namespace Kinesis
}  // namespace Aws