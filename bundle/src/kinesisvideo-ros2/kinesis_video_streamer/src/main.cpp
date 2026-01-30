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
#include <aws_ros2_common/sdk_utils/logging/aws_ros_logger.h>
#include <aws_ros2_common/sdk_utils/ros2_node_parameter_reader.h>
#include <kinesis_video_streamer/ros_stream_subscription_installer.h>
#include <kinesis_video_streamer/streamer.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/executor_options.hpp>


#ifndef RETURN_CODE_MASK
#define RETURN_CODE_MASK (0xff) /* Process exit code is in range (0, 255) */
#endif

#ifndef UNKNOWN_ERROR_KINESIS_VIDEO_EXIT_CODE
#define UNKNOWN_ERROR_KINESIS_VIDEO_EXIT_CODE (0xf0)
#endif

using namespace Aws::Client;
using namespace Aws::Kinesis;

constexpr char kNodeName[] = "kinesis_video_streamer";
const char * kSpinnerThreadCountOverrideParameter = "spinner_thread_count";

int main(int argc, char * argv[])
{
    int return_code = UNKNOWN_ERROR_KINESIS_VIDEO_EXIT_CODE;

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    node_options.automatically_declare_parameters_from_overrides(true);
    auto streamer = std::make_shared<StreamerNode>(kNodeName, std::string(), node_options);
    auto parameter_reader = std::make_shared<Aws::Client::Ros2NodeParameterReader>(streamer);
    auto subscription_installer = std::make_shared<RosStreamSubscriptionInstaller>(streamer);

    Aws::Utils::Logging::InitializeAWSLogging(Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>(
            kNodeName, Aws::Utils::Logging::LogLevel::Warn, streamer));
    Aws::SDKOptions options;
    Aws::InitAPI(options);

    KinesisManagerStatus status = streamer->Initialize(parameter_reader, subscription_installer);
    if (!KINESIS_MANAGER_STATUS_SUCCEEDED(status)) {
        return_code = status;
        goto shutdown_;
    }

    status = streamer->InitializeStreamSubscriptions();
    if (KINESIS_MANAGER_STATUS_SUCCEEDED(status)) {
        AWS_LOG_INFO(__func__, "Starting Kinesis Video Node...");
        uint32_t spinner_thread_count = kDefaultNumberOfSpinnerThreads;
        int spinner_thread_count_input;
        if (Aws::AwsError::AWS_ERR_OK ==
            parameter_reader->ReadParam(ParameterPath(kSpinnerThreadCountOverrideParameter),
                                         spinner_thread_count_input)) {
            spinner_thread_count = static_cast<uint32_t>(spinner_thread_count_input);
        }
        // Use SingleThreadedExecutor for video streaming (sequential processing)
        rclcpp::executors::SingleThreadedExecutor spinner;
        spinner.add_node(streamer);
        spinner.spin();
    } else {
        return_code = status;
        goto shutdown_;
    }

shutdown_:
    AWS_LOG_INFO(__func__, "Shutting down Kinesis Video Node...");
    Aws::Utils::Logging::ShutdownAWSLogging();
    Aws::ShutdownAPI(options);
    return return_code & RETURN_CODE_MASK;
}
