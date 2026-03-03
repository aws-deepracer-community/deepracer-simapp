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

#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <aws_ros2_common/sdk_utils/logging/aws_ros_logger.h>


namespace Aws {
namespace Utils {
namespace Logging {

AWSROSLogger::AWSROSLogger(Aws::Utils::Logging::LogLevel log_level, std::weak_ptr<rclcpp::Node> node)
    : node_(std::move(node)), AWSLogSystem(log_level) {}

AWSROSLogger::~AWSROSLogger() = default;

void
AWSROSLogger::LogInfo(const char* tag, const std::string& message) {
    if (auto node = node_.lock()) {
        RCLCPP_INFO(node->get_logger(), "[%s] %s", tag, message.c_str());
    }
}

void
AWSROSLogger::LogTrace(const char* tag, const std::string& message)
{
    if (auto node = node_.lock()) {
        RCLCPP_DEBUG(node->get_logger(), "[%s] %s", tag, message.c_str());
    }
}

void
AWSROSLogger::LogDebug(const char* tag, const std::string& message) {
    if (auto node = node_.lock()) {
        RCLCPP_DEBUG(node->get_logger(), "[%s] %s", tag, message.c_str());
    }
}

void
AWSROSLogger::LogWarn(const char* tag, const std::string& message) {
    if (auto node = node_.lock()) {
        RCLCPP_WARN(node->get_logger(), "[%s] %s", tag, message.c_str());
    }
}

void
AWSROSLogger::LogError(const char* tag, const std::string& message) {
    if (auto node = node_.lock()) {
        RCLCPP_ERROR(node->get_logger(), "[%s] %s", tag, message.c_str());
    }
}

void
AWSROSLogger::LogFatal(const char* tag, const std::string& message) {
    if (auto node = node_.lock()) {
        RCLCPP_FATAL(node->get_logger(), "[%s] %s", tag, message.c_str());
    }
}

}  // namespace Logging
}  // namespace Utils
}  // namespace Aws
