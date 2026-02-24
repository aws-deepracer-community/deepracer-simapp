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

#include <aws_ros1_common/sdk_utils/logging/aws_ros_logger.h>
#include <ros/ros.h>


namespace Aws {
namespace Utils {
namespace Logging {

AWSROSLogger::AWSROSLogger(Aws::Utils::Logging::LogLevel log_level) : AWSLogSystem(log_level) {}

void AWSROSLogger::Flush() {}

void AWSROSLogger::LogTrace(const char * tag, const std::string & message)
{
  ROS_DEBUG("[%s] %s", tag, message.c_str());
}

void AWSROSLogger::LogInfo(const char * tag, const std::string & message)
{
  ROS_INFO("[%s] %s", tag, message.c_str());
}

void AWSROSLogger::LogDebug(const char * tag, const std::string & message)
{
  ROS_DEBUG("[%s] %s", tag, message.c_str());
}

void AWSROSLogger::LogWarn(const char * tag, const std::string & message)
{
  ROS_WARN("[%s] %s", tag, message.c_str());
}

void AWSROSLogger::LogError(const char * tag, const std::string & message)
{
  ROS_ERROR("[%s] %s", tag, message.c_str());
}

void AWSROSLogger::LogFatal(const char * tag, const std::string & message)
{
  ROS_FATAL("[%s] %s", tag, message.c_str());
}

}  // namespace Logging
}  // namespace Utils
}  // namespace Aws
