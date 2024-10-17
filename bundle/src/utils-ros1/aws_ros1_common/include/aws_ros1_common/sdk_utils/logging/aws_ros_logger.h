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

#ifndef AWS_COMMON_INCLUDE_SDK_UTILS_LOGGING_AWSROSLOGGER_H_
#define AWS_COMMON_INCLUDE_SDK_UTILS_LOGGING_AWSROSLOGGER_H_

#include <aws_common/sdk_utils/logging/aws_log_system.h>

#include <string>

namespace Aws {
namespace Utils {
namespace Logging {

class AWSROSLogger : public AWSLogSystem
{
public:
  /**
   * @param log_level Defaults to Trace. This log level is an additional layer on top of ROS' log
   * filtering. Typically, you would instantiate this with the lowest (most permissive) log level
   * (i.e. Trace), and control the log level via ROS.
   */
  explicit AWSROSLogger(Aws::Utils::Logging::LogLevel log_level = LogLevel::Trace);
  AWSROSLogger(AWSROSLogger const &) = delete;              // Do not allow copy constructor
  AWSROSLogger & operator=(AWSROSLogger const &) = delete;  // Do not allow assignment operator
  ~AWSROSLogger() override = default;
  virtual void Flush();

protected:
  void LogTrace(const char * tag, const std::string & message) override;
  void LogInfo(const char * tag, const std::string & message) override;
  void LogDebug(const char * tag, const std::string & message) override;
  void LogWarn(const char * tag, const std::string & message) override;
  void LogError(const char * tag, const std::string & message) override;
  void LogFatal(const char * tag, const std::string & message) override;
};

}  // namespace Logging
}  // namespace Utils
}  // namespace Aws

#endif  // AWS_COMMON_INCLUDE_SDK_UTILS_LOGGING_AWSROSLOGGER_H_
