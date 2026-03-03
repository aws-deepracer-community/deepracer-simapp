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
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace Aws {
namespace Utils {
namespace Logging {

class AWSROSLogger : public AWSLogSystem
{

public:
    /**
     * @param log_level Defaults to Trace. This log level is an additional layer on top of ROS' log filtering.
     *  Typically, you would instantiate this with the lowest (most permissive) log level (i.e. Trace), and control the log level via ROS.
     * @param node
     */
    explicit AWSROSLogger(Aws::Utils::Logging::LogLevel log_level, std::weak_ptr<rclcpp::Node> node);
    AWSROSLogger(AWSROSLogger const &) = delete;              // Do not allow copy constructor
    AWSROSLogger & operator=(AWSROSLogger const &) = delete;  // Do not allow assignment operator
    ~AWSROSLogger() override;

    // Add the Flush method implementation
    void Flush() override {
        // No-op implementation since ROS2 logger handles its own flushing
    }
    
    // Implement the pure virtual function from LogSystemInterface
    // The actual logging is handled by the AWSLogSystem class
    void vaLog(Aws::Utils::Logging::LogLevel logLevel, const char* tag, const char* formatStr, va_list args) override {
        char message[DEFAULT_LOG_MESSAGE_SIZE_BYTES];
        vsnprintf(message, DEFAULT_LOG_MESSAGE_SIZE_BYTES, formatStr, args);
        
        std::string messageStr(message);
        
        switch (logLevel) {
            case Aws::Utils::Logging::LogLevel::Info:
                LogInfo(tag, messageStr);
                break;
            case Aws::Utils::Logging::LogLevel::Debug:
                LogDebug(tag, messageStr);
                break;
            case Aws::Utils::Logging::LogLevel::Trace:
                LogTrace(tag, messageStr);
                break;
            case Aws::Utils::Logging::LogLevel::Warn:
                LogWarn(tag, messageStr);
                break;
            case Aws::Utils::Logging::LogLevel::Error:
                LogError(tag, messageStr);
                break;
            case Aws::Utils::Logging::LogLevel::Fatal:
                LogFatal(tag, messageStr);
                break;
            default:
                break;
        }
    }

protected:

    void LogInfo(const char* tag, const std::string& message)  override;
    void LogTrace(const char* tag, const std::string& message) override;
    void LogDebug(const char* tag, const std::string& message) override;
    void LogWarn(const char* tag, const std::string& message)  override;
    void LogError(const char* tag, const std::string& message) override;
    void LogFatal(const char* tag, const std::string& message) override;

private:
    std::weak_ptr<rclcpp::Node> node_;
};

}  // namespace Logging
}  // namespace Utils
}  // namespace Aws

#endif  // AWS_COMMON_INCLUDE_SDK_UTILS_LOGGING_AWSROSLOGGER_H_
