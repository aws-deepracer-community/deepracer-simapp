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

#include <aws/core/utils/logging/AWSLogging.h>
#include <aws_common/sdk_utils/logging/aws_log_system.h>

#include <cstdarg>
#include <iostream>
#include <mutex>


namespace Aws {
namespace Utils {
namespace Logging {


AWSLogSystem::AWSLogSystem(Aws::Utils::Logging::LogLevel log_level)
: configured_log_level_(log_level)
{
}

Aws::Utils::Logging::LogLevel AWSLogSystem::GetLogLevel() const
{
  return configured_log_level_;
}

void AWSLogSystem::SetLogLevel(Aws::Utils::Logging::LogLevel log_level)
{
  configured_log_level_.store(log_level);
}

// NOLINTNEXTLINE(cert-dcl50-cpp)
void AWSLogSystem::Log(Aws::Utils::Logging::LogLevel log_level, const char * tag,
                       const char * format, ...)
{
  // Do not log for all log_levels greater than the configured log level.
  //
  // Aws::Utils::Logging::LogLevel values are here -
  // https://sdk.amazonaws.com/cpp/api/0.14.3/aws-cpp-sdk-core_2include_2aws_2core_2utils_2logging_2_log_level_8h_source.html
  if (log_level > configured_log_level_) {
    return;
  }

  // Max log line size DEFAULT_LOG_MESSAGE_SIZE_BYTES(1024) bytes.
  char buf[DEFAULT_LOG_MESSAGE_SIZE_BYTES];

  va_list argptr;
  va_start(argptr, format);
  vsnprintf(buf, sizeof(buf), format, argptr);
  va_end(argptr);

  const std::string s(buf);
  LogMessage(log_level, tag, s);
}

void AWSLogSystem::LogStream(Aws::Utils::Logging::LogLevel log_level, const char * tag,
                             const Aws::OStringStream & message_stream)
{
  if (log_level > configured_log_level_) {
    return;
  }

  LogMessage(log_level, tag, message_stream.rdbuf()->str().c_str());
}

void AWSLogSystem::LogMessage(Aws::Utils::Logging::LogLevel log_level, const char * tag,
                              const std::string & message)
{
  const char * normalized_tag = (tag == nullptr) ? "" : tag;
  switch (log_level) {
    case Aws::Utils::Logging::LogLevel::Info:
      LogInfo(normalized_tag, message);
      break;
    case Aws::Utils::Logging::LogLevel::Debug:
      LogDebug(normalized_tag, message);
      break;
    case Aws::Utils::Logging::LogLevel::Warn:
      LogWarn(normalized_tag, message);
      break;
    case Aws::Utils::Logging::LogLevel::Error:
      LogError(normalized_tag, message);
      break;
    case Aws::Utils::Logging::LogLevel::Fatal:
      LogFatal(normalized_tag, message);
      break;
    case Aws::Utils::Logging::LogLevel::Trace:
      LogTrace(normalized_tag, message);
      break;
    default:
      LogError(normalized_tag, message);
      break;
  }
}

}  // namespace Logging
}  // namespace Utils
}  // namespace Aws
