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

#pragma once

#include <aws/core/utils/logging/LogLevel.h>
#include <aws/core/utils/logging/LogSystemInterface.h>

#include <atomic>
#include <string>

#define DEFAULT_LOG_MESSAGE_SIZE_BYTES 1024

namespace Aws {
namespace Utils {
namespace Logging {

class AWSLogSystem : public Aws::Utils::Logging::LogSystemInterface
{
public:
  explicit AWSLogSystem(Aws::Utils::Logging::LogLevel log_level);
  AWSLogSystem(AWSLogSystem const &) = delete;              // Do not allow copy constructor
  AWSLogSystem & operator=(AWSLogSystem const &) = delete;  // Do not allow assignment operator
  ~AWSLogSystem() override = default;

  // Gets the currently configured log level.
  Aws::Utils::Logging::LogLevel GetLogLevel() const override;

  // Set a new log level. This has the immediate effect of changing the log.
  void SetLogLevel(Aws::Utils::Logging::LogLevel log_level);

  // Log using printf style output. Prefer using LogStream
  void Log(Aws::Utils::Logging::LogLevel log_level, const char * tag, const char * format,
                   ...) override;

  // Logs from a streambuffer.
  void LogStream(Aws::Utils::Logging::LogLevel log_level, const char * tag,
                         const Aws::OStringStream & message_stream) override;

protected:
  virtual void LogInfo(const char * tag, const std::string & message) = 0;
  virtual void LogDebug(const char * tag, const std::string & message) = 0;
  virtual void LogTrace(const char * tag, const std::string & message) = 0;
  virtual void LogWarn(const char * tag, const std::string & message) = 0;
  virtual void LogError(const char * tag, const std::string & message) = 0;
  virtual void LogFatal(const char * tag, const std::string & message) = 0;

private:
  std::atomic<Aws::Utils::Logging::LogLevel> configured_log_level_;

  void LogMessage(Aws::Utils::Logging::LogLevel log_level, const char * tag,
                  const std::string & message);
};

}  // namespace Logging
}  // namespace Utils
}  // namespace Aws
