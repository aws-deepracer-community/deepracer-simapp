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
#include <aws/core/Version.h>
#include <aws/core/client/DefaultRetryStrategy.h>
#include <aws/core/platform/OSVersionInfo.h>
#include <aws_common/sdk_utils/aws_profile_provider.h>
#include <aws_common/sdk_utils/client_configuration_provider.h>
#include <aws_common/sdk_utils/parameter_reader.h>

#include <utility>

#ifndef CLIENT_CONFIG_PREFIX
#define CLIENT_CONFIG_PREFIX "aws_client_configuration"
#endif

namespace Aws {
namespace Client {

/**
 *
 */
class NoRetryStrategy : public RetryStrategy
{
public:
  NoRetryStrategy() = default;

  // NOLINTNEXTLINE(google-runtime-int)
  bool ShouldRetry(const AWSError<CoreErrors> & error, long attemptedRetries) const override
  {
    AWS_UNREFERENCED_PARAM(error);
    AWS_UNREFERENCED_PARAM(attemptedRetries);
    return false;
  }

  // NOLINTNEXTLINE(google-runtime-int)
  long CalculateDelayBeforeNextRetry(const AWSError<CoreErrors> & error, long attemptedRetries) const override
  {
    AWS_UNREFERENCED_PARAM(error);
    AWS_UNREFERENCED_PARAM(attemptedRetries);
    return 0;
  }
};


bool operator==(const ClientConfiguration & left, const ClientConfiguration & right)
{
  bool result = true;

  result &= (right.region == left.region);
  result &= (right.userAgent == left.userAgent);
  result &= (right.endpointOverride == left.endpointOverride);
  result &= (right.proxyHost == left.proxyHost);
  result &= (right.proxyUserName == left.proxyUserName);
  result &= (right.proxyPassword == left.proxyPassword);
  result &= (right.caPath == left.caPath);
  result &= (right.caFile == left.caFile);

  result &= (left.requestTimeoutMs == right.requestTimeoutMs);
  result &= (left.connectTimeoutMs == right.connectTimeoutMs);
  result &= (left.maxConnections == right.maxConnections);
  result &= (left.proxyPort == right.proxyPort);
  result &= (left.useDualStack == right.useDualStack);
  result &= (left.enableClockSkewAdjustment == right.enableClockSkewAdjustment);
  result &= (left.followRedirects == right.followRedirects);
  result &= (left.verifySSL == right.verifySSL);

  return result;
}

bool operator!=(const ClientConfiguration & left, const ClientConfiguration & right)
{
  return !(left == right);
}

ClientConfigurationProvider::ClientConfigurationProvider(
  std::shared_ptr<ParameterReaderInterface> reader)
{
  this->reader_ = std::move(reader);
}

void ClientConfigurationProvider::PopulateUserAgent(Aws::String & user_agent,
                                                    const std::string & ros_version_override)
{
  Aws::String ros_user_agent_suffix = " exec-env/AWS_RoboMaker ros-" CMAKE_ROS_DISTRO "/";
  if (ros_version_override.empty()) {
    ros_user_agent_suffix += CMAKE_ROS_VERSION;
  } else {
    ros_user_agent_suffix += ros_version_override.c_str();
  }
  user_agent += ros_user_agent_suffix;
}

ClientConfiguration ClientConfigurationProvider::GetClientConfiguration(
  const std::string & ros_version_override)
{
  ClientConfiguration config;
  Aws::Config::AWSProfileProvider profile_provider;
  /**
   * Give priority to region parameter from the parameter reader if exists, otherwise use the AWS
   * SDK/CLI config file (defaults to ~/.aws/config). The latter is needed because unlike
   * credentials, region does not get loaded automatically from the profile by the SDK (it simply
   * defaults to us-east-1).
   */
  config.region = profile_provider.GetProfile().GetRegion();
  reader_->ReadParam(ParameterPath(CLIENT_CONFIG_PREFIX, "region"), config.region);
  PopulateUserAgent(config.userAgent, ros_version_override);
  reader_->ReadParam(ParameterPath(CLIENT_CONFIG_PREFIX, "user_agent"), config.userAgent);
  reader_->ReadParam(ParameterPath(CLIENT_CONFIG_PREFIX, "endpoint_override"), config.endpointOverride);
  reader_->ReadParam(ParameterPath(CLIENT_CONFIG_PREFIX, "proxy_host"), config.proxyHost);
  reader_->ReadParam(ParameterPath(CLIENT_CONFIG_PREFIX, "proxy_user_name"), config.proxyUserName);
  reader_->ReadParam(ParameterPath(CLIENT_CONFIG_PREFIX, "proxy_password"), config.proxyPassword);
  reader_->ReadParam(ParameterPath(CLIENT_CONFIG_PREFIX, "ca_path"), config.caPath);
  reader_->ReadParam(ParameterPath(CLIENT_CONFIG_PREFIX, "ca_file"), config.caFile);

  int temp;
  if (AWS_ERR_OK == reader_->ReadParam(ParameterPath(CLIENT_CONFIG_PREFIX, "request_timeout_ms"), temp)) {
    config.requestTimeoutMs = temp;
  }
  if (AWS_ERR_OK == reader_->ReadParam(ParameterPath(CLIENT_CONFIG_PREFIX, "connect_timeout_ms"), temp)) {
    config.connectTimeoutMs = temp;
  }
  if (AWS_ERR_OK == reader_->ReadParam(ParameterPath(CLIENT_CONFIG_PREFIX, "max_connections"), temp)) {
    config.maxConnections = temp;
  }
  if (AWS_ERR_OK == reader_->ReadParam(ParameterPath(CLIENT_CONFIG_PREFIX, "proxy_port"), temp)) {
    config.proxyPort = temp;
  }

  reader_->ReadParam(ParameterPath(CLIENT_CONFIG_PREFIX, "use_dual_stack"), config.useDualStack);
  reader_->ReadParam(ParameterPath(CLIENT_CONFIG_PREFIX, "enable_clock_skew_adjustment"),
                    config.enableClockSkewAdjustment);
  bool followRedirects;
  reader_->ReadParam(ParameterPath(CLIENT_CONFIG_PREFIX, "follow_redirects"), followRedirects);
  config.followRedirects = followRedirects ? Aws::Client::FollowRedirectsPolicy::ALWAYS : Aws::Client::FollowRedirectsPolicy::NEVER;
  reader_->ReadParam(ParameterPath(CLIENT_CONFIG_PREFIX, "verify_SSL"), config.verifySSL);

  // check for non-default strategy
  bool strategy = false;
  auto error = reader_->ReadParam(ParameterPath(CLIENT_CONFIG_PREFIX, "no_retry_strategy"), strategy);

  if (AWS_ERR_OK == error && strategy) {
    config.retryStrategy = std::make_shared<NoRetryStrategy>();
  } else {
    // if max retries is set use the DefaultRetryStrategy
    int max_retries;
    if (AWS_ERR_OK == reader_->ReadParam(ParameterPath(CLIENT_CONFIG_PREFIX, "max_retries"), max_retries)) {
      config.retryStrategy = std::make_shared<Aws::Client::DefaultRetryStrategy>(max_retries);
    }
  }

  return config;
}

}  // namespace Client
}  // namespace Aws
