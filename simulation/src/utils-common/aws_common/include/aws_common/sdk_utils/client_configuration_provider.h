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
#include <aws_common/sdk_utils/parameter_reader.h>

namespace Aws {
namespace Client {

bool operator==(const ClientConfiguration & left, const ClientConfiguration & right);
bool operator!=(const ClientConfiguration & left, const ClientConfiguration & right);

/**
 * Generic provider that uses a ParameterReader to create the client configuration.
 */
class ClientConfigurationProvider
{
public:
  /**
   *  @brief Creates a new ClientConfigurationProvider that uses the provided reader for loading
   * configuraiton.
   *
   *  @param reader A ParameterReaderInterface implementation that the provider will use to read the
   * configuration
   */
  // NOLINTNEXTLINE(google-explicit-constructor, hicpp-explicit-conversions)
  ClientConfigurationProvider(std::shared_ptr<ParameterReaderInterface> reader);
  ~ClientConfigurationProvider() = default;

  /**
   * Creates a client configuration with data read by the given parameter reader.
   * @param ros_version_override @refitem PopulateUserAgent
   * @return ClientConfiguration object
   */
  ClientConfiguration GetClientConfiguration(const std::string & ros_version_override = "");

private:
  std::shared_ptr<ParameterReaderInterface> reader_;
  /**
   * Populates the user agent parameter with ROS distro & version information.
   * @note if the user_agent provided is empty, OS & compiler information would be added as well.
   * Otherwise, only ROS distro & version information would be added.
   * @param user_agent
   * @param ros_version_override optional - some ROS distributions export ROS_VERSION_MAJOR,
   * ROS_VERSION_MINOR, ROS_VERSION_PATCH from their common.h. These would be more accurate and
   * specific than relying on CMAKE_ROS_VERSION, but need to be passed in from upstream.
   */
  void PopulateUserAgent(Aws::String & user_agent, const std::string & ros_version_override = "");
};

}  // namespace Client
}  // namespace Aws
