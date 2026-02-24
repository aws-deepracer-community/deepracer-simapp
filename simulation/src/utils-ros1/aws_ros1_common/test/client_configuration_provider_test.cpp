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
#include <aws/core/client/ClientConfiguration.h>
#include <aws_common/sdk_utils/client_configuration_provider.h>
#include <aws_ros1_common/sdk_utils/ros1_node_parameter_reader.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

#define CLIENT_CONFIG_PREFIX "aws_client_configuration"
#define USER_AGENT_SUFFIX "/user_agent"

/**
 * Populates a ROS node and a client configuration with the same dummy values.
 * @param node
 * @param config
 */
void InitializeNodeAndConfig(ros::NodeHandle & node, Aws::Client::ClientConfiguration & config)
{
  node.setParam(CLIENT_CONFIG_PREFIX "/region", "uk-north-20");
  config.region = "uk-north-20";
  node.setParam(CLIENT_CONFIG_PREFIX "/proxy_port", 787);
  config.proxyPort = 787;
  node.setParam(CLIENT_CONFIG_PREFIX "/connect_timeout_ms", 511111);
  config.connectTimeoutMs = 511111;
  node.setParam(CLIENT_CONFIG_PREFIX "/verify_SSL", true);
  config.verifySSL = true;
  node.setParam(CLIENT_CONFIG_PREFIX "/follow_redirects", true);
  config.followRedirects = Aws::Client::FollowRedirectsPolicy::ALWAYS;
}

/**
 * Tests that GetClientConfiguration returns the expected ClientConfiguration object.
 */
TEST(DefaultClientConfigurationProvider, getClientConfiguration)
{
  Aws::Client::ClientConfiguration prepared_config;
  ros::NodeHandle dummy_node;

  InitializeNodeAndConfig(dummy_node, prepared_config);
  auto parameter_reader = std::make_shared<Aws::Client::Ros1NodeParameterReader>();
  Aws::Client::ClientConfigurationProvider config_provider(parameter_reader);
  Aws::Client::ClientConfiguration generated_config = config_provider.GetClientConfiguration();
  prepared_config.userAgent =
    generated_config
      .userAgent; /* Set the user agent to w/e was generated. Will be tested separately */

  prepared_config.followRedirects = Aws::Client::FollowRedirectsPolicy::NEVER;
  ASSERT_NE(prepared_config, generated_config);

  prepared_config.followRedirects = Aws::Client::FollowRedirectsPolicy::ALWAYS;
  ASSERT_EQ(prepared_config, generated_config);
}

/**
 * Tests that the configuration provider sets userAgent correctly with the ROS distro & version
 * information.
 */
TEST(DefaultClientConfigurationProvider, userAgentTest)
{
  Aws::Client::ClientConfiguration prepared_config;
  ros::NodeHandle dummy_node;
  InitializeNodeAndConfig(dummy_node, prepared_config);
  auto parameter_reader = std::make_shared<Aws::Client::Ros1NodeParameterReader>();
  Aws::Client::ClientConfigurationProvider config_provider(parameter_reader);

  /* Verify userAgent starts with "ros-<distro>" */
  std::vector<std::string> ros1_distros = {"indigo", "jade", "kinetic", "lunar", "melodic"};
  Aws::Client::ClientConfiguration generated_config = config_provider.GetClientConfiguration();
  size_t ros_user_agent_index = std::string::npos;
  for (const auto & distro : ros1_distros) {
    ros_user_agent_index =
      generated_config.userAgent.find(std::string("exec-env/AWS_RoboMaker ros-" + distro).c_str());
    if (std::string::npos != ros_user_agent_index) {
      break;
    }
  }
  ASSERT_NE(std::string::npos, ros_user_agent_index);

  /* Test version override */
  const char version_override[] = "1.12.16";
  generated_config = config_provider.GetClientConfiguration(version_override);
  ASSERT_EQ(0, generated_config.userAgent
                 .substr(generated_config.userAgent.size() - (sizeof(version_override) - 1),
                         sizeof(version_override) - 1)
                 .compare(version_override));

  /* Verify that we only add information on top of the SDK's userAgent */
  Aws::Client::ClientConfiguration stock_config;
  ASSERT_EQ(
    0,
    generated_config.userAgent.substr(0, ros_user_agent_index - 1).compare(stock_config.userAgent));

  /* UserAgent override using ROS parameter */
  prepared_config.userAgent = "my-user-agent";
  std::string stored_user_agent;
  if (dummy_node.hasParam(CLIENT_CONFIG_PREFIX USER_AGENT_SUFFIX)) {
    dummy_node.getParam(CLIENT_CONFIG_PREFIX USER_AGENT_SUFFIX, stored_user_agent);
  }
  dummy_node.setParam(CLIENT_CONFIG_PREFIX USER_AGENT_SUFFIX, prepared_config.userAgent.c_str());
  generated_config = config_provider.GetClientConfiguration();
  ASSERT_EQ(prepared_config, generated_config);
  if (!stored_user_agent.empty()) {
    dummy_node.setParam(CLIENT_CONFIG_PREFIX USER_AGENT_SUFFIX, stored_user_agent);
  } else {
    dummy_node.deleteParam(CLIENT_CONFIG_PREFIX USER_AGENT_SUFFIX);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_configuration_provider");
  return RUN_ALL_TESTS();
}
