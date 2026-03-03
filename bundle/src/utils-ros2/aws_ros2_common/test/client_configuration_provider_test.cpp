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

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <aws/core/client/ClientConfiguration.h>

#include <aws_common/sdk_utils/client_configuration_provider.h>
#include <aws_ros2_common/sdk_utils/ros2_node_parameter_reader.h>

#define CLIENT_CONFIG_PREFIX "aws_client_configuration"

/**
 * Simplified test for ClientConfigurationProvider that only tests basic parameter reading
 * without initializing the full AWS SDK
 */
TEST(DefaultClientConfigurationProvider, basicParameterReading)
{
    // Create a node and set some parameters
    auto node = rclcpp::Node::make_shared("config_test_node");
    
    // Declare and set parameters
    node->declare_parameter(CLIENT_CONFIG_PREFIX ".test_param", "default_value");
    node->set_parameter(rclcpp::Parameter(CLIENT_CONFIG_PREFIX ".test_param", "test_value"));
    
    // Create parameter reader
    auto parameter_reader = std::make_shared<Aws::Client::Ros2NodeParameterReader>(node);
    
    // Read the parameter directly using the parameter reader
    std::string param_value;
    Aws::AwsError result = parameter_reader->ReadParam(
        Aws::Client::ParameterPath(CLIENT_CONFIG_PREFIX, "test_param"), 
        param_value);
    
    // Verify parameter was read correctly
    ASSERT_EQ(Aws::AwsError::AWS_ERR_OK, result);
    ASSERT_EQ("test_value", param_value);
}

/**
 * Test that verifies parameter path construction works correctly
 */
TEST(DefaultClientConfigurationProvider, parameterPathConstruction)
{
    // Test different ways to construct parameter paths
    Aws::Client::ParameterPath path1(CLIENT_CONFIG_PREFIX, "param_name");
    Aws::Client::ParameterPath path2(CLIENT_CONFIG_PREFIX ".param_name");
    
    // Verify paths are constructed correctly
    ASSERT_EQ(path1.get_resolved_path('.', '.'), path2.get_resolved_path('.', '.'));
    ASSERT_EQ(CLIENT_CONFIG_PREFIX ".param_name", path1.get_resolved_path('.', '.'));
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}