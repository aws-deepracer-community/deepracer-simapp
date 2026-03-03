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

#include <aws_common/sdk_utils/aws_error.h>
#include <aws_common/sdk_utils/parameter_reader.h>
#include <aws_ros2_common/sdk_utils/ros2_node_parameter_reader.h>

using namespace Aws::Client;

#define TEST_PARAM_PREFIX "test_params"

/**
 * Test basic parameter reading functionality
 */
TEST(ParameterReader, basicParameterReading)
{
    // Create a node and set some parameters
    auto node = rclcpp::Node::make_shared("param_test_node");
    
    // Set up parameters of different types
    node->declare_parameter(TEST_PARAM_PREFIX ".string_param", "");
    node->declare_parameter(TEST_PARAM_PREFIX ".int_param", 0);
    node->declare_parameter(TEST_PARAM_PREFIX ".double_param", 0.0);
    node->declare_parameter(TEST_PARAM_PREFIX ".bool_param", false);
    
    node->set_parameter(rclcpp::Parameter(TEST_PARAM_PREFIX ".string_param", "test_string"));
    node->set_parameter(rclcpp::Parameter(TEST_PARAM_PREFIX ".int_param", 42));
    node->set_parameter(rclcpp::Parameter(TEST_PARAM_PREFIX ".double_param", 3.14));
    node->set_parameter(rclcpp::Parameter(TEST_PARAM_PREFIX ".bool_param", true));
    
    // Create parameter reader
    auto parameter_reader = std::make_shared<Ros2NodeParameterReader>(node);
    
    // Test reading string parameter
    std::string string_value;
    ASSERT_EQ(Aws::AwsError::AWS_ERR_OK, 
              parameter_reader->ReadParam(ParameterPath(TEST_PARAM_PREFIX, "string_param"), string_value));
    ASSERT_EQ("test_string", string_value);
    
    // Test reading int parameter
    int int_value;
    ASSERT_EQ(Aws::AwsError::AWS_ERR_OK, 
              parameter_reader->ReadParam(ParameterPath(TEST_PARAM_PREFIX, "int_param"), int_value));
    ASSERT_EQ(42, int_value);
    
    // Test reading double parameter
    double double_value;
    ASSERT_EQ(Aws::AwsError::AWS_ERR_OK, 
              parameter_reader->ReadParam(ParameterPath(TEST_PARAM_PREFIX, "double_param"), double_value));
    ASSERT_DOUBLE_EQ(3.14, double_value);
    
    // Test reading bool parameter
    bool bool_value;
    ASSERT_EQ(Aws::AwsError::AWS_ERR_OK, 
              parameter_reader->ReadParam(ParameterPath(TEST_PARAM_PREFIX, "bool_param"), bool_value));
    ASSERT_TRUE(bool_value);
}

/**
 * Test error handling for non-existent parameters
 */
TEST(ParameterReader, nonExistentParameter)
{
    auto node = rclcpp::Node::make_shared("param_test_node");
    auto parameter_reader = std::make_shared<Ros2NodeParameterReader>(node);
    
    // Try to read a parameter that doesn't exist
    std::string value;
    ASSERT_EQ(Aws::AwsError::AWS_ERR_NOT_FOUND, 
              parameter_reader->ReadParam(ParameterPath("non_existent_param"), value));
    
    // Value should remain unchanged
    ASSERT_EQ("", value);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}