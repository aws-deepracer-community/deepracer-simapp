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

#define PARAM_READER_TEST__PARAM_PREFIX "configuration_namespace9181"
#define PARAM_READER_TEST__PARAM_KEY "someBogusParamKey"
#define PARAM_READER_TEST__PARAM_VALUE "uk-north-2180"
using namespace Aws::Client;

class ParameterReaderTest : public ::testing::Test {
protected:
    void SetUp() override {
        producer = ros::NodeHandle("~producers");
        producer.setParam(PARAM_READER_TEST__PARAM_PREFIX "/" PARAM_READER_TEST__PARAM_KEY, PARAM_READER_TEST__PARAM_VALUE);
    }
    void TearDown() {
        producer.deleteParam(PARAM_READER_TEST__PARAM_PREFIX "/" PARAM_READER_TEST__PARAM_KEY);
        consumer.deleteParam(PARAM_READER_TEST__PARAM_PREFIX "/" PARAM_READER_TEST__PARAM_KEY);
    }
    ros::NodeHandle producer;
    ros::NodeHandle consumer;
};

TEST_F(ParameterReaderTest, parameterPathResolution)
{
    auto parameter_reader = std::make_shared<Aws::Client::Ros1NodeParameterReader>();
    auto param_path_flat = ParameterPath(
        "test_parameter_reader/producers/" PARAM_READER_TEST__PARAM_PREFIX "/" PARAM_READER_TEST__PARAM_KEY
    );
    auto param_path_variadic = ParameterPath(
        "test_parameter_reader", "producers", PARAM_READER_TEST__PARAM_PREFIX, PARAM_READER_TEST__PARAM_KEY
    );
    auto param_path_complex_no_node_ns = ParameterPath(std::vector<std::string>{},
            std::vector<std::string>{PARAM_READER_TEST__PARAM_PREFIX, PARAM_READER_TEST__PARAM_KEY});
    auto param_path_complex_with_node_ns = ParameterPath(
            std::vector<std::string>{"test_parameter_reader", "producers"},
            std::vector<std::string>{PARAM_READER_TEST__PARAM_PREFIX, PARAM_READER_TEST__PARAM_KEY});

    ASSERT_EQ(param_path_flat.get_resolved_path('/', '/'), param_path_variadic.get_resolved_path('/', '/'));
    ASSERT_EQ(param_path_flat.get_resolved_path('/', '/'), param_path_complex_with_node_ns.get_resolved_path('/', '/'));

    std::string flat_result;
    parameter_reader->ReadParam(param_path_flat, flat_result);
    std::string variadic_result;
    parameter_reader->ReadParam(param_path_variadic, variadic_result);
    std::string complex_no_node_ns_result;
    parameter_reader->ReadParam(param_path_complex_no_node_ns, complex_no_node_ns_result);
    std::string complex_with_node_ns_result;
    parameter_reader->ReadParam(param_path_complex_with_node_ns, complex_with_node_ns_result);

    ASSERT_EQ(flat_result, variadic_result);
    ASSERT_EQ(variadic_result, complex_with_node_ns_result);
    ASSERT_EQ(flat_result, PARAM_READER_TEST__PARAM_VALUE);

    ASSERT_EQ(std::string(), complex_no_node_ns_result);
    consumer.setParam(PARAM_READER_TEST__PARAM_PREFIX "/" PARAM_READER_TEST__PARAM_KEY, PARAM_READER_TEST__PARAM_VALUE);
    parameter_reader->ReadParam(param_path_complex_no_node_ns, complex_no_node_ns_result);
    ASSERT_EQ(complex_no_node_ns_result, complex_with_node_ns_result);
}

TEST(ParameterReader, failureTests)
{
    auto parameter_reader = std::make_shared<Aws::Client::Ros1NodeParameterReader>();
    auto nonexistent_path = ParameterPath("I don't exist");
    std::string nonexistent_path_result = PARAM_READER_TEST__PARAM_VALUE;
    /* Querying for a nonexistent parameter should return NOT_FOUND and the out parameter remains unchanged. */
    ASSERT_EQ(Aws::AwsError::AWS_ERR_NOT_FOUND, parameter_reader->ReadParam(nonexistent_path, nonexistent_path_result));
    ASSERT_EQ(nonexistent_path_result, std::string(PARAM_READER_TEST__PARAM_VALUE));
}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_parameter_reader");
    return RUN_ALL_TESTS();
}
