/*
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <aws_common/sdk_utils/parameter_reader.h>
#include <aws_ros2_common/sdk_utils/ros2_node_parameter_reader.h>
#include <kinesis_webrtc_streamer/streamer_params.h>
#include "../src/read_option.h"

using namespace Aws::Kinesis;
using namespace testing;
using ::testing::Return;
using ::testing::SaveArg;

class StrictMockParameterReader : public Aws::Client::ParameterReaderInterface
{
public:
  StrictMockParameterReader() = default;
  virtual ~StrictMockParameterReader() = default;
  
  MOCK_CONST_METHOD2(
    ReadParam,
    Aws::AwsError(const Aws::Client::ParameterPath & param_path, int & out)
  );
  MOCK_CONST_METHOD2(
    ReadParam,
    Aws::AwsError(const Aws::Client::ParameterPath & param_path, std::string & out)
  );
  MOCK_CONST_METHOD2(
    ReadParam,
    Aws::AwsError(const Aws::Client::ParameterPath & param_path, double & out)
  );
  MOCK_CONST_METHOD2(
    ReadParam,
    Aws::AwsError(const Aws::Client::ParameterPath & param_path, bool & out)
  );
  MOCK_CONST_METHOD2(
    ReadParam,
    Aws::AwsError(const Aws::Client::ParameterPath & param_path, std::vector<std::string> & out)
  );
  MOCK_CONST_METHOD2(
    ReadParam,
    Aws::AwsError(const Aws::Client::ParameterPath & param_path, std::map<std::string, std::string> & out)
  );
};

class StreamerParamsTestFixture : public ::testing::Test
{
protected:
  std::shared_ptr<StrictMockParameterReader> mock_parameter_reader_;
  
  void SetUp() override
  {
    mock_parameter_reader_ = std::make_shared<StrictMockParameterReader>();
  }

  void TearDown() override
  {
    mock_parameter_reader_.reset();
  }
};

TEST_F(StreamerParamsTestFixture, ReadOptionString)
{
  std::string result_value;
  std::string default_value("test_default_value");
  std::string found_value("test_found_value");
  std::string option_key_value("test_option_key");
  std::vector<std::string> test_namespace({"test_namespace"});

  EXPECT_CALL(*mock_parameter_reader_, ReadParam(_, Matcher<std::string &>(_)))
  .Times(3)
  .WillOnce(Return(Aws::AwsError::AWS_ERR_FAILURE))
  .WillOnce(Return(Aws::AwsError::AWS_ERR_NOT_FOUND))
  .WillOnce(DoAll(SetArgReferee<1>(found_value), Return(Aws::AwsError::AWS_ERR_OK)));

  ReadOption<std::string>(
    mock_parameter_reader_,
    test_namespace,
    option_key_value,
    default_value,
    result_value
  );
  EXPECT_EQ(default_value, result_value);

  ReadOption<std::string>(
    mock_parameter_reader_,
    test_namespace,
    option_key_value,
    default_value,
    result_value
  );
  EXPECT_EQ(default_value, result_value);

  ReadOption<std::string>(
    mock_parameter_reader_,
    test_namespace,
    option_key_value,
    default_value,
    result_value
  );
  EXPECT_EQ(found_value, result_value);
}

TEST_F(StreamerParamsTestFixture, ReadOptionSizeT)
{
  int result_value;
  const int default_value = 0;
  const int found_value = 1;
  std::string option_key_value("test_option_key");
  std::vector<std::string> test_namespace({"test_namespace"});

  EXPECT_CALL(*mock_parameter_reader_, ReadParam(_, Matcher<int &>(_)))
  .Times(3)
  .WillOnce(Return(Aws::AwsError::AWS_ERR_FAILURE))
  .WillOnce(Return(Aws::AwsError::AWS_ERR_NOT_FOUND))
  .WillOnce(DoAll(SetArgReferee<1>(found_value), Return(Aws::AwsError::AWS_ERR_OK)));

  ReadOption<int>(
    mock_parameter_reader_,
    test_namespace,
    option_key_value,
    default_value,
    result_value
  );
  EXPECT_EQ(default_value, result_value);

  ReadOption<int>(
    mock_parameter_reader_,
    test_namespace,
    option_key_value,
    default_value,
    result_value
  );
  EXPECT_EQ(default_value, result_value);

  ReadOption<int>(
    mock_parameter_reader_,
    test_namespace,
    option_key_value,
    default_value,
    result_value
  );
  EXPECT_EQ(found_value, result_value);
}

TEST_F(StreamerParamsTestFixture, ReadStreamerParamsDefaults)
{
  const int stream_count = 2;
  const int params_with_sizet = 1;
  const int params_with_string = 4;
  const int expected_calls_with_sizet = stream_count * params_with_sizet + 1;
  const int expected_calls_with_string = stream_count * params_with_string;

  ON_CALL(*mock_parameter_reader_, ReadParam(_, Matcher<std::string &>(_)))
    .WillByDefault(Return(Aws::AwsError::AWS_ERR_NOT_FOUND));
  ON_CALL(*mock_parameter_reader_, ReadParam(_, Matcher<int &>(_)))
    .WillByDefault(Return(Aws::AwsError::AWS_ERR_NOT_FOUND));

  EXPECT_CALL(*mock_parameter_reader_, ReadParam(_, Matcher<std::string &>(_)))
    .Times(expected_calls_with_string);
  EXPECT_CALL(*mock_parameter_reader_, ReadParam(_, Matcher<int &>(_)))
    .Times(expected_calls_with_sizet)
    .WillOnce(DoAll(SetArgReferee<1>(stream_count), Return(Aws::AwsError::AWS_ERR_OK)));

  std::vector<StreamerParams> params;
  ReadStreamerParams(mock_parameter_reader_, params);

  for (StreamerParams streamer_params : params) {
    EXPECT_EQ(kSignalingChannelNameDefault, streamer_params.signaling_channel_name_);
    EXPECT_EQ(kVideoSubscriptionTopicDefault, streamer_params.video_subscription_topic_);
    EXPECT_EQ(kDataSubscriptionTopicDefault, streamer_params.data_subscription_topic_);
    EXPECT_EQ(kPublishTopicDefault, streamer_params.publish_topic_);
    EXPECT_EQ(kVideoFormatTypeDefault, streamer_params.video_format_type_);
  } 
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
