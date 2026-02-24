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
#include <aws/core/Aws.h>
#include <aws_common/sdk_utils/parameter_reader_mock.h>
#include <aws_common/sdk_utils/client_configuration_provider.h>

using namespace Aws::Client;
using ::testing::_;
using ::testing::Matcher;
using ::testing::DoAll;
using ::testing::SetArgReferee;
using ::testing::Return;
using Aws::AwsError;

class ClientConfigurationProviderFixture : public ::testing::Test
{
protected:
  std::shared_ptr<ParameterReaderMock> param_reader_ =
    std::make_shared<ParameterReaderMock>();
  std::shared_ptr<ClientConfigurationProvider> test_subject_ = 
    std::make_shared<ClientConfigurationProvider>(param_reader_);
};

TEST_F(ClientConfigurationProviderFixture, TestReaderRespected)
{
  const char* expected_str = "N2IxYjk2ZDhkYTNmZWQ2ZjkwMDlhNWRl";
  const int kExpectedInt = 64654;
  const bool kExpectedBool = true;

  EXPECT_CALL(*param_reader_, ReadParam(_, Matcher<std::vector<std::string> &>(_)))
   .WillRepeatedly(Return(AwsError::AWS_ERR_OK));
  EXPECT_CALL(*param_reader_, ReadParam(_, Matcher<double &>(_)))
   .WillRepeatedly(Return(AwsError::AWS_ERR_OK));
  EXPECT_CALL(*param_reader_, ReadParam(_, Matcher<std::string &>(_)))
   .WillRepeatedly(Return(AwsError::AWS_ERR_OK));
  EXPECT_CALL(*param_reader_, ReadParam(_, Matcher<Aws::String &>(_)))
    .WillRepeatedly(DoAll(SetArgReferee<1>(expected_str), Return(AwsError::AWS_ERR_OK)));
  EXPECT_CALL(*param_reader_, ReadParam(_, Matcher<int &>(_)))
    .WillRepeatedly(DoAll(SetArgReferee<1>(kExpectedInt), Return(AwsError::AWS_ERR_OK)));
  EXPECT_CALL(*param_reader_, ReadParam(_, Matcher<bool &>(_)))
    .WillRepeatedly(DoAll(SetArgReferee<1>(kExpectedBool), Return(AwsError::AWS_ERR_OK)));

  ClientConfiguration config = test_subject_->GetClientConfiguration();

  EXPECT_EQ(expected_str, config.region);
  EXPECT_EQ(kExpectedInt, config.maxConnections);
  EXPECT_EQ(kExpectedBool, config.useDualStack);
}

TEST_F(ClientConfigurationProviderFixture, TestAllReaderErrorsIgnored) 
{
  ClientConfiguration default_config;
  const char* unexpected_str = (default_config.region + Aws::String("YjJiOWZmZDQ3Yjg1MTU0MmE3MjFmOTk2")).c_str();
  const int unkExpectedInt = default_config.maxConnections + 45231;
  const bool unkExpectedBool = !default_config.useDualStack;

  EXPECT_CALL(*param_reader_, ReadParam(_, Matcher<Aws::String &>(_)))
    .WillRepeatedly(DoAll(SetArgReferee<1>(unexpected_str), Return(AwsError::AWS_ERR_FAILURE)));
  EXPECT_CALL(*param_reader_, ReadParam(_, Matcher<int &>(_)))
    .WillRepeatedly(DoAll(SetArgReferee<1>(unkExpectedInt), Return(AwsError::AWS_ERR_NOT_FOUND)));
  EXPECT_CALL(*param_reader_, ReadParam(_, Matcher<bool &>(_)))
    .WillRepeatedly(DoAll(SetArgReferee<1>(unkExpectedBool), Return(AwsError::AWS_ERR_EMPTY)));

  ClientConfiguration config = test_subject_->GetClientConfiguration();

  EXPECT_EQ(unexpected_str, config.region);
  EXPECT_NE(unkExpectedInt, config.maxConnections);
  EXPECT_EQ(unkExpectedBool, config.useDualStack);
  EXPECT_EQ(default_config.maxConnections, config.maxConnections);
  EXPECT_NE(default_config.useDualStack, config.useDualStack);
}

TEST_F(ClientConfigurationProviderFixture, TestRosVersionOverride) {
  EXPECT_CALL(*param_reader_, ReadParam(_, Matcher<Aws::String &>(_)))
    .WillRepeatedly(Return(AwsError::AWS_ERR_NOT_FOUND));
  EXPECT_CALL(*param_reader_, ReadParam(_, Matcher<int &>(_)))
    .WillRepeatedly(Return(AwsError::AWS_ERR_NOT_FOUND));
  EXPECT_CALL(*param_reader_, ReadParam(_, Matcher<bool &>(_)))
    .WillRepeatedly(Return(AwsError::AWS_ERR_NOT_FOUND));

  ClientConfiguration config1 = test_subject_->GetClientConfiguration();
  ClientConfiguration config2 = test_subject_->GetClientConfiguration("another_version");
  EXPECT_NE(config1, config2);
  config1.userAgent = config2.userAgent;
  EXPECT_EQ(config1, config2);
}

using client_conf_mutator = std::function<void(ClientConfiguration &)>;
class ClientConfigurationOperatorsFixture :
 public ::testing::TestWithParam<client_conf_mutator> { };

TEST_P(ClientConfigurationOperatorsFixture, TestClientConfigurationEqNeqOperators)
{
  ClientConfiguration config1, config2;

  EXPECT_EQ(config1, config2);

  GetParam()(config1);
  EXPECT_NE(config1, config2);
}

client_conf_mutator g_client_conf_mutators[] = 
{
  [](ClientConfiguration & config)->void { config.region = "OTg5ZWRiMWVhYjEyZDRkMWFhOTRlZGU0"; },
  [](ClientConfiguration & config)->void { config.userAgent = "YmVkYjJjYTA0NTg0ZGJjZDdjOWY2OWYz"; },
  [](ClientConfiguration & config)->void { config.endpointOverride = "MzllMmUxZWUzMmE3OWQ1MzZhZGQ0ZWM2"; },
  [](ClientConfiguration & config)->void { config.proxyHost = "NzgzNzY5NDM5NTNjYmEzNGU0YmY3YjQ4"; },
  [](ClientConfiguration & config)->void { config.proxyUserName = "NmU2MWE2NjhlMGEwYTc1NjU1ZDU0OGQw"; },
  [](ClientConfiguration & config)->void { config.proxyPassword = "ZThkYmM1NmQwMWU1MTU1MDQ4YTA5NDFk"; },
  [](ClientConfiguration & config)->void { config.caPath = "ZWM4Mzk1MTFiM2MyOTE2NzA0MThjMTYw"; },
  [](ClientConfiguration & config)->void { config.caFile = "YzU5OGQzNmYzNWIzMjIzNDFlZDM3NGNh"; },
  [](ClientConfiguration & config)->void { config.requestTimeoutMs = config.requestTimeoutMs + 12; },
  [](ClientConfiguration & config)->void { config.connectTimeoutMs = config.connectTimeoutMs - 42; },
  [](ClientConfiguration & config)->void { config.maxConnections = config.maxConnections + 1981; },
  [](ClientConfiguration & config)->void { config.proxyPort = config.proxyPort + 2; },
  [](ClientConfiguration & config)->void { config.useDualStack =  ! config.useDualStack; },
  [](ClientConfiguration & config)->void { config.enableClockSkewAdjustment = ! config.enableClockSkewAdjustment; },
  [](ClientConfiguration & config)->void { config.followRedirects = (config.followRedirects == FollowRedirectsPolicy::NEVER ? FollowRedirectsPolicy::ALWAYS : FollowRedirectsPolicy::NEVER); },
  [](ClientConfiguration & config)->void { config.verifySSL = !config.verifySSL; }
};
INSTANTIATE_TEST_CASE_P(
  TestClientConfigurationOperators, 
  ClientConfigurationOperatorsFixture,  
  ::testing::ValuesIn(g_client_conf_mutators)
);

int main(int argc, char ** argv)
{
  Aws::SDKOptions options;
  Aws::InitAPI(options);
  testing::InitGoogleTest(&argc, argv);
  auto ret = RUN_ALL_TESTS();
  Aws::ShutdownAPI(options);
  return ret;
}
