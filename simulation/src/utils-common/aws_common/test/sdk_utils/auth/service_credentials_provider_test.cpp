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
// #include <gmock/gmock.h>
#include <aws_common/sdk_utils/parameter_reader_mock.h>
#include <aws_common/sdk_utils/auth/service_credentials_provider.h>
#include <aws/core/Aws.h>
#include <aws/core/utils/json/JsonSerializer.h>

using namespace Aws::Client;
using namespace Aws::Utils;
using namespace Aws::Auth;
using namespace Aws::Utils::Json;
using ::testing::_;
using ::testing::Matcher;
using ::testing::DoAll;
using ::testing::SetArgReferee;
using ::testing::Return;
using Aws::AwsError;


class ServiceCredentialsProviderFixture : public ::testing::Test
{
public:
  static const std::map<std::string, std::string> kFullIotConfigMap;
  static const std::list<std::string> kFullIotConfigMandatoryKeys;
  static const IotRoleConfig kFullIotConfig; 
  static const std::map<std::string, std::string> kFullCredentialsInfo;
  static const std::list<std::string> kFullCredentialsInfoKeys;

protected:
  std::shared_ptr<ParameterReaderMock> param_reader_ =
   std::make_shared<ParameterReaderMock>();
};

const std::map<std::string, std::string> ServiceCredentialsProviderFixture::kFullIotConfigMap = {
  {"cafile", "M2M1NTA0NTQxMDg4YTUxMzcyMzY4MTNh"}, {"certfile", "MmQ2NWEyZmFmNThlOWM1"}, 
  {"keyfile", "MDAxZmZiY2VjYmIwMGM"}, {"endpoint", "xNDhkYzU5MjNm"},
  {"role", "YmIwNTMx"}, {"thing_name", "MzExMTdlMGI2YzY5ZjJmYTli"}, 
  {"connect_timeout_ms", "42"}, {"total_timeout_ms", "27"}
};

const std::list<std::string> ServiceCredentialsProviderFixture::kFullIotConfigMandatoryKeys = {
  "cafile", "certfile", "keyfile", "endpoint", "role", "thing_name"
};

const IotRoleConfig ServiceCredentialsProviderFixture::kFullIotConfig = IotRoleConfig(
  kFullIotConfigMap.at("cafile").c_str(),
  kFullIotConfigMap.at("certfile").c_str(),
  kFullIotConfigMap.at("keyfile").c_str(),
  kFullIotConfigMap.at("endpoint").c_str(),
  kFullIotConfigMap.at("role").c_str(),
  kFullIotConfigMap.at("thing_name").c_str(),
  StringUtils::ConvertToInt32(kFullIotConfigMap.at("connect_timeout_ms").c_str()),
  StringUtils::ConvertToInt32(kFullIotConfigMap.at("total_timeout_ms").c_str())
);

const std::map<std::string, std::string> ServiceCredentialsProviderFixture::kFullCredentialsInfo = {
  {"expiration", "2019-01-10T21:57:06Z"},
  {"accessKeyId", "ZWM2ODYzNDEwZWJhNGM0NjZiYzk4ZDI4"},
  {"secretAccessKey", "YWYyNWM0NmEzZWE1NWQy"},
  {"sessionToken", "YTFhM2NhNjM5OGZlMDlmYmRmMTY3Mzk5WQyNDVkMTJjYThi"}
};

const std::list<std::string> ServiceCredentialsProviderFixture::kFullCredentialsInfoKeys = {
  "expiration", "accessKeyId", "secretAccessKey", "sessionToken"
};

TEST_F(ServiceCredentialsProviderFixture, TestGetServiceAuthConfigNoIotConfig)
{
  EXPECT_CALL(*param_reader_, ReadParam(_, Matcher<std::map<std::string, std::string> &>(_)))
    .WillRepeatedly(Return(AwsError::AWS_ERR_NOT_FOUND));

  ServiceAuthConfig config; 
  bool success = GetServiceAuthConfig(config, param_reader_);
  
  EXPECT_FALSE(success);
}

class TestGetServiceAuthConfigFixture : 
  public ServiceCredentialsProviderFixture,
  public ::testing::WithParamInterface<std::string> {};

TEST_P(TestGetServiceAuthConfigFixture, TestGetServiceAuthConfigPartialIotConfig)
{
  auto missing_config_key = GetParam();
  auto partial_iot_config = std::map<std::string, std::string>(kFullIotConfigMap);
  partial_iot_config.erase(missing_config_key);
  EXPECT_CALL(*param_reader_, ReadParam(_, Matcher<std::map<std::string, std::string> &>(_)))
    .WillRepeatedly(DoAll(SetArgReferee<1>(partial_iot_config), Return(AwsError::AWS_ERR_OK)));

  ServiceAuthConfig config; 
  bool success = GetServiceAuthConfig(config, param_reader_);

  EXPECT_FALSE(success);
}

INSTANTIATE_TEST_CASE_P(
  TestGetServiceAuthConfigPartialIotConfig,
  TestGetServiceAuthConfigFixture,
  ::testing::ValuesIn(ServiceCredentialsProviderFixture::kFullIotConfigMandatoryKeys)
);

TEST_F(ServiceCredentialsProviderFixture, TestGetServiceAuthConfigCompleteIotConfig)
{
  EXPECT_CALL(*param_reader_, ReadParam(_, Matcher<std::map<std::string, std::string> &>(_)))
    .WillRepeatedly(DoAll(SetArgReferee<1>(kFullIotConfigMap), Return(AwsError::AWS_ERR_OK)));

  ServiceAuthConfig config; 
  bool success = GetServiceAuthConfig(config, param_reader_);
  
  EXPECT_TRUE(success);
  EXPECT_STREQ(kFullIotConfigMap.at("cafile").c_str(), config.iot.cafile.c_str());
  EXPECT_STREQ(kFullIotConfigMap.at("certfile").c_str(), config.iot.certfile.c_str());
  EXPECT_STREQ(kFullIotConfigMap.at("keyfile").c_str(), config.iot.keyfile.c_str());
  EXPECT_STREQ(kFullIotConfigMap.at("endpoint").c_str(), config.iot.host.c_str());
  EXPECT_STREQ(kFullIotConfigMap.at("role").c_str(), config.iot.role.c_str());
  EXPECT_STREQ(kFullIotConfigMap.at("thing_name").c_str(), config.iot.name.c_str());
  EXPECT_EQ(StringUtils::ConvertToInt32(kFullIotConfigMap.at("connect_timeout_ms").c_str()), 
    config.iot.connect_timeout_ms);
  EXPECT_EQ(StringUtils::ConvertToInt32(kFullIotConfigMap.at("total_timeout_ms").c_str()), 
    config.iot.total_timeout_ms);
}

TEST_F(ServiceCredentialsProviderFixture, TestServiceCredentialsProviderChainValidIotConf)
{
  ServiceCredentialsProviderChain default_conf_chain;
  ServiceCredentialsProviderChain configured_chain(ServiceAuthConfig{kFullIotConfig});

  // a new credential provider is added to the chain
  EXPECT_EQ(default_conf_chain.GetProviders().size() + 1, configured_chain.GetProviders().size());
}

TEST_F(ServiceCredentialsProviderFixture, TestServiceCredentialsProviderChainInvalidIotConf)
{
  ServiceAuthConfig config = ServiceAuthConfig{kFullIotConfig};
  config.iot.cafile = "";

  ServiceCredentialsProviderChain default_conf_chain;
  ServiceCredentialsProviderChain configured_chain(config);

  // no new credential provider is added to the chain
  EXPECT_EQ(default_conf_chain.GetProviders().size(), configured_chain.GetProviders().size());
}

class OpenIotRoleCredentialsProvider : public IotRoleCredentialsProvider
{
public:
  OpenIotRoleCredentialsProvider(const IotRoleConfig & config): IotRoleCredentialsProvider(config) {};

  void PublicRefresh() { IotRoleCredentialsProvider::Refresh();  }
  void PublicSetCredentials(AWSCredentials & creds) { IotRoleCredentialsProvider::SetCredentials(creds);  }
  bool PublicValidateResponse(Aws::Utils::Json::JsonValue & value) {
    return IotRoleCredentialsProvider::ValidateResponse(value);
  }

  Aws::Auth::AWSCredentials GetCachedCredentials() { return this->cached_; }
};

TEST_F(ServiceCredentialsProviderFixture, TestIotRoleCredentialsProviderRefreshWrongHost)
{
  auto provider = std::make_shared<OpenIotRoleCredentialsProvider>(kFullIotConfig);
  AWSCredentials initial_credentials(provider->GetAWSCredentials());
  
  provider->PublicRefresh();

  // credentials are not changed if the request to get new credentials fails
  EXPECT_EQ(initial_credentials, provider->GetAWSCredentials());
}

TEST_F(ServiceCredentialsProviderFixture, TestIotRoleCredentialsProviderSetCredentials)
{
  auto provider = std::make_shared<OpenIotRoleCredentialsProvider>(kFullIotConfig);
  AWSCredentials aws_credentials{"ZWM2ODYzNDEwZWJhNGM0NjZiYzk4ZDI4", 
    "YWYyNWM0NmEzZWE1NWQy", "YTFhM2NhNjM5OGZlMDlmYmRmMTY3Mzk5WQyNDVkMTJjYThi"};

  provider->PublicSetCredentials(aws_credentials);

  EXPECT_EQ(aws_credentials, provider->GetAWSCredentials());
}

TEST_F(ServiceCredentialsProviderFixture, TestIotRoleCredentialsValidateResponse)
{
  auto provider = std::make_shared<OpenIotRoleCredentialsProvider>(kFullIotConfig);

  Json::JsonValue malformed_json(Aws::String("malformed"));
  EXPECT_FALSE(provider->PublicValidateResponse(malformed_json));

  auto response = Json::JsonValue();
  EXPECT_FALSE(provider->PublicValidateResponse(response));

  EXPECT_FALSE(provider->PublicValidateResponse(response.WithString("credentials", Aws::String("foo"))));

  auto credentials = Json::JsonValue();
  EXPECT_FALSE(provider->PublicValidateResponse(response.WithObject("credentials", credentials)));

  credentials = credentials.WithString("expiration", Aws::String("2019-01-10T21:57:06Z"));
  EXPECT_FALSE(provider->PublicValidateResponse(response.WithObject("credentials", credentials)));

  credentials = credentials.WithString("accessKeyId", Aws::String("ZWM2ODYzNDEwZWJhNGM0NjZiYzk4ZDI4"));
  EXPECT_FALSE(provider->PublicValidateResponse(response.WithObject("credentials", credentials)));

  credentials = credentials.WithString("secretAccessKey", Aws::String("YWYyNWM0NmEzZWE1NWQy"));
  EXPECT_FALSE(provider->PublicValidateResponse(response.WithObject("credentials", credentials)));

  credentials = credentials.WithString("sessionToken", Aws::String("YTFhM2NhNjM5OGZlMDlmYmRmMTY3Mzk5WQyNDVkMTJjYThi"));
  EXPECT_TRUE(provider->PublicValidateResponse(response.WithObject("credentials", credentials)));
}

int main(int argc, char ** argv)
{
  Aws::SDKOptions options;
  Aws::InitAPI(options);

  testing::InitGoogleTest(&argc, argv);
  auto test_result = RUN_ALL_TESTS();

  Aws::ShutdownAPI(options);

  return test_result;
}
