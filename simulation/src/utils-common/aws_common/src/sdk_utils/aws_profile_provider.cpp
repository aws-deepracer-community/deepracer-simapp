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
#include <aws/core/platform/Environment.h>
#include <aws_common/sdk_utils/aws_profile_provider.h>

/**
 * Constants copied from <auth/AWSCredentialsProvider.cpp>
 */
static const char * const kProfileLogTag = "ProfileConfigFileAWSCredentialsProvider";
static const char * const kAwsProfileEnvironmentVariable = "AWS_DEFAULT_PROFILE";
static const char * const kDefaultProfile = "default";

namespace Aws {
namespace Config {
AWSProfileProvider::AWSProfileProvider()
{
  config_file_loader_ =
    std::shared_ptr<AWSProfileConfigLoader>(Aws::MakeShared<AWSConfigFileProfileConfigLoader>(
      kProfileLogTag, Aws::Auth::GetConfigProfileFilename(), true));
  config_file_loader_->Load();
  credentials_file_loader_ =
    std::shared_ptr<AWSProfileConfigLoader>(Aws::MakeShared<AWSConfigFileProfileConfigLoader>(
      kProfileLogTag, GetCredentialsProfileFilename()));
  credentials_file_loader_->Load();

  auto profile_from_env = Aws::Environment::GetEnv(kAwsProfileEnvironmentVariable);
  if (!profile_from_env.empty()) {
    profile_to_use_ = profile_from_env;
  } else {
    profile_to_use_ = kDefaultProfile;
  }
}

const Profile AWSProfileProvider::GetProfile()
{
  auto credentials_file_iter = credentials_file_loader_->GetProfiles().find(profile_to_use_);
  if (credentials_file_iter != credentials_file_loader_->GetProfiles().end()) {
    return credentials_file_iter->second;
  }
  auto config_file_iter = config_file_loader_->GetProfiles().find(profile_to_use_);
  if (config_file_iter != config_file_loader_->GetProfiles().end()) {
    return config_file_iter->second;
  }
  return Profile();
}
}  // namespace Config
}  // namespace Aws
