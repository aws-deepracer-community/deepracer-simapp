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

#include <aws/core/auth/AWSCredentialsProvider.h>
#include <aws/core/config/AWSProfileConfigLoader.h>

namespace Aws {
namespace Config {
/**
 * AWSCredentialsProvider does not expose the Profile nor its ProfileConfigLoader, and so we
 * implement a Profile provider which borrows most of its logic from AWSCredentialsProvider.
 */
class AWSProfileProvider : Aws::Auth::ProfileConfigFileAWSCredentialsProvider
{
public:
  AWSProfileProvider();
  const Aws::Config::Profile GetProfile();

private:
  std::shared_ptr<AWSProfileConfigLoader> config_file_loader_;
  std::shared_ptr<AWSProfileConfigLoader> credentials_file_loader_;
  Aws::String profile_to_use_;
};
}  // namespace Config
}  // namespace Aws
