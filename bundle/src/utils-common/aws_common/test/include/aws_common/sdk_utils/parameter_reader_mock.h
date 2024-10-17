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
#include <gmock/gmock.h>

namespace Aws {
namespace Client {

class ParameterReaderMock : public ParameterReaderInterface 
{
public:
  MOCK_CONST_METHOD2(ReadParam, Aws::AwsError(const ParameterPath &, std::vector<std::string> &));
  MOCK_CONST_METHOD2(ReadParam, Aws::AwsError(const ParameterPath &, double &));
  MOCK_CONST_METHOD2(ReadParam, Aws::AwsError(const ParameterPath &, int &));
  MOCK_CONST_METHOD2(ReadParam, Aws::AwsError(const ParameterPath &, bool &));
  MOCK_CONST_METHOD2(ReadParam, Aws::AwsError(const ParameterPath &, Aws::String &));
  MOCK_CONST_METHOD2(ReadParam, Aws::AwsError(const ParameterPath &, std::string &));
  MOCK_CONST_METHOD2(ReadParam, Aws::AwsError(const ParameterPath &, std::map<std::string, std::string> &));
};

}  // namespace Client
}  // namespace Aws
