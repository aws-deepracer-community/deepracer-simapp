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

#include <aws_ros1_common/sdk_utils/ros1_node_parameter_reader.h>
#include <ros/ros.h>

namespace Aws {
namespace Client {

constexpr char kParameterNsSeparator = '/';
constexpr char kNodeNsSeparator = '/';


template <class T>
static AwsError ReadParamTemplate(const ParameterPath & param_path, T & out)
{
  std::string name = param_path.get_resolved_path(kNodeNsSeparator, kParameterNsSeparator);
  std::string key;
  if (ros::param::search(name, key) && ros::param::get(key, out)) {
    return AWS_ERR_OK;
  }
  return AWS_ERR_NOT_FOUND;
}

AwsError Ros1NodeParameterReader::ReadParam(const ParameterPath & param_path, std::vector<std::string> & out) const
{
  return ReadParamTemplate(param_path, out);
}

AwsError Ros1NodeParameterReader::ReadParam(const ParameterPath & param_path, double & out) const
{
  return ReadParamTemplate(param_path, out);
}

AwsError Ros1NodeParameterReader::ReadParam(const ParameterPath & param_path, int & out) const
{
  return ReadParamTemplate(param_path, out);
}

AwsError Ros1NodeParameterReader::ReadParam(const ParameterPath & param_path, bool & out) const
{
  return ReadParamTemplate(param_path, out);
}

AwsError Ros1NodeParameterReader::ReadParam(const ParameterPath & param_path, std::string & out) const
{
  return ReadParamTemplate(param_path, out);
}

AwsError Ros1NodeParameterReader::ReadParam(const ParameterPath & param_path, Aws::String & out) const
{
  std::string value;
  AwsError result = ReadParam(param_path, value);
  if (result == AWS_ERR_OK) {
    out = Aws::String(value.c_str()); // NOLINT(readability-redundant-string-cstr)
  }
  return result;
}

AwsError Ros1NodeParameterReader::ReadParam(const ParameterPath & param_path,
                                          std::map<std::string, std::string> & out) const
{
  return ReadParamTemplate(param_path, out);
}

} // namespace Client
} // namespace Aws
