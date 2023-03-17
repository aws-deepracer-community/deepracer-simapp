/*
 * Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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
#include <aws/core/client/AWSError.h>
#include <aws/core/utils/logging/LogMacros.h>

namespace Aws {
namespace Kinesis {

/**
 * Reads the parameter using its key and namespaces. If the parameter is not found, the output
 * will be set to the default_value specified
 * 
 * @tparam T the type of the option that is being read and stored.
 * @param parameter_reader
 * @param parameter_namespaces
 * @param option_key
 * @param default_value
 * @param[out] option_value
 */
template <typename T>
void ReadOption(
  const std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
  const std::vector<std::string> & parameter_namespaces,
  const std::string & option_key,
  const T & default_value,
  T & option_value)
{
  // append the option_key to a copy of parameter_namespaces vector
  std::vector<std::string> parameter_path_keys(parameter_namespaces);
  parameter_path_keys.push_back(option_key);

  Aws::AwsError ret = parameter_reader->ReadParam(Aws::Client::ParameterPath(
    {}, parameter_path_keys), option_value);

  switch (ret) {
    case Aws::AwsError::AWS_ERR_OK:
      AWS_LOGSTREAM_INFO(__func__, option_key << " is set to: " << option_value);
      break;
    case Aws::AwsError::AWS_ERR_NOT_FOUND:
      option_value = default_value;
      AWS_LOGSTREAM_INFO(__func__,
        option_key << " parameter not found, setting to default value: " << default_value);
      break;
    default:
      option_value = default_value;
      AWS_LOGSTREAM_ERROR(__func__,
        "Error " << ret << " retrieving option " << option_key 
          << ", setting to default value: " << default_value);
      break;
  }
}

} // namespace Kinesis
} // namespace Aws
