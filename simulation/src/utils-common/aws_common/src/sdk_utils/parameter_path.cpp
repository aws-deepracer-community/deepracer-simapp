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

#include <aws_common/sdk_utils/parameter_reader.h>


namespace Aws {
namespace Client {

std::string ParameterPath::get_resolved_path(char node_namespace_separator,
                                             char parameter_namespace_separator) const
{
  std::string resolved_path = get_node_path(node_namespace_separator);
  if (!resolved_path.empty()) {
    resolved_path += node_namespace_separator;
  }
  resolved_path += get_local_path(parameter_namespace_separator);
  return resolved_path;
}

std::string ParameterPath::get_node_path(char node_namespace_separator) const
{
  std::string resolved_path;
  /* Construct the node's path by the provided lists of keys */
  for (const auto & node_namespace : node_namespaces_) {
    resolved_path += node_namespace + node_namespace_separator;
  }
  if (!resolved_path.empty() && resolved_path.back() == node_namespace_separator) {
    resolved_path.pop_back();
  }
  return resolved_path;
}

std::string ParameterPath::get_local_path(char parameter_namespace_separator) const
{
  std::string resolved_path;
  /* Construct the parameter's path by the provided lists of keys */
  for (const auto & parameter_path_key : parameter_path_keys_) {
    resolved_path += parameter_path_key + parameter_namespace_separator;
  }
  if (!resolved_path.empty() && resolved_path.back() == parameter_namespace_separator) {
    resolved_path.pop_back();
  }
  return resolved_path;
}

const ParameterPath ParameterPath::operator+(const std::string & addend) const
{
  ParameterPath result(node_namespaces_, parameter_path_keys_);
  result += addend;
  return result;
}

ParameterPath & ParameterPath::operator+=(const std::string & addend)
{
  parameter_path_keys_.push_back(addend);
  return *this;
}

}  // namespace Client
}  // namespace Aws
