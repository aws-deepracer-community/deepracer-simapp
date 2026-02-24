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

#include <aws/core/client/ClientConfiguration.h>
#include <aws/core/utils/StringUtils.h>
#include <aws_common/sdk_utils/aws_error.h>

#include <utility>


namespace Aws {
namespace Client {

class ParameterPath
{
public:
  /**
   * @example The parameter under node "lidar_node" in namespace "sensor_nodes",
   * with parameter namespace "settings" and parameter key "frequency", should be specified as follows:
   *   node_namespace: ["sensor_nodes", "lidar_node"]
   *   parameter_path_keys: ["settings", "frequency"]
   * @note Node namespaces should be empty if used by a node looking for a parameter of its own (a local parameter).
   * @param node_namespaces
   * @param parameter_path_keys
   */
  explicit ParameterPath(const std::vector<std::string> & node_namespaces,
                         const std::vector<std::string> & parameter_path_keys) :
    node_namespaces_(node_namespaces), parameter_path_keys_(parameter_path_keys) {}

  /**
   * @example The local parameter "timeout" in parameter namespace "config" should be specified as follows:
   *   ParameterPath("config", "timeout")
   * @tparam Args
   * @param parameter_path_keys
   */
  template<typename ...Args>
  explicit ParameterPath(Args... parameter_path_keys) :
    parameter_path_keys_(std::vector<std::string> {(parameter_path_keys)...}) {}

  /**
   * Appends string as another parameter path component (not nodename namespace component!)
   * @param addend new parameter path component
   * @return new ParameterPath object with the parameter path component appended
   */
  const ParameterPath operator+(const std::string & addend) const;

  /**
   * Appends string as another parameter path component (not nodename namespace component!)
   * @param addend new parameter path component
   * @return current ParameterPath object with the parameter path component appended
   */
  ParameterPath & operator+=(const std::string & addend);

  /**
   * Checks for equivalence between two ParameterPath objects
   * @param addend new parameter path component
   * @return current ParameterPath object with the parameter path component appended
   */
  bool operator==(const ParameterPath & other) const
  {
    return other.node_namespaces_ == node_namespaces_ && other.parameter_path_keys_ == parameter_path_keys_;
  }

  /**
   * Resolves & returns the parameter's path. Supports two separate use cases:
   *  1. A single, 'flat' string was provided upon construction.
   *    In this case, we return it as is.
   *  2. Detailed lists of strings were specified for the parameter & node namespaces.
   *    In this case, we construct the resolved path using the provided separators.
   * @param node_namespace_separator
   * @param parameter_namespace_separator
   * @return string representing the full, resolved path of the parameter.
   * @note If node_namespaces and parameter_path_keys are empty, an empty string would be returned.
   */
  std::string get_resolved_path(char node_namespace_separator,
                                char parameter_namespace_separator) const;

private:
  /**
   * @note Only applies if node_namespaces was specified during construction; otherwise, an empty string is returned.
   * @param node_namespace_separator
   * @return string The parameter's whereabouts in the node namespace hierarchy.
   */
  std::string get_node_path(char node_namespace_separator) const;

  /**
   * @note Only applies if parameter_path_keys was specified during construction; otherwise, an empty string is returned.
   * @param parameter_namespace_separator
   * @return string The parameter path including parameter namespaces but excluding node's namespaces.
   */
  std::string get_local_path(char parameter_namespace_separator) const;

  /**
   * Member variables to store the parameter's path in the namespace hierarchy.
   * node_namespaces_ Node namespaces list.
   * parameter_path_keys_ Parameter namespaces list.
   */
  std::vector<std::string> node_namespaces_;
  std::vector<std::string> parameter_path_keys_;
};


class ParameterReaderInterface
{
public:
  virtual ~ParameterReaderInterface() = default;

  /**
   * read a list from the provided parameter name
   * @param name the name of the parameter to be read
   * @param out the output of 'double' type.
   * @return AWS_ERR_OK if read was successful, AWS_ERR_NOT_FOUND if the parameter was not found
   * @note if the return code is not AWS_ERR_OK, out remains unchanged.
   */
  virtual AwsError ReadParam(const ParameterPath & param_path, std::vector<std::string> & out) const = 0;

  /**
   * read a double value from the provided parameter name
   * @param param_path an object representing the path of the parameter to be read
   * @param out the output of 'double' type
   * @return AWS_ERR_OK if read was successful, AWS_ERR_NOT_FOUND if the parameter was not found
   * @note if the return code is not AWS_ERR_OK, out remains unchanged.
   */
  virtual AwsError ReadParam(const ParameterPath & param_path, double & out) const = 0;

  /**
   * read an integer value from the provided parameter name
   * @param param_path an object representing the path of the parameter to be read
   * @param out the output of 'int' type
   * @return AWS_ERR_OK if read was successful, AWS_ERR_NOT_FOUND if the parameter was not found
   * @note if the return code is not AWS_ERR_OK, out remains unchanged.
   */
  virtual AwsError ReadParam(const ParameterPath & param_path, int & out) const = 0;

  /**
   * read a boolean value from the provided parameter name
   * @param param_path an object representing the path of the parameter to be read
   * @param out the output of 'bool' type
   * @return AWS_ERR_OK if read was successful, AWS_ERR_NOT_FOUND if the parameter was not found
   * @note if the return code is not AWS_ERR_OK, out remains unchanged.
   */
  virtual AwsError ReadParam(const ParameterPath & param_path, bool & out) const = 0;

  /**
   * read a string value from the provided parameter name
   * @param param_path an object representing the path of the parameter to be read
   * @param out the output of 'Aws::String' type
   * @return AWS_ERR_OK if read was successful, AWS_ERR_NOT_FOUND if the parameter was not found
   * @note if the return code is not AWS_ERR_OK, out remains unchanged.
   */
  virtual AwsError ReadParam(const ParameterPath & param_path, Aws::String & out) const = 0;

  /**
   * read a string from the provided parameter name
   * @param param_path an object representing the path of the parameter to be read
   * @param out the output of 'std::string' type
   * @return AWS_ERR_OK if read was successful, AWS_ERR_NOT_FOUND if the parameter was not found
   * @note if the return code is not AWS_ERR_OK, out remains unchanged.
   */
  virtual AwsError ReadParam(const ParameterPath & param_path, std::string & out) const = 0;

  /**
   * read a map from the provided parameter name
   * @param param_path an object representing the path of the parameter to be read
   * @param out the output of 'std::map' type
   * @return AWS_ERR_OK if read was successful, AWS_ERR_NOT_FOUND if the parameter was not found
   * @note if the return code is not AWS_ERR_OK, out remains unchanged.
   */
  virtual AwsError ReadParam(const ParameterPath & param_path, std::map<std::string, std::string> & out) const = 0;
};

}  // namespace Client
}  // namespace Aws
