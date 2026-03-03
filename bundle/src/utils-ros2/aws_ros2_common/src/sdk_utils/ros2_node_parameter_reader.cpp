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

#include <aws_ros2_common/sdk_utils/ros2_node_parameter_reader.h>
#include <rclcpp/rclcpp.hpp>

namespace Aws {
namespace Client {

constexpr char kParameterNsSeparator = '.';
constexpr char kNodeNsSeparator = '/';

/**
 * Generic parameter reading helper function.
 * @param parameter_path object representing the parameter's path
 * @param out reference to which the parameter's value should be written to.
 */
template <class T>
static AwsError ReadParamTemplate(const ParameterPath & param_path,
                                  const std::weak_ptr<rclcpp::Node>& node,
                                  T & out) {
    std::string name = param_path.get_resolved_path(kNodeNsSeparator, kParameterNsSeparator);
    if (std::string::npos != name.find(kNodeNsSeparator)) {
        /* Reading remote node's parameters is not supported yet by Ros2NodeParameterReader. */
        return AWS_ERR_NOT_SUPPORTED;
    }
    if (auto node_handle = node.lock()) {
        if (node_handle->get_parameter(name, out)) {
            return AWS_ERR_OK;
        } else {
            return AWS_ERR_NOT_FOUND;
        }
    } else {
        /* The node object has been destroyed. */
        return AWS_ERR_MEMORY;
    }
}

AwsError Ros2NodeParameterReader::ReadParam(const ParameterPath & param_path, std::vector<std::string> &out) const {
    return ReadParamTemplate(param_path, node_, out);
}

AwsError Ros2NodeParameterReader::ReadParam(const ParameterPath & param_path, double &out) const {
    return ReadParamTemplate(param_path, node_, out);
}

AwsError Ros2NodeParameterReader::ReadParam(const ParameterPath & param_path, int &out) const {
    return ReadParamTemplate(param_path, node_, out);
}

AwsError Ros2NodeParameterReader::ReadParam(const ParameterPath & param_path, bool &out) const {
    return ReadParamTemplate(param_path, node_, out);
}

AwsError Ros2NodeParameterReader::ReadParam(const ParameterPath & param_path, std::string &out) const {
    return ReadParamTemplate(param_path, node_, out);
}

AwsError Ros2NodeParameterReader::ReadParam(const ParameterPath &  /*param_path*/, std::map<std::string, std::string> &  /*out*/) const {
    return AWS_ERR_NOT_SUPPORTED;
}

}  // namespace Client

}  // namespace Aws

