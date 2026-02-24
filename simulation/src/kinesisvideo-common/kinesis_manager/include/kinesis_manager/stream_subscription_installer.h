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
#include <aws_common/sdk_utils/logging/aws_log_system.h>
#include <kinesis_manager/common.h>

#include <boost/function.hpp>
#include <map>

namespace Aws {
namespace Kinesis {

struct StreamSubscriptionDescriptor
{
  KinesisStreamInputType input_type;
  std::string topic_name;
  std::string stream_name;
  uint32_t message_queue_size;
  std::string rekognition_topic_name;
  std::string rekognition_data_stream;
};

typedef std::function<bool(const StreamSubscriptionDescriptor & descriptor)> SubscriberSetupFn;

/**
 * This class abstracts away the installation of media sources (i.e. subscriptions) for Kinesis
 * streams.
 */
class StreamSubscriptionInstaller
{
public:
  StreamSubscriptionInstaller() = default;
  virtual ~StreamSubscriptionInstaller() = default;
  /**
   * Installs a subscription that will provide and handle the stream's input. This function is a
   * shim layer that maps an input type to a pre-set installer function. Child classes should
   * implement the individual installers.
   * @param input_type
   * @param topic_name
   * @param stream_name
   * @param message_queue_size
   * @return KinesisManagerStatus
   */
  virtual KinesisManagerStatus Install(const StreamSubscriptionDescriptor & descriptor) const
  {
    if (descriptor.topic_name.empty() || descriptor.stream_name.empty()) {
      return KINESIS_MANAGER_STATUS_INVALID_INPUT;
    }
    if (0 == installers_.count(descriptor.input_type)) {
      return KINESIS_MANAGER_STATUS_SUBSCRIPTION_INSTALLER_NOT_FOUND;
    }

    bool result = installers_.at(descriptor.input_type)(descriptor);
    return result ? KINESIS_MANAGER_STATUS_SUCCESS
                  : KINESIS_MANAGER_STATUS_SUBSCRIPTION_INSTALLATION_FAILED;
  }
  /**
   * Uninstalls all subscriptions for the given topic name.
   * @param topic_name
   */
  virtual void Uninstall(const std::string & topic_name) = 0;

protected:
  /**
   * Different stream input types will require different subscriber setup functions.
   */
  std::map<KinesisStreamInputType, SubscriberSetupFn> installers_;
};

}  // namespace Kinesis
}  // namespace Aws