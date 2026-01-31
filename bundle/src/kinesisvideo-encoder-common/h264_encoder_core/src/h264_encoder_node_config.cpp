/*
 *  Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <aws/core/utils/logging/LogMacros.h>
#include <h264_encoder_core/h264_encoder_node_config.h>

namespace Aws {
namespace Kinesis {

constexpr char kSubscriptionTopicKey[] = "subscription_topic";
constexpr char kMetadataTopicKey[] = "metadata_topic";
constexpr char kPublicationTopicKey[] = "publication_topic";
constexpr char kQueueSizeKey[] = "queue_size";

constexpr char kDefaultSubscriptionTopic[] = "/raspicam_node/image";
constexpr char kDefaultMetadataTopic[] = "/image_metadata";
constexpr char kDefaultPublicationTopic[] = "/video/encoded";
constexpr int kDefaultQueueSize = 100;

Aws::AwsError GetH264EncoderNodeParams(const Aws::Client::ParameterReaderInterface & param_reader,
                                       H264EncoderNodeParams & params)
{
  params.subscription_topic = kDefaultSubscriptionTopic;
  param_reader.ReadParam(Aws::Client::ParameterPath(kSubscriptionTopicKey), params.subscription_topic);

  params.metadata_topic = kDefaultMetadataTopic;
  param_reader.ReadParam(Aws::Client::ParameterPath(kMetadataTopicKey), params.metadata_topic);

  params.publication_topic = kDefaultPublicationTopic;
  param_reader.ReadParam(Aws::Client::ParameterPath(kPublicationTopicKey), params.publication_topic);

  params.queue_size = kDefaultQueueSize;
  param_reader.ReadParam(Aws::Client::ParameterPath(kQueueSizeKey), params.queue_size);
  if (params.queue_size < 0) {
    AWS_LOGSTREAM_ERROR(__func__, "Invalid queue size " << params.queue_size << "!");
    return AWS_ERR_PARAM;
  } else if (params.queue_size == 0) {
    AWS_LOGSTREAM_WARN(__func__, "Queue size is set to " << params.queue_size << " (infinity)!");
  }

  return AWS_ERR_OK;
}

}  // namespace Kinesis
}  // namespace Aws
