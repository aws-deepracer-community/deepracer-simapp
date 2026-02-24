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

#include <kinesis_manager/kinesis_video_producer_interface.h>

namespace Aws {
namespace Kinesis {

std::shared_ptr<KinesisVideoStreamInterface> KinesisVideoProducerImpl::CreateStreamSync(
    std::unique_ptr<com::amazonaws::kinesis::video::StreamDefinition> stream_definition)
{
  auto video_stream = video_producer_->createStreamSync(std::move(stream_definition));
  return std::make_shared<KinesisVideoStreamImpl>(video_stream);
}

void KinesisVideoProducerImpl::FreeStream(
    std::shared_ptr<KinesisVideoStreamInterface> kinesis_video_stream)
{
  auto video_stream = kinesis_video_stream->GetKinesisVideoStream();
  if (nullptr != video_stream) {
    video_producer_->freeStream(video_stream);
  }
}

}  // namespace Kinesis
}  // namespace Aws
