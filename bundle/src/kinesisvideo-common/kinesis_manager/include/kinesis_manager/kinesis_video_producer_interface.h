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

#include <kinesis_manager/kinesis_video_stream_interface.h>
#include <kinesis-video-producer/KinesisVideoProducer.h>

namespace Aws {
namespace Kinesis {

class KinesisVideoProducerInterface
{
public:
  
  /**
   * Create a video stream
   * @param stream_definition A unique pointer to the StreamDefinition which describes the
   *                          stream to be created.
   * @return An KinesisVideoStream instance which is ready to start streaming.
   */
  virtual std::shared_ptr<KinesisVideoStreamInterface> CreateStreamSync(
    std::unique_ptr<com::amazonaws::kinesis::video::StreamDefinition> stream_definition) = 0;

  /**
   * Frees the stream and removes it from the producer stream list.
   *
   * NOTE: This is a prompt operation and will stop the stream immediately without
   * emptying the buffer.
   *
   * @param KinesisVideo_stream A unique pointer to the KinesisVideoStream to free and remove.
   */
  virtual void FreeStream(
    std::shared_ptr<KinesisVideoStreamInterface> kinesis_video_stream) = 0;

  virtual ~KinesisVideoProducerInterface(){};
};

class KinesisVideoProducerImpl : public KinesisVideoProducerInterface
{
public:
  KinesisVideoProducerImpl(std::unique_ptr<com::amazonaws::kinesis::video::KinesisVideoProducer> video_producer)
  : video_producer_(std::move(video_producer)){};

  virtual std::shared_ptr<KinesisVideoStreamInterface> CreateStreamSync(
    std::unique_ptr<com::amazonaws::kinesis::video::StreamDefinition> stream_definition) override;

  virtual void FreeStream(
    std::shared_ptr<KinesisVideoStreamInterface> kinesis_video_stream) override;

private:
  std::unique_ptr<com::amazonaws::kinesis::video::KinesisVideoProducer> video_producer_;
};


}  // namespace Kinesis
}  // namespace Aws
