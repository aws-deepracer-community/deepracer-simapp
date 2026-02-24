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

#include <kinesis-video-producer/KinesisVideoStream.h>

namespace Aws {
namespace Kinesis {

class KinesisVideoStreamInterface
{
public:
  /**
   * @return true if the stream is ready, false otherwise.
   */
  virtual bool IsReady() const = 0;

  /**
   * Stops the the stream. Consecutive calls will fail until start is called again.
   *
   * NOTE: The function is async and will return immediately but the stream buffer
   * will continue emptying until it's finished and the close stream will be called.
   */
  virtual bool Stop() = 0;

  /**
   * Packages and streams the frame to Kinesis Video service.
   *
   * @param frame The frame to be packaged and streamed.
   * @return true if the encoder accepted the frame and false otherwise.
   */
  virtual bool PutFrame(com::amazonaws::kinesis::video::KinesisVideoFrame frame) const = 0;

  /**
   * Appends a "metadata" - a key/value string pair into the stream.
   *
   * NOTE: The metadata is modeled as MKV tags and are not immediately put into the stream as
   * it might break the fragment.
   * This is a limitation of MKV format as Tags are level 1 elements.
   * Instead, they will be accumulated and inserted in-between the fragments and at the end of the stream.
   * @param 1 name - the metadata name.
   * @param 2 value - the metadata value.
   * @param 3 persistent - whether the metadata is persistent.
   *
   */
  virtual bool PutFragmentMetadata(const std::string& name, 
    const std::string& value, bool persistent = true) = 0;

  virtual std::shared_ptr<com::amazonaws::kinesis::video::KinesisVideoStream> 
    GetKinesisVideoStream() {
      return nullptr;
    };

  virtual ~KinesisVideoStreamInterface() {};
};

class KinesisVideoStreamImpl : public KinesisVideoStreamInterface
{
public:
  KinesisVideoStreamImpl(
    std::shared_ptr<com::amazonaws::kinesis::video::KinesisVideoStream> video_stream)
  : video_stream_(video_stream){};

  virtual bool IsReady() const override;
  virtual bool Stop() override;
  virtual bool PutFrame(com::amazonaws::kinesis::video::KinesisVideoFrame frame) const override;
  virtual bool PutFragmentMetadata(const std::string& name, 
    const std::string& value, bool persistent = true) override;
  virtual std::shared_ptr<com::amazonaws::kinesis::video::KinesisVideoStream> 
    GetKinesisVideoStream() override;

private:
  std::shared_ptr<com::amazonaws::kinesis::video::KinesisVideoStream> video_stream_;
};

}  // namespace Kinesis
}  // namespace Aws
