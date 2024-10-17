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

#include <kinesis_manager/kinesis_video_stream_interface.h>

namespace Aws {
namespace Kinesis {

bool KinesisVideoStreamImpl::IsReady() const  
{
  return video_stream_->isReady();
}

bool KinesisVideoStreamImpl::Stop()
{
  return video_stream_->stop(); 
}

bool KinesisVideoStreamImpl::PutFrame(
  com::amazonaws::kinesis::video::KinesisVideoFrame frame) const
{
  return video_stream_->putFrame(frame);
}

bool KinesisVideoStreamImpl::PutFragmentMetadata(const std::string& name, 
        const std::string& value, bool persistent)
{
  return video_stream_->putFragmentMetadata(name, value, persistent);
}

std::shared_ptr<com::amazonaws::kinesis::video::KinesisVideoStream> 
  KinesisVideoStreamImpl::GetKinesisVideoStream()
{
  return video_stream_;
}

}  // namespace Kinesis
}  // namespace Aws
