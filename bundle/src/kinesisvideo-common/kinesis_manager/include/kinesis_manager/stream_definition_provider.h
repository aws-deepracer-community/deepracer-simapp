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
#include <aws_common/sdk_utils/client_configuration_provider.h>
#include <kinesis-video-producer/KinesisVideoProducer.h>
#include <kinesis_manager/common.h>


namespace Aws {
namespace Kinesis {
/**
 * Responsible for creating stream definition objects by using a parameter reader to get the
 * relevant data.
 * @see https://docs.aws.amazon.com/kinesisvideostreams/latest/dg/how-data.html
 */
class StreamDefinitionProvider
{
public:
  ~StreamDefinitionProvider() = default;
  /**
   * Reads a base64-encoded codecPrivateData using the given Parameter Reader.
   * @param prefix a ParameterPath object representing the path of the parameter's prefix.
   * @param reader implementing ParameterReaderInterface
   * @param out_codec_private_data - newly allocated buffer containing the codec private data.
   *  @note The pointer remains unchanged in case of an error or if no codec data was provided.
   *  @note The caller of GetCodecPrivateData is responsible for freeing up this buffer.
   * @param out_codec_private_data_size - the actual size of the allocated buffer will be stored
   * here.
   * @return kinesis_manager_status_t with KINESIS_MANAGER_SUCCESS on success or if no codec data
   * was provided
   */
  virtual KinesisManagerStatus
  GetCodecPrivateData(const Aws::Client::ParameterPath & prefix,
                      const Aws::Client::ParameterReaderInterface & reader,
                      PBYTE * out_codec_private_data,
                      uint32_t * out_codec_private_data_size) const;

  /**
   * Creates a StreamDefinition by using a given Parameter Reader. Uses sensible defaults in case
   * the parameter is not configured.
   * @see
   * https://github.com/awslabs/amazon-kinesis-video-streams-producer-sdk-cpp/blob/master/kinesis-video-producer/src/StreamDefinition.h
   *  and https://docs.aws.amazon.com/kinesisvideostreams/latest/dg/how-data.html
   * @param prefix a ParameterPath object representing the path of the parameter's prefix.
   * @param reader implementing ParameterReaderInterface
   * @param codec_private_data the picture and sequence parameter sets of the stream (optional).
   * @param codec_private_data_size size in bytes of codec_private_data.
   * @return unique_ptr to StreamDefinition on success or nullptr on failure.
   */
  virtual std::unique_ptr<com::amazonaws::kinesis::video::StreamDefinition>
  GetStreamDefinition(const Aws::Client::ParameterPath & prefix,
                      const Aws::Client::ParameterReaderInterface & reader,
                      const PBYTE codec_private_data,
                      uint32_t codec_private_data_size) const;
};

}  // namespace Kinesis
}  // namespace Aws
