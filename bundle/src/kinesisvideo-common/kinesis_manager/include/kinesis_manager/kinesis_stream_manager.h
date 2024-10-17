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
#include <aws/kinesis/KinesisClient.h>
#include <aws/kinesis/model/GetRecordsRequest.h>
#include <aws/kinesis/model/GetRecordsResult.h>
#include <aws/kinesis/model/Record.h>
#include <aws_common/sdk_utils/client_configuration_provider.h>
#include <aws_common/sdk_utils/logging/aws_log_system.h>
#include <kinesis-video-producer/KinesisVideoProducer.h>
#include <kinesis_manager/common.h>
#include <kinesis_manager/kinesis_client_facade.h>
#include <kinesis_manager/stream_definition_provider.h>
#include <kinesis_manager/stream_subscription_installer.h>
#include <kinesis_manager/kinesis_video_producer_interface.h>

namespace Aws {
namespace Kinesis {

/**
 * Constructs parameter paths for AWS Kinesis applications.
 * @param stream_idx
 * @param parameter_name
 * @return ParameterPath object representing the parameter path, i.e.
 * kinesis_video/stream<stream_idx>/<parameter_name>
 * @note calling GetStreamParameterPath with negative stream index would result in construction of a
 * global parameter path, rather than one that's related to a specific stream. This is appropriate
 * for parameters such as "stream_count".
 */
inline Aws::Client::ParameterPath GetStreamParameterPath(int stream_idx, const char * parameter_name)
{
  Aws::Client::ParameterPath path(kStreamParameters.prefix);
  if (INVALID_STREAM_ID != stream_idx) {
    path += kStreamParameters.stream_namespace + std::to_string(stream_idx);
  }
  if (nullptr != parameter_name) {
    path += parameter_name;
  }
  return path;
}

/**
 * Use to retrieve the prefix path for a given stream, i.e. "kinesis_video/stream5/" for
 * stream_idx=5
 */
inline Aws::Client::ParameterPath GetStreamParameterPrefix(int stream_idx)
{
  return GetStreamParameterPath(stream_idx, nullptr);
}
/**
 * Use for parameters which live under the global kinesis_video/ namespace rather than under a
 * particular stream (e.g. stream_count).
 */
inline Aws::Client::ParameterPath GetKinesisVideoParameter(const char * parameter_name)
{
  return GetStreamParameterPath(INVALID_STREAM_ID, parameter_name);
}

class KinesisStreamManagerInterface
{
public:
  KinesisStreamManagerInterface(const Aws::Client::ParameterReaderInterface * parameter_reader,
                                const StreamDefinitionProvider * stream_definition_provider,
                                StreamSubscriptionInstaller * subscription_installer)
  : parameter_reader_(parameter_reader),
    stream_definition_provider_(stream_definition_provider),
    subscription_installer_(subscription_installer){};
  KinesisStreamManagerInterface() = default;
  virtual ~KinesisStreamManagerInterface() = default;

  using VideoProducerFactory = std::function<std::unique_ptr<KinesisVideoProducerInterface>(
    std::string,
    std::unique_ptr<com::amazonaws::kinesis::video::DeviceInfoProvider>,
    std::unique_ptr<com::amazonaws::kinesis::video::ClientCallbackProvider>,
    std::unique_ptr<com::amazonaws::kinesis::video::StreamCallbackProvider>,
    std::unique_ptr<com::amazonaws::kinesis::video::CredentialProvider>
  )>;

  /**
   * Initializes the video producer with the given callbacks.
   * @note This function must be called if the KinesisStreamManager is to be used for video streams.
   * @note This function should only be called once.
   * @param region
   * @param device_info_provider
   * @param client_callback_provider
   * @param stream_callback_provider
   * @param credential_provider
   * @param video_producer_factory
   * @return KinesisManagerStatus
   */
  virtual KinesisManagerStatus InitializeVideoProducer(std::string region,
    std::unique_ptr<com::amazonaws::kinesis::video::DeviceInfoProvider> device_info_provider,
    std::unique_ptr<com::amazonaws::kinesis::video::ClientCallbackProvider> client_callback_provider,
    std::unique_ptr<com::amazonaws::kinesis::video::StreamCallbackProvider> stream_callback_provider,
    std::unique_ptr<com::amazonaws::kinesis::video::CredentialProvider> credential_provider,
    VideoProducerFactory video_producer_factory = KinesisStreamManagerInterface::CreateDefaultVideoProducer) = 0;

  /**
   * Initializes the video producer using the default callbacks provided as part of the Kinesis
   * Manager package.
   * @param region
   * @param video_producer_factory
   * @return KinesisManagerStatus
   */
  virtual KinesisManagerStatus InitializeVideoProducer(std::string region,
    VideoProducerFactory video_producer_factory = KinesisStreamManagerInterface::CreateDefaultVideoProducer) = 0;

  /**
   * Initializes a video stream using the given stream definition.
   * @note The video producer must have been initialized before any calls to this function are made.
   * @note If a stream by the same name already exists in AWS Kinesis, it will be used without
   * creating a new one.
   * @param stream_definition
   * @return KinesisManagerStatus
   */
  virtual KinesisManagerStatus InitializeVideoStream(
    std::unique_ptr<com::amazonaws::kinesis::video::StreamDefinition> stream_definition) = 0;

  /**
   * Transmits a single video frame into AWS Kinesis.
   * @note Both the video producer and the given stream must have been initialized before any calls
   * to this function are made.
   * @param stream_name
   * @param frame
   * @return KinesisManagerStatus
   */
  virtual KinesisManagerStatus PutFrame(std::string stream_name, Frame & frame) const = 0;

  /**
   * Sends metadata to AWS Kinesis to be prepended to the next video fragment of the stream.
   * @note Both the video producer and the given stream must have been initialized before any calls
   * to this function are made.
   * @param stream_name
   * @param name the metadata name
   * @param value the metadata value
   * @return KinesisManagerStatus
   */
  virtual KinesisManagerStatus PutMetadata(std::string stream_name, const std::string & name,
                                           const std::string & value) const = 0;

  /**
   * Stops and frees the resources used by a given stream. This does not delete the stream in AWS
   * Kinesis.
   * @param stream_name
   */
  virtual void FreeStream(std::string stream_name) = 0;

  /**
   * This function should be called whenever the codec data (sequence and picture parameter sets)
   * have changed for the stream. Typically this would happen if encoder settings (such as fps or
   * resolution) were changed.
   * @param stream_name
   * @param codec_private_data
   * @return KINESIS_MANAGER_STATUS_SUCCESS if the stream was updated successfully or if no update
   * was necessary.
   */
  virtual KinesisManagerStatus ProcessCodecPrivateDataForStream(
    const std::string & stream_name, std::vector<uint8_t> codec_private_data) = 0;

  /**
   * This function returns the analysis results from Rekognition for the video stream, by reading
   * the associated Kinesis data stream.
   * @param stream_name the name of the Kinesis video stream.
   * @param records will store the fetched records.
   * @return KINESIS_MANAGER_STATUS_SUCCESS If the GetRecords call completed successfully.
   */
  virtual KinesisManagerStatus FetchRekognitionResults(const std::string & stream_name,
                                                       Aws::Vector<Model::Record> * records) = 0;

  static std::unique_ptr<KinesisVideoProducerInterface> CreateDefaultVideoProducer(
    std::string region,
    std::unique_ptr<com::amazonaws::kinesis::video::DeviceInfoProvider> device_info_provider,
    std::unique_ptr<com::amazonaws::kinesis::video::ClientCallbackProvider> client_callback_provider,
    std::unique_ptr<com::amazonaws::kinesis::video::StreamCallbackProvider> stream_callback_provider,
    std::unique_ptr<com::amazonaws::kinesis::video::CredentialProvider> credential_provider);

protected:
  /**
   * This function orchestrates the universal setup of a video streaming application.
   *  @see common.h for details about the input format for the stream definition.
   * @param parameter_reader
   * @param subscription_installer responsible for subscribing to a media source and
   *  setting up callbacks that would convert/transmit the data to AWS Kinesis.
   * @param stream_definition_provider
   * @return KinesisManagerStatus returning KINESIS_MANAGER_STATUS_SUCCESS if at least one stream
   * was successfully bootstrapped.
   */
  virtual KinesisManagerStatus KinesisVideoStreamerSetup();
  /**
   * This function orchestrates the setup of a single video stream and is called by
   * KinesisVideoStreamerSetup.
   * @param stream_idx
   * @param codec_private_data
   * @param codec_private_data_size
   * @param stream_name If not null, the stream name will be stored in this location.
   * @return KinesisManagerStatus
   */
  virtual KinesisManagerStatus KinesisVideoStreamSetup(const uint16_t stream_idx,
                                                       const PBYTE codec_private_data,
                                                       const uint32_t codec_private_data_size,
                                                       std::string * stream_name);
  /**
   * Loads the relevant parameters associated with the stream and fills the provided descriptor.
   * @param descriptor
   * @return KinesisManagerStatus
   */
  virtual KinesisManagerStatus GenerateStreamSubscriptionDescriptor(
    int stream_idx, StreamSubscriptionDescriptor & descriptor);
  /**
   * Initializes the subscription for the stream's input.
   * @param descriptor a subscription descriptor previously generated by
   * GenerateStreamSubscriptionDescriptor.
   * @return KinesisManagerStatus
   */
  virtual KinesisManagerStatus InitializeStreamSubscription(
    const StreamSubscriptionDescriptor & descriptor) = 0;

  const Aws::Client::ParameterReaderInterface * parameter_reader_ = nullptr;
  const StreamDefinitionProvider * stream_definition_provider_ = nullptr;
  StreamSubscriptionInstaller * subscription_installer_ = nullptr;
};

/**
 * Use this class to manage AWS Kinesis streams.
 *  In order to use Video Streams, make sure to call InitializeVideoProducer before creating any
 * streams.
 */
class KinesisStreamManager : public KinesisStreamManagerInterface
{
public:
  KinesisStreamManager(Aws::Client::ParameterReaderInterface * parameter_reader,
                       StreamDefinitionProvider * stream_definition_provider,
                       StreamSubscriptionInstaller * subscription_installer,
                       std::unique_ptr<KinesisClient> kinesis_client)
  : kinesis_client_(std::move(kinesis_client)),
    KinesisStreamManagerInterface(parameter_reader, stream_definition_provider,
                                  subscription_installer){};
  KinesisStreamManager() = default;
  ~KinesisStreamManager() = default;

  KinesisManagerStatus InitializeVideoProducer(std::string region,
    std::unique_ptr<com::amazonaws::kinesis::video::DeviceInfoProvider> device_info_provider,
    std::unique_ptr<com::amazonaws::kinesis::video::ClientCallbackProvider> client_callback_provider,
    std::unique_ptr<com::amazonaws::kinesis::video::StreamCallbackProvider> stream_callback_provider,
    std::unique_ptr<com::amazonaws::kinesis::video::CredentialProvider> credential_provider,
    KinesisStreamManagerInterface::VideoProducerFactory video_producer_factory = KinesisStreamManagerInterface::CreateDefaultVideoProducer) override;
  KinesisManagerStatus InitializeVideoProducer(std::string region,
    KinesisStreamManagerInterface::VideoProducerFactory video_producer_factory = KinesisStreamManagerInterface::CreateDefaultVideoProducer) override;

  KinesisManagerStatus InitializeVideoStream(
    std::unique_ptr<com::amazonaws::kinesis::video::StreamDefinition> stream_definition) override;

  KinesisManagerStatus PutFrame(std::string stream_name, Frame & frame) const override;

  KinesisManagerStatus PutMetadata(std::string stream_name, const std::string & name,
                                   const std::string & value) const override;

  void FreeStream(std::string stream_name) override;

  KinesisManagerStatus KinesisVideoStreamerSetup() override
  {
    return KinesisStreamManagerInterface::KinesisVideoStreamerSetup();
  }

  KinesisManagerStatus KinesisVideoStreamSetup(const uint16_t stream_idx,
                                               const PBYTE codec_private_data,
                                               const uint32_t codec_private_data_size,
                                               std::string * stream_name) override
  {
    return KinesisStreamManagerInterface::KinesisVideoStreamSetup(
      stream_idx, codec_private_data, codec_private_data_size, stream_name);
  }

  KinesisManagerStatus ProcessCodecPrivateDataForStream(
    const std::string & stream_name, std::vector<uint8_t> codec_private_data) override;

  KinesisManagerStatus FetchRekognitionResults(const std::string & stream_name,
                                               Aws::Vector<Model::Record> * records) override;

  KinesisVideoProducerInterface * get_video_producer()
  {
    return video_producer_.get();
  }

protected:
  KinesisManagerStatus InitializeStreamSubscription(
    const StreamSubscriptionDescriptor & descriptor) override;

private:
  /**
   * Updates RekognitionStreamInfo::shard_iterator by calling Kinesis' ListShards and
   * GetShardIterator APIs.
   * @param stream_name
   * @return KinesisManagerStatus
   */
  KinesisManagerStatus UpdateShardIterator(const std::string & stream_name);

  std::map<std::string, std::shared_ptr<KinesisVideoStreamInterface>> video_streams_;
  std::map<std::string, std::vector<uint8_t>> video_streams_codec_data_;
  std::unique_ptr<KinesisVideoProducerInterface> video_producer_;
  std::unique_ptr<KinesisClient> kinesis_client_;

  struct RekognitionStreamInfo
  {
    Aws::String data_stream_name; /* Analysis results will be fetched from this stream. */
    Aws::String shard_iterator;
  };
  std::map<std::string, RekognitionStreamInfo>
    rekognition_config_; /* Video stream name to RekognitionStreamInfo */
};

}  // namespace Kinesis
}  // namespace Aws
