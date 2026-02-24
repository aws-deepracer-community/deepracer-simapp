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
#include <aws/core/utils/logging/LogMacros.h>
#include <aws/kinesis/model/GetShardIteratorRequest.h>
#include <aws/kinesis/model/ListShardsRequest.h>
#include <aws/kinesis/model/ListShardsResult.h>
#include <kinesis-video-producer/ClientCallbackProvider.h>
#include <kinesis-video-producer/KinesisVideoProducer.h>
#include <kinesis_manager/default_callbacks.h>
#include <kinesis_manager/kinesis_client_facade.h>
#include <kinesis_manager/kinesis_stream_manager.h>
#include <kinesis_manager/stream_subscription_installer.h>

using namespace com::amazonaws::kinesis::video;
using namespace Aws::Utils::Logging;


namespace Aws {
namespace Kinesis {

using namespace std;

KinesisManagerStatus KinesisStreamManagerInterface::KinesisVideoStreamSetup(
  const uint16_t stream_idx, const PBYTE codec_private_data, const uint32_t codec_private_data_size,
  std::string * stream_name)
{
  KinesisManagerStatus status = KINESIS_MANAGER_STATUS_ERROR_BASE;
  unique_ptr<StreamDefinition> stream_definition = stream_definition_provider_->GetStreamDefinition(
    GetStreamParameterPrefix(stream_idx), *parameter_reader_, codec_private_data, codec_private_data_size);
  if (!stream_definition) {
    AWS_LOGSTREAM_ERROR(__func__, "Skipping stream id "
                                    << stream_idx << " due to failure to load stream definition.");
    return KINESIS_MANAGER_STATUS_GET_STREAM_DEFINITION_FAILED;
  }
  /* Stream initialization */
  if (nullptr != stream_name) {
    *stream_name = stream_definition->getStreamName();
  }
  status = InitializeVideoStream(std::move(stream_definition));
  if (KINESIS_MANAGER_STATUS_FAILED(status)) {
    AWS_LOGSTREAM_ERROR(
      __func__, "Skipping stream id "
                  << stream_idx << " due to failure initializing stream. Error code: " << status);
  }
  return status;
}

KinesisManagerStatus KinesisStreamManagerInterface::GenerateStreamSubscriptionDescriptor(
  int stream_idx, StreamSubscriptionDescriptor & descriptor)
{
  KinesisManagerStatus status = KINESIS_MANAGER_STATUS_SUCCESS;
  int param_status = AWS_ERR_OK;
  param_status |= parameter_reader_->ReadParam(
    GetStreamParameterPath(stream_idx, kStreamParameters.topic_name),
    descriptor.topic_name);
  param_status |= parameter_reader_->ReadParam(
    GetStreamParameterPath(stream_idx, kStreamParameters.stream_name),
    descriptor.stream_name);
  param_status |= parameter_reader_->ReadParam(
    GetStreamParameterPath(stream_idx, kStreamParameters.topic_type),
    descriptor.input_type);
  if (AWS_ERR_OK != param_status) {
    AWS_LOGSTREAM_ERROR(__func__, "Missing parameters - can't construct descriptor (topic: "
                                    << descriptor.topic_name
                                    << " stream: " << descriptor.stream_name
                                    << " type: " << descriptor.input_type << ") " << param_status);
    return KINESIS_MANAGER_STATUS_INVALID_INPUT;
  }
  /* Rekognition data stream and topic name - one cannot be provided without the other */
  AwsError data_stream_read_result = parameter_reader_->ReadParam(
    GetStreamParameterPath(stream_idx, kStreamParameters.rekognition_data_stream),
    descriptor.rekognition_data_stream);
  AwsError rekognition_topic_read_result = parameter_reader_->ReadParam(
    GetStreamParameterPath(stream_idx, kStreamParameters.rekognition_topic_name),
    descriptor.rekognition_topic_name);
  if (data_stream_read_result != rekognition_topic_read_result ||
      (data_stream_read_result != AWS_ERR_OK && data_stream_read_result != AWS_ERR_NOT_FOUND)) {
    AWS_LOGSTREAM_ERROR(
      __func__, "Invalid input: error reading parameters for AWS Rekognition support (data stream: "
                  << descriptor.rekognition_data_stream << " code: " << data_stream_read_result
                  << " Rekognition topic: " << descriptor.rekognition_topic_name
                  << " code: " << rekognition_topic_read_result << ")");
    return KINESIS_MANAGER_STATUS_INVALID_INPUT;
  }
  uint32_t message_queue_size = kDefaultMessageQueueSize;
  int message_queue_size_input;
  if (AWS_ERR_OK ==
      parameter_reader_->ReadParam(
        GetStreamParameterPath(stream_idx, kStreamParameters.message_queue_size),
        message_queue_size_input)) {
    if (0 > message_queue_size_input) {
      AWS_LOGSTREAM_WARN(__func__, descriptor.stream_name << " Message queue size provided ("
                                                          << message_queue_size_input << ")"
                                                          << "is invalid. Using the default of "
                                                          << message_queue_size);
    } else {
      message_queue_size = static_cast<uint32_t>(message_queue_size_input);
    }
  }
  descriptor.message_queue_size = message_queue_size;
  return status;
}

KinesisManagerStatus KinesisStreamManagerInterface::KinesisVideoStreamerSetup()
{
  KinesisManagerStatus status = KINESIS_MANAGER_STATUS_ERROR_BASE;
  int video_stream_count = 0;
  parameter_reader_->ReadParam(GetKinesisVideoParameter(kStreamParameters.stream_count),
                             video_stream_count);
  if (0 >= video_stream_count) {
    AWS_LOGSTREAM_WARN(__func__, "Stream count " << video_stream_count << " is invalid. Aborting");
  }
  for (int stream_idx = 0; stream_idx < video_stream_count; stream_idx++) {
    /* Load stream definition */
    PBYTE codec_private_data = nullptr;
    uint32_t codec_private_data_size = 0;
    KinesisManagerStatus get_codec_private_data_result =
      stream_definition_provider_->GetCodecPrivateData(GetStreamParameterPrefix(stream_idx),
                                                       *parameter_reader_, &codec_private_data,
                                                       &codec_private_data_size);
    if (KINESIS_MANAGER_STATUS_FAILED(get_codec_private_data_result)) {
      AWS_LOGSTREAM_ERROR(__func__, "Skipping stream id "
                                      << stream_idx
                                      << " due to failure to load codec private data. Error code: "
                                      << get_codec_private_data_result);
      continue;
    }
    if (KINESIS_MANAGER_STATUS_FAILED(KinesisVideoStreamSetup(stream_idx, codec_private_data,
                                                              codec_private_data_size, nullptr))) {
      SAFE_MEMFREE(codec_private_data);
      continue;
    }
    /* Subscribe to the specified topic */
    StreamSubscriptionDescriptor descriptor;
    if (KINESIS_MANAGER_STATUS_FAILED(
          GenerateStreamSubscriptionDescriptor(stream_idx, descriptor))) {
      FreeStream(descriptor.stream_name);
      SAFE_MEMFREE(codec_private_data);
      continue;
    }
    KinesisManagerStatus subscription_installation_result =
      InitializeStreamSubscription(descriptor);
    if (KINESIS_MANAGER_STATUS_FAILED(subscription_installation_result)) {
      AWS_LOGSTREAM_ERROR(__func__, "Failed to subscribe to '"
                                      << descriptor.topic_name << "' for stream '"
                                      << descriptor.stream_name
                                      << "'. Error code: " << subscription_installation_result);
      FreeStream(descriptor.stream_name);
      SAFE_MEMFREE(codec_private_data);
      continue;
    }
    status = KINESIS_MANAGER_STATUS_SUCCESS;
  }
  return status;
}

unique_ptr<KinesisVideoProducerInterface> KinesisStreamManagerInterface::CreateDefaultVideoProducer(
  std::string region,
  unique_ptr<com::amazonaws::kinesis::video::DeviceInfoProvider> device_info_provider,
  unique_ptr<com::amazonaws::kinesis::video::ClientCallbackProvider> client_callback_provider,
  unique_ptr<com::amazonaws::kinesis::video::StreamCallbackProvider> stream_callback_provider,
  unique_ptr<com::amazonaws::kinesis::video::CredentialProvider> credential_provider)
{
  return std::make_unique<KinesisVideoProducerImpl>(KinesisVideoProducer::createSync(
    std::move(device_info_provider), std::move(client_callback_provider),
    std::move(stream_callback_provider), std::move(credential_provider), region));
}

KinesisManagerStatus KinesisStreamManager::InitializeStreamSubscription(
  const StreamSubscriptionDescriptor & descriptor)
{
  KinesisManagerStatus status = subscription_installer_->Install(descriptor);
  if (KINESIS_MANAGER_STATUS_SUCCEEDED(status) && !descriptor.rekognition_data_stream.empty()) {
    RekognitionStreamInfo rekognition_info{
      .data_stream_name = Aws::String(descriptor.rekognition_data_stream.c_str())};
    rekognition_config_.insert({descriptor.stream_name, rekognition_info});
  }
  return status;
}

KinesisManagerStatus KinesisStreamManager::InitializeVideoProducer(
  std::string region, unique_ptr<DeviceInfoProvider> device_info_provider,
  unique_ptr<ClientCallbackProvider> client_callback_provider,
  unique_ptr<StreamCallbackProvider> stream_callback_provider,
  unique_ptr<CredentialProvider> credential_provider,
  KinesisStreamManagerInterface::VideoProducerFactory video_producer_factory)
{
  if (video_producer_) {
    return KINESIS_MANAGER_STATUS_VIDEO_PRODUCER_ALREADY_INITIALIZED;
  }
  if (region.empty()) {
    AWS_LOG_ERROR(__func__,
                  "Region not provided. Check that the config file is correct and was loaded properly.");
    return KINESIS_MANAGER_STATUS_INVALID_INPUT;
  }
  if (!device_info_provider || !client_callback_provider || !stream_callback_provider || !credential_provider) {
    return KINESIS_MANAGER_STATUS_INVALID_INPUT;
  }
  video_producer_ = video_producer_factory(region, std::move(device_info_provider), std::move(client_callback_provider),
            std::move(stream_callback_provider), std::move(credential_provider));
  return KINESIS_MANAGER_STATUS_SUCCESS;
}

KinesisManagerStatus KinesisStreamManager::InitializeVideoProducer(std::string region,
    KinesisStreamManagerInterface::VideoProducerFactory video_producer_factory)
{
  unique_ptr<DeviceInfoProvider> device_provider = make_unique<DefaultDeviceInfoProvider>();
  unique_ptr<ClientCallbackProvider> client_callback_provider =
    make_unique<DefaultClientCallbackProvider>();
  unique_ptr<StreamCallbackProvider> stream_callback_provider =
    make_unique<DefaultStreamCallbackProvider>();
  unique_ptr<CredentialProvider> credentials_provider =
    std::make_unique<ProducerSdkAWSCredentialsProvider>();
  if (!credentials_provider) {
    AWS_LOG_ERROR(__func__,
                  "Credential provider is invalid, have you set the environment variables required "
                  "for AWS access?");
    return KINESIS_MANAGER_STATUS_DEFAULT_CREDENTIAL_PROVIDER_CREATION_FAILED;
  }
  return InitializeVideoProducer(
    region, std::move(device_provider), std::move(client_callback_provider),
    std::move(stream_callback_provider), std::move(credentials_provider),
    video_producer_factory);
}

KinesisManagerStatus KinesisStreamManager::InitializeVideoStream(
  unique_ptr<StreamDefinition> stream_definition)
{
  if (!video_producer_) {
    return KINESIS_MANAGER_STATUS_VIDEO_PRODUCER_NOT_INITIALIZED;
  }
  if (!stream_definition) {
    return KINESIS_MANAGER_STATUS_INVALID_INPUT;
  }
  string stream_name = stream_definition->getStreamName();
  if (stream_name.empty()) {
    return KINESIS_MANAGER_STATUS_EMPTY_STREAM_NAME;
  }
  if (video_streams_.count(stream_name) > 0) {
    return KINESIS_MANAGER_STATUS_STREAM_ALREADY_EXISTS;
  }

  StreamInfo stream_info = stream_definition->getStreamInfo();
  shared_ptr<KinesisVideoStreamInterface> stream;
  try {
    stream = video_producer_->CreateStreamSync(std::move(stream_definition));
  } catch (const std::runtime_error & e) {
    stream = nullptr;
  }

  if (stream) {
    video_streams_.insert({stream_name, stream});
    if (0 < stream_info.streamCaps.trackInfoList[0].codecPrivateDataSize) {
      std::vector<uint8_t> codec_private_data;
      codec_private_data.assign(
        stream_info.streamCaps.trackInfoList[0].codecPrivateData,
        stream_info.streamCaps.trackInfoList[0].codecPrivateData
        + stream_info.streamCaps.trackInfoList[0].codecPrivateDataSize);
      video_streams_codec_data_.insert({stream_name, codec_private_data});
    }
    return KINESIS_MANAGER_STATUS_SUCCESS;
  } else {
    return KINESIS_MANAGER_STATUS_CREATESTREAMSYNC_FAILED;
  }
};

KinesisManagerStatus KinesisStreamManager::PutFrame(std::string stream_name, Frame & frame) const
{
  if (!video_producer_) {
    return KINESIS_MANAGER_STATUS_VIDEO_PRODUCER_NOT_INITIALIZED;
  }
  if (0 == video_streams_.count(stream_name)) {
    return KINESIS_MANAGER_STATUS_PUTFRAME_STREAM_NOT_FOUND;
  }
  if (!video_streams_.at(stream_name)->IsReady()) {
    AWS_LOG_WARN(__func__, "Stream not ready yet, skipping putFrame.");
    return KINESIS_MANAGER_STATUS_PUTFRAME_FAILED;
  }
  bool result = video_streams_.at(stream_name)->PutFrame(frame);
  return result ? KINESIS_MANAGER_STATUS_SUCCESS : KINESIS_MANAGER_STATUS_PUTFRAME_FAILED;
};

KinesisManagerStatus KinesisStreamManager::PutMetadata(std::string stream_name,
                                                       const std::string & name,
                                                       const std::string & value) const
{
  if (!video_producer_) {
    return KINESIS_MANAGER_STATUS_VIDEO_PRODUCER_NOT_INITIALIZED;
  }
  if (0 == video_streams_.count(stream_name)) {
    return KINESIS_MANAGER_STATUS_PUTMETADATA_STREAM_NOT_FOUND;
  }
  if (!video_streams_.at(stream_name)->IsReady()) {
    AWS_LOG_WARN(__func__, "Stream not ready yet, skipping putFragmentMetadata.");
    return KINESIS_MANAGER_STATUS_PUTMETADATA_FAILED;
  }
  bool result = video_streams_.at(stream_name)->PutFragmentMetadata(name, value, false);
  return result ? KINESIS_MANAGER_STATUS_SUCCESS : KINESIS_MANAGER_STATUS_PUTMETADATA_FAILED;
};

void KinesisStreamManager::FreeStream(std::string stream_name)
{
  if (video_producer_ && video_streams_.count(stream_name) > 0) {
    if (video_streams_.at(stream_name)->IsReady()) {
      video_streams_.at(stream_name)->Stop();
    }
    video_producer_->FreeStream(video_streams_.at(stream_name));
    video_streams_.erase(stream_name);
  }
}

KinesisManagerStatus KinesisStreamManager::ProcessCodecPrivateDataForStream(
  const std::string & stream_name, std::vector<uint8_t> codec_private_data)
{
  if (0 < video_streams_codec_data_.count(stream_name) &&
      video_streams_codec_data_.at(stream_name) == codec_private_data) {
    /* Codec data is already up to date */
    return KINESIS_MANAGER_STATUS_SUCCESS;
  }
  AWS_LOGSTREAM_INFO(__func__, "Updating new codec data for " << stream_name);
  /* Get stream configuration ID */
  int video_stream_count = 0, stream_idx = 0;
  KinesisManagerStatus status = KINESIS_MANAGER_STATUS_PROCESS_CODEC_DATA_STREAM_CONFIG_NOT_FOUND;
  parameter_reader_->ReadParam(GetKinesisVideoParameter(kStreamParameters.stream_count),
                             video_stream_count);
  for (int stream_idx = 0; stream_idx < video_stream_count; stream_idx++) {
    std::string configured_stream_name;
    parameter_reader_->ReadParam(
      GetStreamParameterPath(stream_idx, kStreamParameters.stream_name),
      configured_stream_name);
    if (configured_stream_name == stream_name) {
      status = KINESIS_MANAGER_STATUS_SUCCESS;
      break;
    }
  }
  if (KINESIS_MANAGER_STATUS_FAILED(status)) {
    return status;
  }
  /* Re-create the stream with the new codec data */
  FreeStream(stream_name);
  status = KinesisVideoStreamSetup(stream_idx, codec_private_data.data(), codec_private_data.size(),
                                   nullptr);
  if (KINESIS_MANAGER_STATUS_FAILED(status)) {
    /* At this point we have an active subscription without the ability to stream data; need to
     * unsubscribe */
    std::string topic_name;
    parameter_reader_->ReadParam(
      GetStreamParameterPath(stream_idx, kStreamParameters.topic_name), topic_name);
    AWS_LOGSTREAM_ERROR(__func__, "KinesisVideoStreamSetup failed, uninstalling subscriptions to "
                                    << topic_name << " Error code: " << status);
    subscription_installer_->Uninstall(topic_name);
  }
  return status;
}

KinesisManagerStatus KinesisStreamManager::UpdateShardIterator(const std::string & stream_name)
{
  if (!rekognition_config_.at(stream_name).shard_iterator.empty()) {
    /* Already loaded */
    return KINESIS_MANAGER_STATUS_SUCCESS;
  }
  /* First ListShards, then GetShardIterator */
  Model::ListShardsRequest list_shards_request;
  list_shards_request.SetStreamName(rekognition_config_.at(stream_name).data_stream_name);
  list_shards_request.SetMaxResults(1);
  auto list_shards_outcome = kinesis_client_->ListShards(list_shards_request);
  if (!list_shards_outcome.IsSuccess()) {
    AWS_LOGSTREAM_ERROR(__func__,
                        "ListShards failed with code "
                          << static_cast<int>(list_shards_outcome.GetError().GetErrorType()) << ": "
                          << list_shards_outcome.GetError().GetMessage());
    return KINESIS_MANAGER_STATUS_LIST_SHARDS_FAILED;
  }
  if (list_shards_outcome.GetResult().GetShards().empty()) {
    AWS_LOG_ERROR(__func__, "ListShards: no shards available");
    return KINESIS_MANAGER_STATUS_LIST_SHARDS_EMPTY;
  }

  Aws::String shard_id = list_shards_outcome.GetResult().GetShards().front().GetShardId();
  Model::GetShardIteratorRequest get_shard_iterator_request;
  get_shard_iterator_request.SetStreamName(rekognition_config_.at(stream_name).data_stream_name);
  get_shard_iterator_request.SetShardId(shard_id);
  get_shard_iterator_request.SetShardIteratorType(Model::ShardIteratorType::LATEST);
  auto get_shard_iterator_outcome = kinesis_client_->GetShardIterator(get_shard_iterator_request);
  if (!get_shard_iterator_outcome.IsSuccess()) {
    AWS_LOGSTREAM_ERROR(
      __func__, "GetShardIterator failed with code "
                  << static_cast<int>(get_shard_iterator_outcome.GetError().GetErrorType()) << ": "
                  << get_shard_iterator_outcome.GetError().GetMessage());
    return KINESIS_MANAGER_STATUS_GET_SHARD_ITERATOR_FAILED;
  }
  rekognition_config_.at(stream_name).shard_iterator =
    get_shard_iterator_outcome.GetResult().GetShardIterator();
  return KINESIS_MANAGER_STATUS_SUCCESS;
}

KinesisManagerStatus KinesisStreamManager::FetchRekognitionResults(
  const std::string & stream_name, Aws::Vector<Model::Record> * records)
{
  KinesisManagerStatus status = KINESIS_MANAGER_STATUS_SUCCESS;
  if (0 == rekognition_config_.count(stream_name)) {
    AWS_LOG_WARN(__func__, "AWS Rekognition configuration is missing for this stream. Skipping");
    return status;
  }
  if (rekognition_config_.at(stream_name).shard_iterator.empty()) {
    status = UpdateShardIterator(stream_name);
    if (KINESIS_MANAGER_STATUS_FAILED(status)) {
      return status;
    }
  }
  Model::GetRecordsRequest get_records_request;
  get_records_request.SetShardIterator(rekognition_config_.at(stream_name).shard_iterator);
  get_records_request.SetLimit(kDefaultRecordsLimitForRekognitionResults);
  auto get_records_outcome = kinesis_client_->GetRecords(get_records_request);
  if (get_records_outcome.IsSuccess()) {
    rekognition_config_.at(stream_name).shard_iterator =
      get_records_outcome.GetResult().GetNextShardIterator();
    *records = get_records_outcome.GetResult().GetRecords();
  } else {
    if (KinesisErrors::PROVISIONED_THROUGHPUT_EXCEEDED ==
        get_records_outcome.GetError().GetErrorType()) {
      return KINESIS_MANAGER_STATUS_GET_RECORDS_THROTTLED;
    } else if (KinesisErrors::EXPIRED_ITERATOR == get_records_outcome.GetError().GetErrorType()) {
      rekognition_config_.at(stream_name).shard_iterator.clear();
      AWS_LOG_WARN(
        __func__,
        "GetRecords failed due to expired iterator. A new one will be fetched at the next run.");
    } else {
      AWS_LOGSTREAM_ERROR(
        __func__, "GetRecords failed with code "
                    << static_cast<int>(get_records_outcome.GetError().GetErrorType()) << ": "
                    << get_records_outcome.GetError().GetMessage());
    }
    return KINESIS_MANAGER_STATUS_GET_RECORDS_FAILED;
  }
  return status;
}

}  // namespace Kinesis
}  // namespace Aws
