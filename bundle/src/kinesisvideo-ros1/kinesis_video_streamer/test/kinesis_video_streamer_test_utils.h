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

#include <aws_common/sdk_utils/aws_error.h>
#include <aws_common/sdk_utils/logging/aws_log_system.h>
#include <kinesis_manager/stream_definition_provider.h>
#include <kinesis_video_streamer/ros_stream_subscription_installer.h>
#include <kinesis_video_streamer/streamer.h>

using namespace com::amazonaws::kinesis::video;
using namespace Aws;
using namespace Aws::Client;
using namespace Aws::Kinesis;


struct TestData
{
  uint32_t get_codec_private_data_call_count = 0;
  KinesisManagerStatus get_codec_private_data_return_value = KINESIS_MANAGER_STATUS_SUCCESS;
  uint32_t get_stream_definition_call_count = 0;
  StreamDefinition * get_stream_definition_return_value = nullptr;
  uint32_t subscribe_call_count = 0;
  KinesisManagerStatus subscribe_return_value = KINESIS_MANAGER_STATUS_SUCCESS;
  uint32_t initialize_video_producer_call_count = 0;
  KinesisManagerStatus initialize_video_producer_return_value = KINESIS_MANAGER_STATUS_SUCCESS;
  uint32_t initialize_video_stream_call_count = 0;
  KinesisManagerStatus initialize_video_stream_return_value = KINESIS_MANAGER_STATUS_SUCCESS;
  uint32_t put_frame_call_count = 0;
  KinesisManagerStatus put_frame_return_value = KINESIS_MANAGER_STATUS_SUCCESS;
  uint32_t put_metadata_call_count = 0;
  KinesisManagerStatus put_metadata_return_value = KINESIS_MANAGER_STATUS_SUCCESS;
  uint32_t free_stream_call_count = 0;
  uint32_t kinesis_video_frame_callback_call_count = 0;
  uint32_t rekognition_kinesis_video_frame_callback_call_count = 0;
  uint32_t fetch_rekognition_results_call_count = 0;
  uint32_t image_callback_call_count = 0;
  uint32_t process_codec_private_data_call_count = 0;
  KinesisManagerStatus process_codec_private_data_return_value = KINESIS_MANAGER_STATUS_SUCCESS;

  void Reset()
  {
    get_codec_private_data_call_count = 0;
    get_codec_private_data_return_value = KINESIS_MANAGER_STATUS_SUCCESS;
    get_stream_definition_call_count = 0;
    get_stream_definition_return_value = nullptr;
    subscribe_call_count = 0;
    subscribe_return_value = KINESIS_MANAGER_STATUS_SUCCESS;
    initialize_video_producer_call_count = 0;
    initialize_video_producer_return_value = KINESIS_MANAGER_STATUS_SUCCESS;
    initialize_video_stream_call_count = 0;
    initialize_video_stream_return_value = KINESIS_MANAGER_STATUS_SUCCESS;
    put_frame_call_count = 0;
    put_frame_return_value = KINESIS_MANAGER_STATUS_SUCCESS;
    put_metadata_call_count = 0;
    put_metadata_return_value = KINESIS_MANAGER_STATUS_SUCCESS;
    free_stream_call_count = 0;
    kinesis_video_frame_callback_call_count = 0;
    rekognition_kinesis_video_frame_callback_call_count = 0;
    fetch_rekognition_results_call_count = 0;
    image_callback_call_count = 0;
    process_codec_private_data_call_count = 0;
    process_codec_private_data_return_value = KINESIS_MANAGER_STATUS_SUCCESS;
  }
};

struct MockStreamDefinitionProvider : public StreamDefinitionProvider
{
  TestData * data_;
  MockStreamDefinitionProvider(TestData * data) { data_ = data; }

  KinesisManagerStatus GetCodecPrivateData(const ParameterPath & prefix,
                                           const ParameterReaderInterface & reader,
                                           PBYTE * out_codec_private_data,
                                           uint32_t * out_codec_private_data_size) const override
  {
    data_->get_codec_private_data_call_count++;
    return data_->get_codec_private_data_return_value;
  }

  std::unique_ptr<StreamDefinition> GetStreamDefinition(const ParameterPath & prefix,
                                                   const ParameterReaderInterface & reader,
                                                   const PBYTE codec_private_data,
                                                   uint32_t codec_private_data_size) const override
  {
    data_->get_stream_definition_call_count++;
    return std::unique_ptr<StreamDefinition>(data_->get_stream_definition_return_value);
  }
};

struct MockStreamSubscriptionInstaller : public RosStreamSubscriptionInstaller
{
  TestData * data_;
  MockStreamSubscriptionInstaller(TestData * data,
                                  Aws::Kinesis::KinesisStreamManagerInterface & stream_manager,
                                  ros::NodeHandle & handle)
  : data_(data), RosStreamSubscriptionInstaller(handle)
  {
  }

  KinesisManagerStatus Install(const StreamSubscriptionDescriptor & descriptor) const override
  {
    data_->subscribe_call_count++;
    return data_->subscribe_return_value;
  }
  void Uninstall(std::string & topic_name) {}
};

class TestParameterReader : public ParameterReaderInterface
{
public:
  TestParameterReader() { TestParameterReader(""); }

  TestParameterReader(std::string test_prefix)
  {
    int_map_ = {
      {test_prefix + "retention_period", 2},
      {test_prefix + "streaming_type", 0},
      {test_prefix + "max_latency", 0},
      {test_prefix + "fragment_duration", 2},
      {test_prefix + "timecode_scale", 1},
      {test_prefix + "nal_adaptation_flags",
       NAL_ADAPTATION_ANNEXB_NALS | NAL_ADAPTATION_ANNEXB_CPD_NALS},
      {test_prefix + "frame_rate", 24},
      {test_prefix + "avg_bandwidth_bps", 4 * 1024 * 1024},
      {test_prefix + "buffer_duration", 120},
      {test_prefix + "replay_duration", 40},
      {test_prefix + "connection_staleness", 30},
    };
    bool_map_ = {
      {test_prefix + "key_frame_fragmentation", true}, {test_prefix + "frame_timecodes", true},
      {test_prefix + "absolute_fragment_time", true},  {test_prefix + "fragment_acks", true},
      {test_prefix + "restart_on_error", true},        {test_prefix + "recalculate_metrics", true},
    };
    string_map_ = {
      {test_prefix + "stream_name", "testStream"},   {test_prefix + "kms_key_id", ""},
      {test_prefix + "content_type", "video/h264"},  {test_prefix + "codec_id", "V_MPEG4/ISO/AVC"},
      {test_prefix + "track_name", "kinesis_video"},
    };
    map_map_ = {
      {test_prefix + "tags", {{"someKey", "someValue"}}},
    };
  }

  TestParameterReader(std::map<std::string, int> int_map, std::map<std::string, bool> bool_map,
                      std::map<std::string, std::string> string_map, std::map<std::string, std::map<std::string, std::string>> map_map)
  : int_map_(int_map), bool_map_(bool_map), string_map_(string_map), map_map_(map_map)
  {
  }

  AwsError ReadParam(const ParameterPath & param_path, int & out) const override
  {
    std::string name = FormatParameterPath(param_path);
    if (int_map_.count(name) > 0) {
      out = int_map_.at(name);
      return AWS_ERR_OK;
    }
    return AWS_ERR_NOT_FOUND;
  }

  AwsError ReadParam(const ParameterPath & param_path, bool & out) const
  {
    std::string name = FormatParameterPath(param_path);
    if (bool_map_.count(name) > 0) {
      out = bool_map_.at(name);
      return AWS_ERR_OK;
    }
    return AWS_ERR_NOT_FOUND;
  }

  AwsError ReadParam(const ParameterPath & param_path, std::string & out) const
  {
    std::string name = FormatParameterPath(param_path);
    if (string_map_.count(name) > 0) {
      out = string_map_.at(name);
      return AWS_ERR_OK;
    }
    return AWS_ERR_NOT_FOUND;
  }

  AwsError ReadParam(const ParameterPath & param_path, Aws::String & out) const
  {
    return AWS_ERR_EMPTY;
  }

  AwsError ReadParam(const ParameterPath & param_path, std::map<std::string, std::string> & out) const
  {
    std::string name = FormatParameterPath(param_path);
    if (map_map_.count(name) > 0) {
      out = map_map_.at(name);
      return AWS_ERR_OK;
    }
    return AWS_ERR_NOT_FOUND;
  }

  AwsError ReadParam(const ParameterPath & param_path, std::vector<std::string> & out) const
  {
    return AWS_ERR_EMPTY;
  }

  AwsError ReadParam(const ParameterPath & param_path, double & out) const
  {
    return AWS_ERR_EMPTY;
  }

  std::map<std::string, int> int_map_;
  std::map<std::string, bool> bool_map_;
  std::map<std::string, std::string> string_map_;
  std::map<std::string, std::map<std::string, std::string>> map_map_;

private:
  std::string FormatParameterPath(const ParameterPath & param_path) const
  {
    return param_path.get_resolved_path('/', '/');
  }
};

struct MockStreamManager : public KinesisStreamManagerInterface
{
  TestData * data_;
  MockStreamManager(TestData * data) { data_ = data; }

  MockStreamManager(TestData * data, TestParameterReader * parameter_reader,
                    StreamDefinitionProvider * stream_definition_provider,
                    StreamSubscriptionInstaller * subscription_installer)
  : KinesisStreamManagerInterface(parameter_reader, stream_definition_provider,
                                  subscription_installer)
  {
    data_ = data;
  }

  KinesisManagerStatus InitializeVideoProducer(
    std::string region, std::unique_ptr<DeviceInfoProvider> device_info_provider,
    std::unique_ptr<ClientCallbackProvider> client_callback_provider,
    std::unique_ptr<StreamCallbackProvider> stream_callback_provider,
    std::unique_ptr<CredentialProvider> credential_provider,
    VideoProducerFactory video_producer_factory) override
  {
    data_->initialize_video_producer_call_count++;
    return data_->initialize_video_producer_return_value;
  }

  KinesisManagerStatus InitializeVideoProducer(std::string region,
    VideoProducerFactory video_producer_factory) override
  {
    data_->initialize_video_producer_call_count++;
    return data_->initialize_video_producer_return_value;
  }

  KinesisManagerStatus InitializeVideoStream(
    std::unique_ptr<StreamDefinition> stream_definition) override
  {
    data_->initialize_video_stream_call_count++;
    return data_->initialize_video_stream_return_value;
  }

  KinesisManagerStatus PutFrame(std::string stream_name, Frame & frame) const override
  {
    data_->put_frame_call_count++;
    return data_->put_frame_return_value;
  }

  KinesisManagerStatus PutMetadata(std::string stream_name, const std::string & name,
                                   const std::string & value) const
  {
    data_->put_metadata_call_count++;
    return data_->put_metadata_return_value;
  }

  void FreeStream(std::string stream_name) override { data_->free_stream_call_count++; }

  KinesisManagerStatus KinesisVideoStreamerSetup() override
  {
    return KinesisStreamManagerInterface::KinesisVideoStreamerSetup();
  }

  KinesisManagerStatus ProcessCodecPrivateDataForStream(
    const std::string & stream_name, std::vector<uint8_t> codec_private_data) override
  {
    data_->process_codec_private_data_call_count++;
    return data_->process_codec_private_data_return_value;
  }

  KinesisManagerStatus FetchRekognitionResults(const std::string & stream_name,
                                               Aws::Vector<Model::Record> * records) override
  {
    data_->fetch_rekognition_results_call_count++;
    return KINESIS_MANAGER_STATUS_SUCCESS;
  }

  KinesisManagerStatus GenerateStreamSubscriptionDescriptor(
    int stream_idx, StreamSubscriptionDescriptor & descriptor) override
  {
    return KinesisStreamManagerInterface::GenerateStreamSubscriptionDescriptor(stream_idx,
                                                                               descriptor);
  }

  KinesisManagerStatus InitializeStreamSubscription(
    const StreamSubscriptionDescriptor & descriptor) override
  {
    return subscription_installer_->Install(descriptor);
  }
};
