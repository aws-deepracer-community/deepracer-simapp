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

#include <aws/core/Aws.h>
#include <aws_common/sdk_utils/aws_error.h>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <kinesis-video-producer/KinesisVideoProducer.h>
#include <kinesis-video-producer/Logger.h>
#include <kinesis_manager/common.h>
#include <kinesis_manager/kinesis_stream_manager.h>
#include <kinesis_manager/stream_definition_provider.h>
#include <aws/kinesis/model/GetShardIteratorRequest.h>
#include <aws/kinesis/model/ListShardsRequest.h>

using namespace std;
using namespace com::amazonaws::kinesis::video;
using namespace Aws;
using namespace Aws::Client;
using namespace Aws::Kinesis;

LOGGER_TAG("aws.kinesis.kinesis_manager_unittest");

#define PARAM_NS_SEPARATOR "/"
#define PARAM_NS_SEPARATOR_CHAR '/'

using namespace std;
using namespace std::chrono;
using namespace Aws;
using namespace Aws::Kinesis;
using namespace com::amazonaws::kinesis::video;
using ::testing::NiceMock;
using ::testing::_;
using ::testing::A;
using ::testing::Return;
using ::testing::Eq;
using ::testing::StrEq;
using ::testing::InSequence;
using ::testing::DoAll;
using ::testing::SetArgReferee;
using ::testing::ContainerEq;
using Aws::AwsError;

/**
 * Parameter reader that sets the output using provided std::mapS.
 */
class TestParameterReader : public ParameterReaderInterface
{
public:
  TestParameterReader(string test_prefix)
  {
    int_map_ = {
      {test_prefix + PARAM_NS_SEPARATOR "retention_period", 2},
      {test_prefix + PARAM_NS_SEPARATOR "streaming_type", 0},
      {test_prefix + PARAM_NS_SEPARATOR "max_latency", 0},
      {test_prefix + PARAM_NS_SEPARATOR "fragment_duration", 2},
      {test_prefix + PARAM_NS_SEPARATOR "timecode_scale", 1},
      {test_prefix + PARAM_NS_SEPARATOR "nal_adaptation_flags",
       NAL_ADAPTATION_ANNEXB_NALS | NAL_ADAPTATION_ANNEXB_CPD_NALS},
      {test_prefix + PARAM_NS_SEPARATOR"frame_rate", 24},
      {test_prefix + PARAM_NS_SEPARATOR "avg_bandwidth_bps", 4 * 1024 * 1024},
      {test_prefix + PARAM_NS_SEPARATOR "buffer_duration", 120},
      {test_prefix + PARAM_NS_SEPARATOR "replay_duration", 40},
      {test_prefix + PARAM_NS_SEPARATOR "connection_staleness", 30},
    };
    bool_map_ = {
      {test_prefix + PARAM_NS_SEPARATOR "key_frame_fragmentation", true},
      {test_prefix + PARAM_NS_SEPARATOR "frame_timecodes", true},
      {test_prefix + PARAM_NS_SEPARATOR "absolute_fragment_time", true},
      {test_prefix + PARAM_NS_SEPARATOR "fragment_acks", true},
      {test_prefix + PARAM_NS_SEPARATOR "restart_on_error", true},
      {test_prefix + PARAM_NS_SEPARATOR "recalculate_metrics", true},
    };
    string_map_ = {
      {test_prefix + PARAM_NS_SEPARATOR "stream_name", "testStream"},
      {test_prefix + PARAM_NS_SEPARATOR "kms_key_id", ""},
      {test_prefix + PARAM_NS_SEPARATOR "content_type", "video/h264"},
      {test_prefix + PARAM_NS_SEPARATOR "codec_id", "V_MPEG4/ISO/AVC"},
      {test_prefix + PARAM_NS_SEPARATOR "track_name", "kinesis_video"},
    };
    map_map_ = {
      {test_prefix + PARAM_NS_SEPARATOR "tags", {{"someKey", "someValue"}}},
    };
  }

  TestParameterReader(const vector<string> & test_prefix)
  {
      TestParameterReader(Join(test_prefix));
  }

  TestParameterReader(map<string, int> int_map, map<string, bool> bool_map,
                      map<string, string> string_map, map<string, map<string, string>> map_map)
  : int_map_(int_map), bool_map_(bool_map), string_map_(string_map), map_map_(map_map)
  {
  }

  string Join(const vector<string> & test_prefix)
  {
      string expanded;
      for (auto item = test_prefix.begin(); item != test_prefix.end(); item++) {
          expanded += *item + PARAM_NS_SEPARATOR;
      }
      if (!expanded.empty() && expanded.back() == PARAM_NS_SEPARATOR_CHAR) {
          expanded.pop_back();
      }
      return expanded;
  }

  AwsError ReadParam(const ParameterPath & param_path, int & out) const
  {
    AwsError result = AWS_ERR_NOT_FOUND;
    std::string name = FormatParameterPath(param_path);
    if (int_map_.count(name) > 0) {
      out = int_map_.at(name);
      result = AWS_ERR_OK;
    }
    return result;
  }

  AwsError ReadParam(const ParameterPath & param_path, bool & out) const
  {
    AwsError result = AWS_ERR_NOT_FOUND;
    std::string name = FormatParameterPath(param_path);
    if (bool_map_.count(name) > 0) {
      out = bool_map_.at(name);
      result = AWS_ERR_OK;
    }
    return result;
  }

  AwsError ReadParam(const ParameterPath & param_path, string & out) const
  {
    AwsError result = AWS_ERR_NOT_FOUND;
    std::string name = FormatParameterPath(param_path);
    if (string_map_.count(name) > 0) {
      out = string_map_.at(name);
      result = AWS_ERR_OK;
    }
    return result;
  }

  AwsError ReadParam(const ParameterPath & param_path, Aws::String & out) const
  {
    return AWS_ERR_EMPTY;
  }

  AwsError ReadParam(const ParameterPath & param_path, map<string, string> & out) const
  {
    AwsError result = AWS_ERR_NOT_FOUND;
    std::string name = FormatParameterPath(param_path);
    if (map_map_.count(name) > 0) {
      out = map_map_.at(name);
      result = AWS_ERR_OK;
    }
    return result;
  }

  AwsError ReadParam(const ParameterPath & param_path, std::vector<std::string> & out) const
  {
    return AWS_ERR_EMPTY;
  }

  AwsError ReadParam(const ParameterPath & param_path, double & out) const { return AWS_ERR_EMPTY; }

  map<string, int> int_map_;
  map<string, bool> bool_map_;
  map<string, string> string_map_;
  map<string, map<string, string>> map_map_;


  static string DoFormatParameterPath(const ParameterPath & param_path)
  {
    return param_path.get_resolved_path(PARAM_NS_SEPARATOR_CHAR, PARAM_NS_SEPARATOR_CHAR);
  }

private:
  string FormatParameterPath(const ParameterPath & param_path) const
  {
    return DoFormatParameterPath(param_path);
  }
};

/**
 * Tests stream definitions for equivalence.
 * @param stream1
 * @param stream2
 * @return true if the streams are equivalent, false otherwise.
 */
static bool are_streams_equivalent(unique_ptr<StreamDefinition> stream1,
                                   unique_ptr<StreamDefinition> stream2)
{
  bool result = true;
  StreamInfo stream1_info = stream1->getStreamInfo();
  StreamInfo stream2_info = stream2->getStreamInfo();
  /**
   * Compare complex structures first
   */
  if (stream1_info.streamCaps.trackInfoList[0].codecPrivateDataSize !=
      stream2_info.streamCaps.trackInfoList[0].codecPrivateDataSize) {
    return false;
  } else {
    result &= (0 == memcmp((void *)&(stream1_info.streamCaps.trackInfoList[0].codecPrivateData),
                           (void *)&(stream2_info.streamCaps.trackInfoList[0].codecPrivateData),
                           stream1_info.streamCaps.trackInfoList[0].codecPrivateDataSize));
  }
  if (stream1_info.tagCount != stream2_info.tagCount) {
    return false;
  } else {
    for (int tag_idx = 0; tag_idx < stream1_info.tagCount; tag_idx++) {
      result &= (stream1_info.tags[tag_idx].version == stream2_info.tags[tag_idx].version);
      result &= (0 == strncmp(stream1_info.tags[tag_idx].name, stream2_info.tags[tag_idx].name,
                              MAX_TAG_NAME_LEN));
      result &= (0 == strncmp(stream1_info.tags[tag_idx].value, stream2_info.tags[tag_idx].value,
                              MAX_TAG_VALUE_LEN));
    }
  }
  /**
   * Zero out pointers contained within the structs and use memcmp.
   */
  stream1_info.streamCaps.trackInfoList = nullptr;
  stream2_info.streamCaps.trackInfoList = nullptr;
  stream1_info.tags = nullptr;
  stream2_info.tags = nullptr;
  result &= (0 == memcmp((void *)&(stream1_info), (void *)&(stream2_info), sizeof(stream1_info)));
  return result;
}

/**
 * Initializes the video producer and generates a basic stream definition.
 */
unique_ptr<StreamDefinition> DefaultProducerSetup(
  KinesisStreamManager & stream_manager,
  string region, string test_prefix, std::shared_ptr<ParameterReaderInterface> parameter_reader,
  KinesisStreamManagerInterface::VideoProducerFactory video_producer_factory)
{
#ifdef PLATFORM_TESTING_ACCESS_KEY
  setenv("AWS_ACCESS_KEY_ID", PLATFORM_TESTING_ACCESS_KEY, 1);
#endif
#ifdef PLATFORM_TESTING_SECRET_KEY
  setenv("AWS_SECRET_ACCESS_KEY", PLATFORM_TESTING_SECRET_KEY, 1);
#endif
  stream_manager.InitializeVideoProducer(region, video_producer_factory);

  StreamDefinitionProvider stream_definition_provider;
  unique_ptr<StreamDefinition> stream_definition = stream_definition_provider.GetStreamDefinition(
    ParameterPath(test_prefix.c_str()), *parameter_reader, nullptr, 0);
  return move(stream_definition);
}

/**
 * Initializes the video producer and generates a basic stream definition.
 */
unique_ptr<StreamDefinition> DefaultProducerSetup(
  KinesisStreamManager & stream_manager,
  string region, string test_prefix,
  KinesisStreamManagerInterface::VideoProducerFactory video_producer_factory)
{
   std::shared_ptr<ParameterReaderInterface> parameter_reader =
    std::make_shared<TestParameterReader>(test_prefix);
  return DefaultProducerSetup(stream_manager, region, test_prefix, parameter_reader, video_producer_factory);
}

/**
 * Mock class for Aws::Kinesis::KinesisClient, fully functional as all it's methods are virtual.
 */
class KinesisClientMock : public KinesisClient
{
public:
  MOCK_CONST_METHOD0(GetServiceClientName, const char *());
  MOCK_CONST_METHOD1(ListShards, Model::ListShardsOutcome(const Model::ListShardsRequest&));
  MOCK_CONST_METHOD1(GetShardIterator,
    Model::GetShardIteratorOutcome(const Model::GetShardIteratorRequest&));
  MOCK_CONST_METHOD1(GetRecords,
    Model::GetRecordsOutcome(const Model::GetRecordsRequest&));
};

class KinesisVideoStreamMock : public KinesisVideoStreamInterface
{
public:
  MOCK_CONST_METHOD0(IsReady, bool());
  MOCK_METHOD0(Stop, bool());
  MOCK_CONST_METHOD1(PutFrame, bool(KinesisVideoFrame));
  MOCK_METHOD3(PutFragmentMetadata, bool(const std::string&, const std::string&, bool));
};

class KinesisVideoProducerMock : public KinesisVideoProducerInterface
{
public:
  std::shared_ptr<KinesisVideoStreamInterface> CreateStreamSync(std::unique_ptr<StreamDefinition> stream_definition) {
    return CreateStreamSyncProxy(stream_definition.get());
  }
  MOCK_METHOD1(CreateStreamSyncProxy,
    std::shared_ptr<KinesisVideoStreamInterface>(StreamDefinition* stream_definition));
  MOCK_METHOD1(FreeStream, void(std::shared_ptr<KinesisVideoStreamInterface> kinesis_video_stream));
};

namespace Aws {
namespace Kinesis {

namespace Model
{
  bool operator==(const Record & left, const Record & right)
  {
    bool result = true;

    result &= (left.GetSequenceNumber() == right.GetSequenceNumber());
    result &= (left.GetApproximateArrivalTimestamp() == right.GetApproximateArrivalTimestamp());
    result &= (left.GetData() == right.GetData());
    result &= (left.GetPartitionKey() == right.GetPartitionKey());
    result &= (left.GetEncryptionType() == right.GetEncryptionType());

    return true;
  }

}  // namespace Model
}  // namespace Kinesis
}  // namespace Aws

class StreamSubscriptionInstallerMock : public StreamSubscriptionInstaller
{
public:
  MOCK_CONST_METHOD1(Install, KinesisManagerStatus(const StreamSubscriptionDescriptor & descriptor));
  MOCK_METHOD1(Uninstall, void(const std::string & topic_name));
};

class StreamDefinitionProviderPartialMock : public StreamDefinitionProvider
{
public:
  MOCK_CONST_METHOD4(GetCodecPrivateData,
    KinesisManagerStatus(const ParameterPath &, const ParameterReaderInterface &, PBYTE *, uint32_t *));
};

class StreamDefinitionProviderFullMock: public StreamDefinitionProvider
{
public:
  MOCK_CONST_METHOD4(GetCodecPrivateData,
    KinesisManagerStatus(const ParameterPath &, const ParameterReaderInterface &, PBYTE *, uint32_t *));

  MOCK_CONST_METHOD4(GetStreamDefinitionProxy,
    StreamDefinition*(const ParameterPath &, const ParameterReaderInterface &, const PBYTE, uint32_t));

  unique_ptr<StreamDefinition> GetStreamDefinition(const ParameterPath & prefix,
    const ParameterReaderInterface & reader, const PBYTE codec_private_data,
    uint32_t codec_private_data_size) const override
    {
      StreamDefinition* stream_definition = GetStreamDefinitionProxy(prefix, reader,
        codec_private_data, codec_private_data_size);
      return std::unique_ptr<StreamDefinition>(stream_definition);
    }
};

KinesisStreamManagerInterface::VideoProducerFactory ConstVideoProducerFactory(
  unique_ptr<KinesisVideoProducerInterface> video_producer)
  {
    return [& video_producer](
        std::string region,
        unique_ptr<com::amazonaws::kinesis::video::DeviceInfoProvider> device_info_provider,
        unique_ptr<com::amazonaws::kinesis::video::ClientCallbackProvider> client_callback_provider,
        unique_ptr<com::amazonaws::kinesis::video::StreamCallbackProvider> stream_callback_provider,
        unique_ptr<com::amazonaws::kinesis::video::CredentialProvider> credential_provider
      ) -> unique_ptr<KinesisVideoProducerInterface> {
        return std::move(video_producer);
      };
  }

class KinesisStreamManagerMockingFixture : public ::testing::Test
{
public:
  KinesisStreamManagerMockingFixture()
  {
    parameter_reader_ = std::make_shared<TestParameterReader>(int_map_, bool_map_, string_map_, map_map_);
  }

protected:
  string test_prefix_ = "some/test/prefix";
  string encoded_string_ = "aGVsbG8gd29ybGQ=";
  map<string, int> int_map_ = {};
  map<string, bool> bool_map_ = {};
  map<string, string> tags_;
  map<string, map<string, string>> map_map_ = {};
  map<string, string> string_map_ = {
    {test_prefix_ + "codecPrivateData", encoded_string_},
  };

  std::shared_ptr<ParameterReaderInterface> parameter_reader_;

  StreamDefinitionProvider stream_definition_provider_;
  StreamSubscriptionInstallerMock subscription_installer_ ;
};

TEST_F(KinesisStreamManagerMockingFixture, testPutMetadataNotInitialized)
{
  std::unique_ptr<NiceMock<KinesisClientMock>> kinesis_client = std::unique_ptr<NiceMock<KinesisClientMock>>{};
  KinesisStreamManager stream_manager(parameter_reader_.get(), & stream_definition_provider_,
    & subscription_installer_, std::move(kinesis_client));
  std::string stream_name = "stream_name1";
  std::string metadata_name = "metadata_name";
  std::string metadata_value = "metadata_value";

  auto status = stream_manager.PutMetadata(stream_name, metadata_name, metadata_value);

  ASSERT_TRUE(KINESIS_MANAGER_STATUS_FAILED(status));
}

TEST_F(KinesisStreamManagerMockingFixture, testPutMetadataStreamNotReady)
{
  KinesisStreamManager stream_manager;
  std::string test_prefix = "kinesis_video";
  std::shared_ptr<ParameterReaderInterface> parameter_reader = std::make_shared<TestParameterReader>(test_prefix);
  std::string stream_name;
  parameter_reader->ReadParam(GetKinesisVideoParameter(kStreamParameters.stream_name), stream_name);
  std::string metadata_name = "metadata_name";
  std::string metadata_value = "metadata_value";
  auto video_producer = std::make_unique<KinesisVideoProducerMock>();
  auto video_stream_mock = std::make_shared<KinesisVideoStreamMock>();

  EXPECT_CALL(*video_producer.get(), CreateStreamSyncProxy(_))
    .WillOnce(Return(video_stream_mock));
  EXPECT_CALL(*video_stream_mock, IsReady())
    .WillOnce(Return(false));

  auto stream_definition = DefaultProducerSetup(stream_manager, std::string("us-west-2"), test_prefix,
    ConstVideoProducerFactory(std::move(video_producer)));

  auto status = stream_manager.InitializeVideoStream(std::move(stream_definition));
  EXPECT_EQ(KINESIS_MANAGER_STATUS_SUCCESS, status);

  status = stream_manager.PutMetadata(stream_name, metadata_name, metadata_value);
  ASSERT_EQ(KINESIS_MANAGER_STATUS_PUTMETADATA_FAILED, status);
}

TEST_F(KinesisStreamManagerMockingFixture, testPutMetadataSuccess)
{
  KinesisStreamManager stream_manager;
  std::string test_prefix = "kinesis_video";
  std::shared_ptr<ParameterReaderInterface> parameter_reader = std::make_shared<TestParameterReader>(test_prefix);
  std::string stream_name;
  parameter_reader->ReadParam(GetKinesisVideoParameter(kStreamParameters.stream_name), stream_name);
  std::string metadata_name = "metadata_name";
  std::string metadata_value = "metadata_value";
  auto video_producer = std::make_unique<KinesisVideoProducerMock>();
  auto video_stream_mock = std::make_shared<KinesisVideoStreamMock>();

  EXPECT_CALL(*video_producer.get(), CreateStreamSyncProxy(_))
    .WillOnce(Return(video_stream_mock));

  auto status = stream_manager.InitializeVideoProducer(string("us-west-2"),
    ConstVideoProducerFactory(std::move(video_producer)));
  EXPECT_EQ(KINESIS_MANAGER_STATUS_SUCCESS, status);

  ON_CALL(*video_stream_mock, IsReady())
    .WillByDefault(Return(true));
  {
    InSequence video_stream_mock_seq;

    EXPECT_CALL(*video_stream_mock, PutFragmentMetadata(StrEq(metadata_name), StrEq(metadata_value), _))
      .WillOnce(Return(false));

    EXPECT_CALL(*video_stream_mock, PutFragmentMetadata(StrEq(metadata_name), StrEq(metadata_value), _))
      .WillOnce(Return(true));
  }

  auto stream_definition = DefaultProducerSetup(stream_manager, std::string("us-west-2"), test_prefix,
    ConstVideoProducerFactory(std::move(video_producer)));
  status = stream_manager.InitializeVideoStream(std::move(stream_definition));
  EXPECT_EQ(KINESIS_MANAGER_STATUS_SUCCESS, status);

  status = stream_manager.PutMetadata(stream_name, metadata_name, metadata_value);
  ASSERT_TRUE(KINESIS_MANAGER_STATUS_FAILED(status));
  status = stream_manager.PutMetadata(stream_name, metadata_name, metadata_value);
  ASSERT_TRUE(KINESIS_MANAGER_STATUS_SUCCEEDED(status));
}

TEST_F(KinesisStreamManagerMockingFixture, testFreeStream)
{
  KinesisStreamManager stream_manager;
  std::string test_prefix = "kinesis_video";
  std::shared_ptr<ParameterReaderInterface> parameter_reader = std::make_shared<TestParameterReader>(test_prefix);
  std::string stream_name;
  parameter_reader->ReadParam(GetKinesisVideoParameter(kStreamParameters.stream_name), stream_name);
  auto video_producer = std::make_unique<KinesisVideoProducerMock>();
  auto video_stream_mock = std::make_shared<KinesisVideoStreamMock>();

  EXPECT_CALL(*video_producer.get(), CreateStreamSyncProxy(_))
    .WillOnce(Return(video_stream_mock));
  EXPECT_CALL(*video_producer.get(), FreeStream(_)).Times(1);

  auto stream_definition = DefaultProducerSetup(stream_manager, std::string("us-west-2"), test_prefix,
    ConstVideoProducerFactory(std::move(video_producer)));
  auto status = stream_manager.InitializeVideoStream(std::move(stream_definition));
  EXPECT_EQ(KINESIS_MANAGER_STATUS_SUCCESS, status);

  ON_CALL(*video_stream_mock, IsReady())
    .WillByDefault(Return(true));
  EXPECT_CALL(*video_stream_mock, Stop()).Times(1);

  stream_manager.FreeStream(stream_name);
}

TEST_F(KinesisStreamManagerMockingFixture, testProcessCodecPrivateDataForStreamKinesisVideoStreamSetupFailure)
{
  std::string stream_name = "stream_name1";
  std::string topic_name = "topic1";
  std::vector<uint8_t> codec_private_data = {1,2,3};
  int stream_idx = 0;
  int stream_count_param = 1;
  map<string, int> int_map = {
    {TestParameterReader::DoFormatParameterPath(GetKinesisVideoParameter(kStreamParameters.stream_count)),
      stream_count_param}
  };
  map<string, bool> bool_map;
  map<string, string> string_map = {
    {TestParameterReader::DoFormatParameterPath(GetStreamParameterPath(stream_idx, kStreamParameters.stream_name)),
      stream_name},
    {TestParameterReader::DoFormatParameterPath(GetStreamParameterPath(stream_idx, kStreamParameters.topic_name)),
      topic_name}
  };
  map<string, map<string, string>> map_map;
  TestParameterReader parameter_reader(int_map, bool_map, string_map, map_map);

  StreamDefinitionProviderFullMock stream_definition_provider;
  std::unique_ptr<NiceMock<KinesisClientMock>> kinesis_client = std::unique_ptr<NiceMock<KinesisClientMock>>{};
  KinesisStreamManager stream_manager(&parameter_reader,
    & stream_definition_provider, & subscription_installer_, std::move(kinesis_client));

  // force failure on KinesisVideoStreamSetup, and thus recovery path
  EXPECT_CALL(stream_definition_provider, GetStreamDefinitionProxy(_,_,_,_))
    .WillOnce(Return(nullptr));

  EXPECT_CALL(subscription_installer_, Uninstall(StrEq(topic_name)))
    .Times(1);

  auto status = stream_manager.ProcessCodecPrivateDataForStream(stream_name, codec_private_data);

  ASSERT_TRUE(KINESIS_MANAGER_STATUS_FAILED(status));
}

TEST_F(KinesisStreamManagerMockingFixture, testKinesisVideoStreamSetupZeroStreamCount)
{
  map<string, int> int_map = {
    {TestParameterReader::DoFormatParameterPath(GetKinesisVideoParameter(kStreamParameters.stream_count)), 0}
  };
  auto parameter_reader = std::make_shared<TestParameterReader>(int_map, bool_map_, string_map_, map_map_);
  std::unique_ptr<NiceMock<KinesisClientMock>> kinesis_client = std::unique_ptr<NiceMock<KinesisClientMock>>{};
  KinesisStreamManager stream_manager(parameter_reader.get(), & stream_definition_provider_,
    & subscription_installer_, std::move(kinesis_client));

  auto status = stream_manager.KinesisVideoStreamerSetup();

  ASSERT_TRUE(KINESIS_MANAGER_STATUS_FAILED(status));
}

TEST_F(KinesisStreamManagerMockingFixture, testKinesisVideoStreamSetupSingleStreamFailsGetCodecPrivateData)
{
  map<string, int> int_map = {
    {TestParameterReader::DoFormatParameterPath(GetKinesisVideoParameter(kStreamParameters.stream_count)), 1}
  };
  auto parameter_reader = std::make_shared<TestParameterReader>(int_map, bool_map_, string_map_, map_map_);
  StreamDefinitionProviderPartialMock stream_definition_provider;
  std::unique_ptr<NiceMock<KinesisClientMock>> kinesis_client = std::unique_ptr<NiceMock<KinesisClientMock>>{};
  KinesisStreamManager stream_manager(parameter_reader.get(), & stream_definition_provider,
    & subscription_installer_, std::move(kinesis_client));

  EXPECT_CALL(stream_definition_provider, GetCodecPrivateData(_,_,_,_))
    .WillOnce(Return(KINESIS_MANAGER_STATUS_ERROR_BASE));

  auto status = stream_manager.KinesisVideoStreamerSetup();

  ASSERT_TRUE(KINESIS_MANAGER_STATUS_FAILED(status));
}

TEST_F(KinesisStreamManagerMockingFixture, testKinesisVideoStreamSetupAndFetchRekognitionResultsSingleStreamSuccessful)
{
  int stream_idx = 0;
  std::string stream_name = "stream_name1";
  map<string, int> int_map = {
    {TestParameterReader::DoFormatParameterPath(GetKinesisVideoParameter(kStreamParameters.stream_count)), 1},
    {TestParameterReader::DoFormatParameterPath(GetStreamParameterPath(stream_idx, kStreamParameters.topic_type)), 42}
  };
  map<string, string> string_map = {
    {TestParameterReader::DoFormatParameterPath(GetStreamParameterPath(stream_idx, kStreamParameters.topic_name)),
      "foo"},
    {TestParameterReader::DoFormatParameterPath(GetStreamParameterPath(stream_idx, kStreamParameters.stream_name)),
      stream_name},
    {TestParameterReader::DoFormatParameterPath(GetStreamParameterPath(stream_idx, kStreamParameters.rekognition_data_stream)),
      "rekognition_data_stream"},
    {TestParameterReader::DoFormatParameterPath(GetStreamParameterPath(stream_idx, kStreamParameters.rekognition_topic_name)),
      "rekognition_topic_name"},
  };

  TestParameterReader parameter_reader(int_map, bool_map_, string_map, map_map_);
  Aws::Vector<Model::Record> kinesis_records;
  StreamDefinitionProviderPartialMock stream_definition_provider;
  // This takes almost 20 seconds due to call to parent class with no AWS credentials available
  auto kinesis_client = std::make_unique<NiceMock<KinesisClientMock>>();
  NiceMock<KinesisClientMock> * kinesis_client_p = kinesis_client.get();
  KinesisStreamManager stream_manager(&parameter_reader, & stream_definition_provider,
    & subscription_installer_, std::move(kinesis_client));
  auto video_producer = std::make_unique<KinesisVideoProducerMock>();
  auto video_stream_mock = std::make_shared<KinesisVideoStreamMock>();

  EXPECT_CALL(*video_producer.get(), CreateStreamSyncProxy(_))
    .WillOnce(Return(video_stream_mock));

  stream_manager.InitializeVideoProducer(string("us-west-2"), ConstVideoProducerFactory(std::move(video_producer)));
  EXPECT_CALL(stream_definition_provider, GetCodecPrivateData(_,_,_,_))
    .WillOnce(Return(KINESIS_MANAGER_STATUS_SUCCESS));
  EXPECT_CALL(subscription_installer_, Install(_))
    .WillOnce(Return(KINESIS_MANAGER_STATUS_SUCCESS));

  auto fetch_status_not_configured = stream_manager.FetchRekognitionResults(stream_name, &kinesis_records);

  EXPECT_TRUE(KINESIS_MANAGER_STATUS_SUCCEEDED(fetch_status_not_configured));

  auto setup_status = stream_manager.KinesisVideoStreamerSetup();

  ASSERT_TRUE(KINESIS_MANAGER_STATUS_SUCCEEDED(setup_status));

  Model::ListShardsResult list_shards_result;
  Model::Shard shard1;
  shard1.SetShardId("shard1Id");
  list_shards_result.SetShards({shard1});
  Model::ListShardsOutcome list_shards_outcome(list_shards_result);

  EXPECT_CALL(*kinesis_client_p, ListShards(_))
    .WillRepeatedly(Return(list_shards_outcome));
  Model::GetShardIteratorResult get_shard_iterator_result;
  get_shard_iterator_result.SetShardIterator("shardIterator");
  Model::GetShardIteratorOutcome get_shard_iterator_outcome(get_shard_iterator_result);
  EXPECT_CALL(*kinesis_client_p, GetShardIterator(_))
    .WillRepeatedly(Return(get_shard_iterator_outcome));

  Model::Record record1;
  record1.SetSequenceNumber("seq_number1");
  Aws::Vector<Model::Record> expected_kinesis_records = {record1};
  Model::GetRecordsResult get_records_result;
  get_records_result.SetRecords(expected_kinesis_records);
  {
    InSequence get_records_seq;

    Model::GetRecordsOutcome get_records_outcome_ok(get_records_result);
    EXPECT_CALL(*kinesis_client_p, GetRecords(_))
      .WillOnce(Return(get_records_outcome_ok));

    AWSError<KinesisErrors> get_records_error1(Aws::Kinesis::KinesisErrors::PROVISIONED_THROUGHPUT_EXCEEDED, true);
    Model::GetRecordsOutcome get_records_outcome_error1(get_records_error1);
    EXPECT_CALL(*kinesis_client_p, GetRecords(_))
      .WillOnce(Return(get_records_outcome_error1));

    AWSError<KinesisErrors> get_records_error2(Aws::Kinesis::KinesisErrors::EXPIRED_ITERATOR, true);
    Model::GetRecordsOutcome get_records_outcome_error2(get_records_error2);
    EXPECT_CALL(*kinesis_client_p, GetRecords(_))
      .WillOnce(Return(get_records_outcome_error2));

    AWSError<KinesisErrors> get_records_error3(Aws::Kinesis::KinesisErrors::ACCESS_DENIED, true);
    Model::GetRecordsOutcome get_records_outcome_error3(get_records_error3);
    EXPECT_CALL(*kinesis_client_p, GetRecords(_))
      .WillOnce(Return(get_records_outcome_error3));
  }

  auto fetch_status = stream_manager.FetchRekognitionResults(stream_name, &kinesis_records);

  ASSERT_TRUE(KINESIS_MANAGER_STATUS_SUCCEEDED(fetch_status));
  ASSERT_EQ(expected_kinesis_records, kinesis_records);
  ASSERT_THAT(kinesis_records, ContainerEq(expected_kinesis_records));

  fetch_status = stream_manager.FetchRekognitionResults(stream_name, &kinesis_records);

  ASSERT_TRUE(KINESIS_MANAGER_STATUS_FAILED(fetch_status));

  fetch_status = stream_manager.FetchRekognitionResults(stream_name, &kinesis_records);

  ASSERT_TRUE(KINESIS_MANAGER_STATUS_FAILED(fetch_status));

  fetch_status = stream_manager.FetchRekognitionResults(stream_name, &kinesis_records);

  ASSERT_TRUE(KINESIS_MANAGER_STATUS_FAILED(fetch_status));
}

TEST_F(KinesisStreamManagerMockingFixture, mockStreamInitializationTestActualKinesisVideoProducer)
{
  std::unique_ptr<NiceMock<KinesisClientMock>> kinesis_client = std::unique_ptr<NiceMock<KinesisClientMock>>{};
  KinesisStreamManager stream_manager(parameter_reader_.get(), & stream_definition_provider_,
    & subscription_installer_, std::move(kinesis_client));

  /* Before calling InitializeVideoProducer */
  KinesisManagerStatus status =
    stream_manager.InitializeVideoStream(move(unique_ptr<StreamDefinition>()));
  ASSERT_TRUE(KINESIS_MANAGER_STATUS_FAILED(status) &&
              KINESIS_MANAGER_STATUS_VIDEO_PRODUCER_NOT_INITIALIZED == status);

  ASSERT_FALSE(stream_manager.get_video_producer());
  unique_ptr<StreamDefinition> stream_definition =
    DefaultProducerSetup(stream_manager, string("us-west-2"), string("stream/test"), parameter_reader_,
      // this takes almost 20 seconds because an actual client is created without AWS credentials available
      KinesisStreamManagerInterface::CreateDefaultVideoProducer);
  ASSERT_TRUE(stream_manager.get_video_producer());

  /* Video producer has been created but the stream definition is empty. */
  status = stream_manager.InitializeVideoStream(unique_ptr<StreamDefinition>{});
  ASSERT_TRUE(KINESIS_MANAGER_STATUS_FAILED(status) &&
              KINESIS_MANAGER_STATUS_INVALID_INPUT == status);
}

TEST_F(KinesisStreamManagerMockingFixture, mockStreamInitializationTestKinesisVideoProducerMock)
{
  KinesisStreamManager  stream_manager;
  ASSERT_FALSE(stream_manager.get_video_producer());
  auto video_producer = std::make_unique<KinesisVideoProducerMock>();
  auto video_stream_mock = std::make_shared<KinesisVideoStreamMock>();

  EXPECT_CALL(*video_producer.get(), CreateStreamSyncProxy(_))
    .WillOnce(Return(video_stream_mock));

  auto stream_definition =
    DefaultProducerSetup(stream_manager, string("us-west-2"), string("stream/test"), parameter_reader_,
      ConstVideoProducerFactory(std::move(video_producer)));
  ASSERT_TRUE(stream_manager.get_video_producer());

  /* Video producer has been created but the stream definition is empty. */
  KinesisManagerStatus status = stream_manager.InitializeVideoStream(unique_ptr<StreamDefinition>{});
  EXPECT_TRUE(KINESIS_MANAGER_STATUS_FAILED(status));
  ASSERT_EQ(KINESIS_MANAGER_STATUS_INVALID_INPUT, status);

  std::string stream_name = "stream_name1";
  status = stream_manager.InitializeVideoStream(move(stream_definition));
  ASSERT_TRUE(KINESIS_MANAGER_STATUS_SUCCEEDED(status));
}

TEST_F(KinesisStreamManagerMockingFixture, mockPutFrameTest)
{
  KinesisStreamManager stream_manager;
  auto video_producer = std::make_unique<KinesisVideoProducerMock>();
  auto video_stream_mock = std::make_shared<KinesisVideoStreamMock>();
  Frame frame;
  string stream_name("testStream");

  EXPECT_CALL(*video_producer.get(), CreateStreamSyncProxy(_))
    .WillOnce(Return(video_stream_mock));

  /* Before calling InitializeVideoProducer */
  KinesisManagerStatus status = stream_manager.PutFrame(stream_name, frame);
  EXPECT_TRUE(KINESIS_MANAGER_STATUS_FAILED(status));
  ASSERT_EQ(KINESIS_MANAGER_STATUS_VIDEO_PRODUCER_NOT_INITIALIZED ,status);

  /* Stream name not found (i.e. before calling InitializeVideoStream) */
  auto stream_definition =
    DefaultProducerSetup(stream_manager, string("us-west-2"), string("frame/test"),
      ConstVideoProducerFactory(std::move(video_producer)));
  status = stream_manager.PutFrame(string(stream_name), frame);
  EXPECT_TRUE(KINESIS_MANAGER_STATUS_FAILED(status));
  ASSERT_EQ(KINESIS_MANAGER_STATUS_PUTFRAME_STREAM_NOT_FOUND, status);

  status = stream_manager.InitializeVideoStream(move(stream_definition));
  ASSERT_TRUE(KINESIS_MANAGER_STATUS_SUCCEEDED(status));

  {
    InSequence video_stream_mock_seq;

    EXPECT_CALL(*video_stream_mock, IsReady())
      .WillOnce(Return(false));

    EXPECT_CALL(*video_stream_mock, IsReady())
      .WillOnce(Return(true));

    EXPECT_CALL(*video_stream_mock, PutFrame(_))
      .WillOnce(Return(false));

    EXPECT_CALL(*video_stream_mock, IsReady())
      .WillOnce(Return(true));

    EXPECT_CALL(*video_stream_mock, PutFrame(_))
      .WillOnce(Return(true));
  }

  // not ready
  status = stream_manager.PutFrame(stream_name, frame);
  ASSERT_TRUE(KINESIS_MANAGER_STATUS_FAILED(status) &&
              KINESIS_MANAGER_STATUS_PUTFRAME_FAILED == status);

  // ready but putFrame fails
  status = stream_manager.PutFrame(stream_name, frame);
  ASSERT_TRUE(KINESIS_MANAGER_STATUS_FAILED(status));

  // ready and putFrame ok
  status = stream_manager.PutFrame(stream_name, frame);
  ASSERT_TRUE(KINESIS_MANAGER_STATUS_SUCCEEDED(status));
}

/**
 * Tests that GetCodecPrivateData successfully reads and decodes the given base64-encoded buffer.
 */
TEST(StreamDefinitionProviderSuite, getCodecPrivateDataTest)
{
  string test_prefix = "some/test/prefix";
  vector<string> test_prefix_list{"some", "test", "prefix"};
  Aws::Kinesis::StreamDefinitionProvider stream_definition_provider;

  string decoded_string = "hello world";
  string encoded_string = "aGVsbG8gd29ybGQ=";
  map<string, int> int_map = {};
  map<string, bool> bool_map = {};
  map<string, string> tags;
  map<string, map<string, string>> map_map = {};
  map<string, string> string_map = {
    {test_prefix + PARAM_NS_SEPARATOR "codecPrivateData", encoded_string},
  };
  TestParameterReader parameter_reader(int_map, bool_map, string_map, map_map);

  PBYTE codec_private_data;
  uint32_t codec_private_data_size;
  ASSERT_TRUE(KINESIS_MANAGER_STATUS_SUCCEEDED(stream_definition_provider.GetCodecPrivateData(
              ParameterPath(test_prefix_list), parameter_reader, &codec_private_data,
              &codec_private_data_size)));
  ASSERT_EQ(decoded_string.length(), codec_private_data_size);
  ASSERT_TRUE(0 == strncmp(decoded_string.c_str(), (const char *)codec_private_data,
                           codec_private_data_size));

  /* Invalid input tests */
  KinesisManagerStatus status = stream_definition_provider.GetCodecPrivateData(
              ParameterPath(test_prefix_list), parameter_reader, nullptr, &codec_private_data_size);
  ASSERT_TRUE(KINESIS_MANAGER_STATUS_FAILED(status) &&
              KINESIS_MANAGER_STATUS_INVALID_INPUT == status);
  status = stream_definition_provider.GetCodecPrivateData(
    ParameterPath(test_prefix_list), parameter_reader, &codec_private_data, nullptr);
  ASSERT_TRUE(KINESIS_MANAGER_STATUS_FAILED(status) &&
              KINESIS_MANAGER_STATUS_INVALID_INPUT == status);

  /* Empty input */
  string_map = {};
  TestParameterReader empty_parameter_reader(int_map, bool_map, string_map, map_map);
  codec_private_data = nullptr;
  ASSERT_TRUE(KINESIS_MANAGER_STATUS_SUCCEEDED(stream_definition_provider.GetCodecPrivateData(
              ParameterPath(test_prefix_list), empty_parameter_reader, &codec_private_data,
              &codec_private_data_size)) && !codec_private_data);

  /* Dependency failure */
  string_map = {
    {test_prefix + PARAM_NS_SEPARATOR "codecPrivateData", "1"},
  };
  TestParameterReader parameter_reader_with_invalid_values(int_map, bool_map, string_map, map_map);
  status = stream_definition_provider.GetCodecPrivateData(ParameterPath(test_prefix_list),
            parameter_reader_with_invalid_values, &codec_private_data, &codec_private_data_size);
  ASSERT_TRUE(KINESIS_MANAGER_STATUS_FAILED(status) &&
              KINESIS_MANAGER_STATUS_BASE64DECODE_FAILED == status);
}

/**
 * Tests that GetStreamDefinition returns the expected StreamDefinition object by comparing it to a
 * manually created StreamDefinition.
 */
TEST(StreamDefinitionProviderSuite, getStreamDefinitionTest)
{
  string test_prefix = "some/test/prefix";
  vector<string> test_prefix_list{"some", "test", "prefix"};
  Aws::Kinesis::StreamDefinitionProvider stream_definition_provider;

  TestParameterReader parameter_reader = TestParameterReader(test_prefix);
  map<string, string> string_map = parameter_reader.string_map_;
  map<string, bool> bool_map = parameter_reader.bool_map_;
  map<string, int> int_map = parameter_reader.int_map_;
  map<string, map<string, string>> map_map = parameter_reader.map_map_;

  unique_ptr<StreamDefinition> generated_stream_definition =
    stream_definition_provider.GetStreamDefinition(ParameterPath(test_prefix_list),
                                                   parameter_reader, nullptr, 0);
  auto equivalent_stream_definition = make_unique<StreamDefinition>(
    string_map[test_prefix + PARAM_NS_SEPARATOR "stream_name"],
    hours(int_map[test_prefix + PARAM_NS_SEPARATOR "retention_period"]),
    &map_map[test_prefix + PARAM_NS_SEPARATOR "tags"],
    string_map[test_prefix + PARAM_NS_SEPARATOR "kms_key_id"],
    static_cast<STREAMING_TYPE>(int_map[test_prefix + PARAM_NS_SEPARATOR "streaming_type"]),
    string_map[test_prefix + PARAM_NS_SEPARATOR "content_type"],
    milliseconds(int_map[test_prefix + PARAM_NS_SEPARATOR "max_latency"]),
    seconds(int_map[test_prefix + PARAM_NS_SEPARATOR "fragment_duration"]),
    milliseconds(int_map[test_prefix + PARAM_NS_SEPARATOR "timecode_scale"]),
    bool_map[test_prefix + PARAM_NS_SEPARATOR "key_frame_fragmentation"],
    bool_map[test_prefix + PARAM_NS_SEPARATOR "frame_timecodes"],
    bool_map[test_prefix + PARAM_NS_SEPARATOR "absolute_fragment_time"],
    bool_map[test_prefix + PARAM_NS_SEPARATOR "fragment_acks"],
    bool_map[test_prefix + PARAM_NS_SEPARATOR "restart_on_error"],
    bool_map[test_prefix + PARAM_NS_SEPARATOR "recalculate_metrics"],
    static_cast<NAL_ADAPTATION_FLAGS>(int_map[test_prefix + PARAM_NS_SEPARATOR "nal_adaptation_flags"]),
    int_map[test_prefix + PARAM_NS_SEPARATOR "frame_rate"],
    int_map[test_prefix + PARAM_NS_SEPARATOR "avg_bandwidth_bps"],
    seconds(int_map[test_prefix + PARAM_NS_SEPARATOR "buffer_duration"]),
    seconds(int_map[test_prefix + PARAM_NS_SEPARATOR "replay_duration"]),
    seconds(int_map[test_prefix + PARAM_NS_SEPARATOR "connection_staleness"]),
    string_map[test_prefix + PARAM_NS_SEPARATOR "codec_id"],
    string_map[test_prefix + PARAM_NS_SEPARATOR "track_name"], nullptr, 0);
  ASSERT_TRUE(
    are_streams_equivalent(move(equivalent_stream_definition), move(generated_stream_definition)));

  auto different_stream_definition = make_unique<StreamDefinition>(
    string_map[test_prefix + PARAM_NS_SEPARATOR "stream_name"],
    hours(int_map[test_prefix + PARAM_NS_SEPARATOR "retention_period"]),
    &map_map[test_prefix + PARAM_NS_SEPARATOR "tags"],
    string_map[test_prefix + PARAM_NS_SEPARATOR "kms_key_id"],
    static_cast<STREAMING_TYPE>(int_map[test_prefix + PARAM_NS_SEPARATOR "streaming_type"]),
    string_map[test_prefix + PARAM_NS_SEPARATOR "content_type"],
    milliseconds(int_map[test_prefix + PARAM_NS_SEPARATOR "max_latency"]),
    seconds(int_map[test_prefix + PARAM_NS_SEPARATOR "fragment_duration"]),
    milliseconds(int_map[test_prefix + PARAM_NS_SEPARATOR "timecode_scale"]),
    bool_map[test_prefix + PARAM_NS_SEPARATOR "key_frame_fragmentation"],
    bool_map[test_prefix + PARAM_NS_SEPARATOR "frame_timecodes"],
    bool_map[test_prefix + PARAM_NS_SEPARATOR "absolute_fragment_time"],
    bool_map[test_prefix + PARAM_NS_SEPARATOR "fragment_acks"],
    bool_map[test_prefix + PARAM_NS_SEPARATOR "restart_on_error"],
    bool_map[test_prefix + PARAM_NS_SEPARATOR "recalculate_metrics"],
    static_cast<NAL_ADAPTATION_FLAGS>(int_map[test_prefix + PARAM_NS_SEPARATOR "nal_adaptation_flags"]), 4914918,
    int_map[test_prefix + PARAM_NS_SEPARATOR "avg_bandwidth_bps"],
    seconds(int_map[test_prefix + PARAM_NS_SEPARATOR "buffer_duration"]),
    seconds(int_map[test_prefix + PARAM_NS_SEPARATOR "replay_duration"]),
    seconds(int_map[test_prefix + PARAM_NS_SEPARATOR "connection_staleness"]),
    string_map[test_prefix + PARAM_NS_SEPARATOR "codec_id"],
    string_map[test_prefix + PARAM_NS_SEPARATOR "track_name"], nullptr, 0);
  generated_stream_definition = stream_definition_provider.GetStreamDefinition(
    ParameterPath(test_prefix_list), parameter_reader, nullptr, 0);
  ASSERT_FALSE(
    are_streams_equivalent(move(different_stream_definition), move(generated_stream_definition)));

  /* Invalid input tests */
  generated_stream_definition = stream_definition_provider.GetStreamDefinition(
    ParameterPath(test_prefix_list), parameter_reader, nullptr, 100);
  ASSERT_FALSE(generated_stream_definition);
}

/**
 * Initializes the video producer and generates a basic stream definition.
 */
unique_ptr<StreamDefinition> DefaultProducerSetup(
  Aws::Kinesis::KinesisStreamManager & stream_manager, string region, const std::vector<std::string> & test_prefix)
{
#ifdef PLATFORM_TESTING_ACCESS_KEY
  setenv("AWS_ACCESS_KEY_ID", PLATFORM_TESTING_ACCESS_KEY, 1);
#endif
#ifdef PLATFORM_TESTING_SECRET_KEY
  setenv("AWS_SECRET_ACCESS_KEY", PLATFORM_TESTING_SECRET_KEY, 1);
#endif
  stream_manager.InitializeVideoProducer(region);

  Aws::Kinesis::StreamDefinitionProvider stream_definition_provider;
  TestParameterReader parameter_reader = TestParameterReader(test_prefix);
  unique_ptr<StreamDefinition> stream_definition = stream_definition_provider.GetStreamDefinition(
    ParameterPath(test_prefix), parameter_reader, nullptr, 0);
  return move(stream_definition);
}

/**
 * Tests the InitializeVideoProducer function.
 */
TEST(KinesisStreamManagerSuite, videoInitializationTest)
{
  string test_prefix = "some/test/prefix";
  Aws::Kinesis::KinesisStreamManager stream_manager;

  /* Empty region */
  KinesisManagerStatus status = stream_manager.InitializeVideoProducer("");
  ASSERT_TRUE(KINESIS_MANAGER_STATUS_FAILED(status) &&
              KINESIS_MANAGER_STATUS_INVALID_INPUT == status);

  /* Non empty region, invalid callback/info providers */
  status = stream_manager.InitializeVideoProducer("us-west-2", nullptr, nullptr, nullptr, nullptr);
  ASSERT_TRUE(KINESIS_MANAGER_STATUS_FAILED(status) &&
              KINESIS_MANAGER_STATUS_INVALID_INPUT == status);

  status = stream_manager.InitializeVideoProducer("us-west-2");
  ASSERT_TRUE(KINESIS_MANAGER_STATUS_SUCCEEDED(status));
  ASSERT_TRUE(stream_manager.get_video_producer());

  /* Duplicate initialization */
  auto video_producer = stream_manager.get_video_producer();
  status = stream_manager.InitializeVideoProducer("us-west-2");
  ASSERT_TRUE(KINESIS_MANAGER_STATUS_FAILED(status) &&
              KINESIS_MANAGER_STATUS_VIDEO_PRODUCER_ALREADY_INITIALIZED == status);
  auto video_producer_post_call = stream_manager.get_video_producer();
  ASSERT_EQ(video_producer, video_producer_post_call);
}

#ifdef BUILD_AWS_TESTING
// the following tests perform AWS API calls and require user confiugration
// to enable them run: colcon build --cmake-args -DBUILD_AWS_TESTING=1

/**
 * Tests the InitializeVideoStream function. This will attempt to create and load a test stream in
 * the test account.
 */
TEST(KinesisStreamManagerSuite, streamInitializationTest)
{
  Aws::Kinesis::KinesisStreamManager stream_manager;
  /* Before calling InitializeVideoProducer */
  KinesisManagerStatus status =
    stream_manager.InitializeVideoStream(move(unique_ptr<StreamDefinition>()));
  ASSERT_TRUE(KINESIS_MANAGER_STATUS_FAILED(status) &&
              KINESIS_MANAGER_STATUS_VIDEO_PRODUCER_NOT_INITIALIZED == status);

  ASSERT_FALSE(stream_manager.get_video_producer());
  unique_ptr<StreamDefinition> stream_definition =
    DefaultProducerSetup(stream_manager, string("us-west-2"), string("stream/test"));
  ASSERT_TRUE(stream_manager.get_video_producer());

  /* Video producer has been created but the stream definition is empty. */
  status = stream_manager.InitializeVideoStream(unique_ptr<StreamDefinition>{});
  ASSERT_TRUE(KINESIS_MANAGER_STATUS_FAILED(status) &&
              KINESIS_MANAGER_STATUS_INVALID_INPUT == status);

  status = stream_manager.InitializeVideoStream(move(stream_definition));
  ASSERT_TRUE(KINESIS_MANAGER_STATUS_SUCCEEDED(status));
}

/**
 * Tests the PutFrame function. This will load the test stream and attempt to transmit a dummy frame
 * to it.
 */
TEST(KinesisStreamManagerSuite, putFrameTest)
{
  Aws::Kinesis::KinesisStreamManager stream_manager;
  Frame frame;
  string stream_name("testStream");
  /* Before calling InitializeVideoProducer */
  KinesisManagerStatus status = stream_manager.PutFrame(stream_name, frame);
  ASSERT_TRUE(KINESIS_MANAGER_STATUS_FAILED(status) &&
              KINESIS_MANAGER_STATUS_VIDEO_PRODUCER_NOT_INITIALIZED == status);

  /* Stream name not found (i.e. before calling InitializeVideoStream) */
  unique_ptr<StreamDefinition> stream_definition =
    DefaultProducerSetup(stream_manager, string("us-west-2"), string("frame/test"));
  status = stream_manager.PutFrame(string(stream_name), frame);
  ASSERT_TRUE(KINESIS_MANAGER_STATUS_FAILED(status) &&
              KINESIS_MANAGER_STATUS_PUTFRAME_STREAM_NOT_FOUND == status);

  status = stream_manager.InitializeVideoStream(move(stream_definition));
  ASSERT_TRUE(KINESIS_MANAGER_STATUS_SUCCEEDED(status));

  /* Invalid frame */
  frame.size = 0;
  status = stream_manager.PutFrame(stream_name, frame);
  ASSERT_TRUE(KINESIS_MANAGER_STATUS_FAILED(status) &&
              KINESIS_MANAGER_STATUS_PUTFRAME_FAILED == status);

  /* Valid (but dummy) frame */
  frame.size = 4;
  std::vector<uint8_t> bytes = {0x00, 0x01, 0x02, 0x03};
  frame.frameData = reinterpret_cast<PBYTE>((void *)(bytes.data()));
  frame.duration = 5000000;
  frame.index = 1;
  UINT64 timestamp = 0;
  timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count() /
              DEFAULT_TIME_UNIT_IN_NANOS;
  frame.decodingTs = timestamp;
  frame.presentationTs = timestamp;
  frame.flags = (FRAME_FLAGS)0;

  status = stream_manager.PutFrame(stream_name, frame);
  ASSERT_TRUE(KINESIS_MANAGER_STATUS_SUCCEEDED(status));
}
#endif

int main(int argc, char ** argv)
{
  LOG_CONFIGURE_STDOUT("ERROR");
  Aws::SDKOptions options;
  Aws::InitAPI(options);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
