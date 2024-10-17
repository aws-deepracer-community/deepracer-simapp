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
#include <aws_common/sdk_utils/client_configuration_provider.h>
#include <com/amazonaws/kinesis/video/utils/Include.h>
#include <kinesis_manager/stream_definition_provider.h>

using namespace com::amazonaws::kinesis::video;
using namespace Aws::Client;


#define STREAM_DEFINITION_MAX_CODEC_PRIVATE_DATA_SIZE (1024)


namespace Aws {
namespace Kinesis {

using namespace std;
using namespace std::chrono;

KinesisManagerStatus StreamDefinitionProvider::GetCodecPrivateData(
  const ParameterPath & prefix, const ParameterReaderInterface & reader,
  PBYTE * out_codec_private_data, uint32_t * out_codec_private_data_size) const
{
  if (nullptr == out_codec_private_data ||
      nullptr == out_codec_private_data_size) {
    return KINESIS_MANAGER_STATUS_INVALID_INPUT;
  }
  std::string b64_encoded_codec_private_data;
  reader.ReadParam(prefix + "codecPrivateData", b64_encoded_codec_private_data);
  if (!b64_encoded_codec_private_data.empty()) {
    uint8_t temp_codec_data[STREAM_DEFINITION_MAX_CODEC_PRIVATE_DATA_SIZE] = {0};
    uint32_t decoded_buffer_size = sizeof(temp_codec_data);
    if (STATUS_SUCCESS != base64Decode(const_cast<char *>(b64_encoded_codec_private_data.c_str()),
                                       temp_codec_data, &decoded_buffer_size)) {
      return KINESIS_MANAGER_STATUS_BASE64DECODE_FAILED;
    }
    PBYTE codec_private_data = (PBYTE)malloc(decoded_buffer_size);
    if (nullptr == codec_private_data) {
      return KINESIS_MANAGER_STATUS_MALLOC_FAILED;
    }
    memset(codec_private_data, 0, decoded_buffer_size);
    memcpy(codec_private_data, temp_codec_data, decoded_buffer_size);
    *out_codec_private_data = codec_private_data;
    *out_codec_private_data_size = decoded_buffer_size;
  }
  return KINESIS_MANAGER_STATUS_SUCCESS;
}

unique_ptr<StreamDefinition> StreamDefinitionProvider::GetStreamDefinition(
    const ParameterPath & prefix, const ParameterReaderInterface & reader,
    const PBYTE codec_private_data, uint32_t codec_private_data_size) const
{
  if (nullptr == codec_private_data && 0 != codec_private_data_size) {
    return unique_ptr<StreamDefinition>{};
  }

  std::string stream_name = "default";
  reader.ReadParam(prefix + "stream_name", stream_name);

  map<string, string> tags;
  reader.ReadParam(prefix + "tags", tags);

  int retention_period = 2;
  reader.ReadParam(prefix + "retention_period", retention_period);

  std::string kms_key_id;
  reader.ReadParam(prefix + "kms_key_id", kms_key_id);

  int streaming_type_id = 0;
  reader.ReadParam(prefix + "streaming_type", streaming_type_id);
  STREAMING_TYPE streaming_type = static_cast<STREAMING_TYPE>(streaming_type_id);

  std::string content_type = "video/h264";
  reader.ReadParam(prefix + "content_type", content_type);


  int max_latency = 0;
  reader.ReadParam(prefix + "max_latency", max_latency);

  int fragment_duration = 2;
  reader.ReadParam(prefix + "fragment_duration", fragment_duration);

  int timecode_scale = 1;
  reader.ReadParam(prefix + "timecode_scale", timecode_scale);

  bool key_frame_fragmentation = true;
  reader.ReadParam(prefix + "key_frame_fragmentation", key_frame_fragmentation);

  bool frame_timecodes = true;
  reader.ReadParam(prefix + "frame_timecodes", frame_timecodes);

  bool absolute_fragment_time = true;
  reader.ReadParam(prefix + "absolute_fragment_time", absolute_fragment_time);

  bool fragment_acks = true;
  reader.ReadParam(prefix + "fragment_acks", fragment_acks);

  bool restart_on_error = true;
  reader.ReadParam(prefix + "restart_on_error", restart_on_error);

  bool recalculate_metrics = true;
  reader.ReadParam(prefix + "recalculate_metrics", recalculate_metrics);

  int nal_adaptation_flag_id = NAL_ADAPTATION_ANNEXB_NALS | NAL_ADAPTATION_ANNEXB_CPD_NALS;
  reader.ReadParam(prefix + "nal_adaptation_flags", nal_adaptation_flag_id);
  NAL_ADAPTATION_FLAGS nal_adaptation_flags =
    static_cast<NAL_ADAPTATION_FLAGS>(nal_adaptation_flag_id);

  int frame_rate = 24;
  reader.ReadParam(prefix + "frame_rate", frame_rate);

  int avg_bandwidth_bps = 4 * 1024 * 1024;
  reader.ReadParam(prefix + "avg_bandwidth_bps", avg_bandwidth_bps);

  int buffer_duration = 120;
  reader.ReadParam(prefix + "buffer_duration", buffer_duration);

  int replay_duration = 40;
  reader.ReadParam(prefix + "replay_duration", replay_duration);

  int connection_staleness = 30;
  reader.ReadParam(prefix + "connection_staleness", connection_staleness);

  std::string codec_id = "V_MPEG4/ISO/AVC";
  reader.ReadParam(prefix + "codec_id", codec_id);

  std::string track_name = "kinesis_video";
  reader.ReadParam(prefix + "track_name", track_name);


  auto stream_definition = make_unique<StreamDefinition>(
    stream_name, hours(retention_period), &tags, kms_key_id, streaming_type, content_type,
    milliseconds(max_latency), seconds(fragment_duration), milliseconds(timecode_scale),
    key_frame_fragmentation, frame_timecodes, absolute_fragment_time, fragment_acks,
    restart_on_error, recalculate_metrics, nal_adaptation_flags, frame_rate, avg_bandwidth_bps,
    seconds(buffer_duration), seconds(replay_duration), seconds(connection_staleness), codec_id,
    track_name, codec_private_data, codec_private_data_size);
  return stream_definition;
}
}  // namespace Kinesis
}  // namespace Aws
