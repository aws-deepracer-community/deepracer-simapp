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
#include <kinesis_manager/default_callbacks.h>

using namespace std;
using namespace com::amazonaws::kinesis::video;


namespace Aws {
namespace Kinesis {

/*******************************
 * Client callbacks
 *******************************/
UINT64 DefaultClientCallbackProvider::getCallbackCustomData()
{
  return reinterpret_cast<UINT64>(this);
}
StorageOverflowPressureFunc DefaultClientCallbackProvider::getStorageOverflowPressureCallback()
{
  return storageOverflowPressure;
}
STATUS DefaultClientCallbackProvider::storageOverflowPressure(UINT64 custom_handle,
                                                              UINT64 remaining_bytes)
{
  UNUSED_PARAM(custom_handle);
  return STATUS_SUCCESS;
}

/*******************************
 * Credentials setup
 *******************************/
unique_ptr<CredentialProvider> CreateDefaultCredentialProvider()
{
  char const *access_key, *secret_key, *session_token;
  if (nullptr == (access_key = getenv(AWS_ACCESS_KEY_ENV_VAR))) {
    return unique_ptr<CredentialProvider>{};
  }
  if (nullptr == (secret_key = getenv(AWS_SECRET_KEY_ENV_VAR))) {
    return unique_ptr<CredentialProvider>{};
  }
  if (nullptr == (session_token = getenv(AWS_SESSION_TOKEN_ENV_VAR))) {
    session_token = "";
  }

  auto now_time = std::chrono::duration_cast<std::chrono::seconds>(
    std::chrono::system_clock::now().time_since_epoch());
  unique_ptr<Credentials> credentials_ = make_unique<Credentials>(
    string(access_key), string(secret_key), string(session_token),
    now_time + std::chrono::seconds(AWS_DEFAULT_CREDENTIAL_ROTATION_PERIOD_IN_SECONDS));
  return make_unique<StaticCredentialProvider>(*credentials_.get());
}

/*******************************
 * Stream callbacks
 *******************************/
UINT64 DefaultStreamCallbackProvider::getCallbackCustomData()
{
  return reinterpret_cast<UINT64>(this);
}

StreamConnectionStaleFunc DefaultStreamCallbackProvider::getStreamConnectionStaleCallback()
{
  return streamConnectionStaleHandler;
};

StreamErrorReportFunc DefaultStreamCallbackProvider::getStreamErrorReportCallback()
{
  return streamErrorReportHandler;
};

DroppedFrameReportFunc DefaultStreamCallbackProvider::getDroppedFrameReportCallback()
{
  return droppedFrameReportHandler;
};

STATUS DefaultStreamCallbackProvider::streamConnectionStaleHandler(UINT64 custom_data,
                                                                   STREAM_HANDLE stream_handle,
                                                                   UINT64 last_buffering_ack)
{
  AWS_LOGSTREAM_WARN(__func__, "Reporting stream stale. Last ACK received " << last_buffering_ack);
  return STATUS_SUCCESS;
}

STATUS
DefaultStreamCallbackProvider::streamErrorReportHandler(UINT64 custom_data,
                                                        STREAM_HANDLE stream_handle,
                                                        UPLOAD_HANDLE upload_handle,
                                                        UINT64 errored_timecode,
                                                        STATUS status_code)
{
  AWS_LOGSTREAM_ERROR(__func__, "Reporting stream error. Errored timecode: "
                                  << errored_timecode << " Status: " << status_code);
  return STATUS_SUCCESS;
}

STATUS
DefaultStreamCallbackProvider::droppedFrameReportHandler(UINT64 custom_data,
                                                         STREAM_HANDLE stream_handle,
                                                         UINT64 dropped_frame_timecode)
{
  AWS_LOGSTREAM_WARN(__func__,
                     "Reporting dropped frame. Frame timecode " << dropped_frame_timecode);
  return STATUS_SUCCESS;
}

}  // namespace Kinesis
}  // namespace Aws