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
/**
 * @brief This module provides default callbacks for use with the Kinesis Producer Library.
 *  You can implement your own custom callbacks or use the default ones declared here.
 * @see https://docs.aws.amazon.com/kinesisvideostreams/latest/dg/producer-reference-callbacks.html
 */
#pragma once
#include <aws/core/auth/AWSCredentialsProviderChain.h>
#include <kinesis-video-producer/Auth.h>
#include <kinesis-video-producer/ClientCallbackProvider.h>
#include <kinesis-video-producer/DefaultCallbackProvider.h>
#include <kinesis-video-producer/DefaultDeviceInfoProvider.h>

#define AWS_ACCESS_KEY_ENV_VAR "AWS_ACCESS_KEY_ID"
#define AWS_SECRET_KEY_ENV_VAR "AWS_SECRET_ACCESS_KEY"
#define AWS_SESSION_TOKEN_ENV_VAR "AWS_SESSION_TOKEN"
#define AWS_DEFAULT_CREDENTIAL_ROTATION_PERIOD_IN_SECONDS (2400)


namespace Aws {
namespace Kinesis {

/**
 * Creates a static CredentialProvider holding a single credential pair with expiration set to
 * AWS_DEFAULT_CREDENTIAL_ROTATION_PERIOD_IN_SECONDS.. The credentials will be loaded from
 * environment variables matching the AWS SDK.
 * @return CredentialProvider
 */
std::unique_ptr<com::amazonaws::kinesis::video::CredentialProvider> CreateDefaultCredentialProvider();
/**
 * Credentials provider which uses the AWS SDK's default credential provider chain.
 * @note You need to have called Aws::InitAPI before using this provider.
 */
class ProducerSdkAWSCredentialsProvider : public com::amazonaws::kinesis::video::CredentialProvider
{
public:
  ProducerSdkAWSCredentialsProvider(std::shared_ptr<Auth::DefaultAWSCredentialsProviderChain>
                                      default_aws_credentials_provider = nullptr)
  {
    if (default_aws_credentials_provider) {
      default_aws_credentials_provider_ = default_aws_credentials_provider;
    } else {
      default_aws_credentials_provider_ =
        Aws::MakeShared<Auth::DefaultAWSCredentialsProviderChain>(__func__);
    }
  }

private:
  std::shared_ptr<Auth::DefaultAWSCredentialsProviderChain> default_aws_credentials_provider_;

  void updateCredentials(com::amazonaws::kinesis::video::Credentials & producer_sdk_credentials) override
  {
    Auth::AWSCredentials aws_sdk_credentials =
      default_aws_credentials_provider_->GetAWSCredentials();
    producer_sdk_credentials.setAccessKey(aws_sdk_credentials.GetAWSAccessKeyId().c_str());
    producer_sdk_credentials.setSecretKey(aws_sdk_credentials.GetAWSSecretKey().c_str());
    producer_sdk_credentials.setSessionToken(aws_sdk_credentials.GetSessionToken().c_str());
    auto now = std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::system_clock::now().time_since_epoch());
    auto refresh_interval = std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::milliseconds(Auth::REFRESH_THRESHOLD));
    producer_sdk_credentials.setExpiration(now + refresh_interval);
  }
};

class DefaultClientCallbackProvider : public com::amazonaws::kinesis::video::ClientCallbackProvider
{
public:
  UINT64 getCallbackCustomData() override;
  StorageOverflowPressureFunc getStorageOverflowPressureCallback() override;
  static STATUS storageOverflowPressure(UINT64 custom_handle, UINT64 remaining_bytes);
};

class DefaultStreamCallbackProvider : public com::amazonaws::kinesis::video::StreamCallbackProvider
{
public:
  UINT64 getCallbackCustomData() override;
  StreamConnectionStaleFunc getStreamConnectionStaleCallback() override;
  StreamErrorReportFunc getStreamErrorReportCallback() override;
  DroppedFrameReportFunc getDroppedFrameReportCallback() override;

private:
  static STATUS streamConnectionStaleHandler(UINT64 custom_data, STREAM_HANDLE stream_handle,
                                             UINT64 last_buffering_ack);
  static STATUS streamErrorReportHandler(UINT64 custom_data, STREAM_HANDLE stream_handle,
                                         UPLOAD_HANDLE upload_handle, UINT64 errored_timecode,
                                         STATUS status_code);
  static STATUS droppedFrameReportHandler(UINT64 custom_data, STREAM_HANDLE stream_handle,
                                          UINT64 dropped_frame_timecode);
};

}  // namespace Kinesis
}  // namespace Aws