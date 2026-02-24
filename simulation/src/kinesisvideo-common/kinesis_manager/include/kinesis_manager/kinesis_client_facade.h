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

#include <aws/core/utils/Outcome.h>
#include <aws/kinesis/KinesisClient.h>
#include <aws/kinesis/model/GetRecordsRequest.h>
#include <aws/kinesis/model/GetRecordsResult.h>
#include <aws_common/sdk_utils/throttling_manager.h>

namespace Aws {
namespace Kinesis {

/**
 * Kinesis client extension which uses the ClientThrottlingManager to throttle calls to GetRecords.
 */
class KinesisClientFacade : public KinesisClient, Client::ThrottlingManager
{
public:
  KinesisClientFacade(Aws::Client::ClientConfiguration config)
  : KinesisClient(config), Client::ThrottlingManager()
  {
    this->Client::ThrottlingManager::SetMaxApiTps(
      "GetRecords",
      5.0); /* Default of 5 calls GetRecords per second assuming a single shard iterator. */
  }
  /**
   * GetRecords
   * @refitem KinesisClient::GetRecords
   */
  Model::GetRecordsOutcome GetRecords(const Model::GetRecordsRequest & request) const override
  {
    auto get_records_base =
      [this](const Model::GetRecordsRequest & request) -> Model::GetRecordsOutcome {
      return this->KinesisClient::GetRecords(request);
    };
    return MakeCall<Model::GetRecordsOutcome, Model::GetRecordsRequest, KinesisErrors>(
      get_records_base, request, __func__, KinesisErrors::PROVISIONED_THROUGHPUT_EXCEEDED, true);
  }
  /**
   * Add additional overrides for every API you which to throttle and update max_api_tps_
   * accordingly.
   */
};
}  // namespace Kinesis
}  // namespace Aws
