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
#include <aws/core/client/AWSError.h>

#include <chrono>
#include <functional>
#include <mutex>
#include <unordered_map>

namespace Aws {
namespace Client {

/**
 * This class implements non-blocking client-side throttling; inherit from it and use MakeCall. For
 * usage example see KinesisClientFacade of the kinesis_manager package.
 * @note typically, the bucket size for each API is unknown to the client, so this implements linear
 * (average) throttling rather than leaky bucket.
 */
class ThrottlingManager
{
protected:
  /**
   * MakeCall - API throttling wrapper
   * @tparam T API response object, typically Aws::Utils::Outcome.
   * @tparam U API request object, e.g. GetRecordsRequest.
   * @tparam E the class to be used for the throttling error, e.g. KinesisErrors.
   * @param api The SDK API function to call.
   * @param api_param parameter to pass along to the API call.
   * @param api_name name of the API function, e.g. "GetRecords".
   * @param error_on_throttling_args the arguments required to construct an object of class E.
   * @return T or T(E(error_on_throttling_args)) if additional wait time is needed before making the
   * call.
   */
  template <class T, class U, class E, typename... ErrorArgs>
  T MakeCall(std::function<T(const U & request)> api, const U & api_param, std::string api_name,
             ErrorArgs... error_on_throttling_args) const
  {
    if (0 == max_api_tps_.count(api_name)) {
      return api(api_param);
    }
    /* Min delta is 1s / TPS, e.g. 5 TPS -> 200ms wait time between calls */
    auto min_delta_allowed = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::seconds(1) / max_api_tps_.at(api_name))
                               .count();
    std::lock_guard<std::mutex> lock(api_call_time_mutex_);
    {
      if (0 == last_call_time_per_api_.count(api_name)) {
        last_call_time_per_api_.insert(std::make_pair(
          api_name, std::chrono::time_point<std::chrono::steady_clock>(std::chrono::seconds(0))));
      }
      auto now = std::chrono::steady_clock::now();
      auto delta_in_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                           now - last_call_time_per_api_.at(api_name))
                           .count();
      if (delta_in_ms >= min_delta_allowed) {
        last_call_time_per_api_.at(api_name) = now;
        return api(api_param);
      } else {
        T outcome(Aws::Client::AWSError<E>(error_on_throttling_args...));
        return outcome;
      }
    }
  }
  /**
   * Sets the call rate limit for an API.
   * @param api name of the API function
   * @param tps max number of calls per second
   */
  void SetMaxApiTps(const std::string & api, double tps) { max_api_tps_[api] = tps; }

private:
  std::unordered_map<std::string, double> max_api_tps_;
  mutable std::mutex api_call_time_mutex_;
  mutable std::unordered_map<std::string, std::chrono::time_point<std::chrono::steady_clock>>
    last_call_time_per_api_;
};

}  // namespace Client
}  // namespace Aws