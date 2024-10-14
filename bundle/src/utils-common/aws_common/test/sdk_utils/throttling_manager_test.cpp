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
#include <aws/core/utils/Outcome.h>
#include <aws_common/sdk_utils/throttling_manager.h>
#include <gtest/gtest.h>

#include <atomic>
#include <thread>
#include <cmath>

enum class DummyClientErrors { THROTTLING_ERROR };
typedef Aws::Utils::Outcome<int, Aws::Client::AWSError<DummyClientErrors>> DummyOutcome;

class BaseClient
{
public:
  virtual DummyOutcome ThrottledFunction(int number) const
  {
    throttled_function_call_count_++;
    return DummyOutcome(number);
  }
  mutable std::atomic<int> throttled_function_call_count_{0};
};

class ThrottledClient : public BaseClient, Aws::Client::ThrottlingManager
{
public:
  explicit ThrottledClient(double max_api_tps)
  {
    Aws::Client::ThrottlingManager::SetMaxApiTps("ThrottledFunction", max_api_tps);
  }

  DummyOutcome ThrottledFunction() const
  {
    throttled_function_call_count_++;
    auto base_func = [this](int number) -> DummyOutcome {
      return this->BaseClient::ThrottledFunction(number);
    };
    return MakeCall<DummyOutcome, int, DummyClientErrors>(
      base_func, 0, __func__, DummyClientErrors::THROTTLING_ERROR, true);
  }

  mutable std::atomic<int> throttled_function_call_count_{0};
};

TEST(ThrottlingManagerTest, simpleThrottling)
{
  double max_tps = 100.0;
  int api_call_count = 100;
  ThrottledClient throttled_client(max_tps);
  for (int idx = 0; idx < api_call_count; idx++) {
    auto outcome = throttled_client.ThrottledFunction();
    ASSERT_TRUE(outcome.IsSuccess());
    std::this_thread::sleep_for(std::chrono::milliseconds(int(1000.0 / max_tps)));
  }
  /* Double the TPS without changing the limit. Every 2nd call should be throttled and not passed
   * through to the base client */
  max_tps *= 2;
  int succesful_outcomes = 0;
  for (int idx = 0; idx < api_call_count; idx++) {
    int call_count_before = throttled_client.BaseClient::throttled_function_call_count_;
    auto outcome = throttled_client.ThrottledFunction();
    if (outcome.IsSuccess()) {
      succesful_outcomes++;
    } else {
      ASSERT_EQ(DummyClientErrors::THROTTLING_ERROR, outcome.GetError().GetErrorType());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(int(1000.0 / max_tps)));
  }
  ASSERT_NEAR(succesful_outcomes, api_call_count / 2.0, 3);
}

/**
 * Thread routine. Calls the client's API for the duration specified, sleeping in between calls.
 * @see multiThreadedClientThrottling test case
 */
void MakeCalls(ThrottledClient * throttled_client, std::chrono::milliseconds duration,
               std::chrono::microseconds sleep_duration)
{
  auto now = std::chrono::steady_clock::now();
  auto prev_now = now - sleep_duration;
  auto end = std::chrono::steady_clock::now() + duration;
  while (now < end) {
    throttled_client->ThrottledFunction();
    /* Sleep for sleep_duration but compensate for some of the overhead by subtracting the delta
     * between now and prev_now. */
    if (now - prev_now > sleep_duration) {
      std::this_thread::sleep_for(sleep_duration - (now - prev_now - sleep_duration));
    } else {
      std::this_thread::sleep_for(sleep_duration);
    }
    prev_now = now;
    now = std::chrono::steady_clock::now();
  }
}

/**
 * Spawns multiple threads which call the throttled function & verifies that no more than max_tps
 * calls reached the downstream client.
 */
TEST(ThrottlingManagerTest, multiThreadedClientThrottling)
{
  const int milliseconds_to_run = 3500, sleep_duration_in_us = 150, max_tps = 125;
  int threads_to_spawn = std::max(2u, std::thread::hardware_concurrency());
  ThrottledClient throttled_client(max_tps);
  std::vector<std::thread> threads;
  threads.reserve(threads_to_spawn);
for (int tid = 0; tid < threads_to_spawn; tid++) {
    threads.emplace_back(MakeCalls, &throttled_client,
                                  std::chrono::milliseconds(milliseconds_to_run),
                                  std::chrono::microseconds(sleep_duration_in_us));
  }
  for (auto & t : threads) {
    t.join();
  }
  int expected_non_throttled_call_count = std::ceil(max_tps * milliseconds_to_run / static_cast<float>(1000));
  ASSERT_LE(throttled_client.BaseClient::throttled_function_call_count_,
            expected_non_throttled_call_count);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
