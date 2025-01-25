/*
 * Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include <aws_common/fs_utils/wordexp_ros.h>
#include <wordexp.h>
#include <cstdlib>

int wordexp_ros(const char *words, wordexp_t *pwordexp, int flags) {
  bool should_unset_home = false;
  if (nullptr == getenv("HOME") && nullptr != getenv("ROS_HOME")) {
    // We don't want to change the behavior of anything else so we should unset HOME once we're done.
    should_unset_home = true;
    setenv("HOME", getenv("ROS_HOME"), true);
  }

  int wordexp_result = wordexp(words, pwordexp, flags);
  if (should_unset_home) {
    unsetenv("HOME");
  }
  return wordexp_result;
}
