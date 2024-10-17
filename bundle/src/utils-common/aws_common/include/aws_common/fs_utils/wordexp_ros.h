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
#pragma once

#include <wordexp.h>

/**
 * @brief Provide wordexp-like behavior, with one difference: in the case that $HOME is not defined -
 *  use $ROS_HOME before falling back to looking up the PWD entry for the user.
 */
int wordexp_ros(const char *words, wordexp_t *pwordexp, int flags);
