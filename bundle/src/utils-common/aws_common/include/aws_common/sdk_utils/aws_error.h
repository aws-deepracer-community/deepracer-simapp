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

/**
 *  @file
 *  @brief Contains common Error handling functionality for AWS ROS libraries.
 *
 */

namespace Aws {
/**
 *  @enum Aws::AwsError
 *  @brief Defines error return codes for functions
 *  This enum defines standard error codes that will be returned by AWS ROS libraries.
 */
enum AwsError {
  /** Indicates that there is no error. */
  AWS_ERR_OK = 0,
  /** A generic error occured */
  AWS_ERR_FAILURE,
  /** An error related to memory usage/allocation */
  AWS_ERR_MEMORY,
  /** An error when a NULL value is supplied as a parameter when a non-NULL value is required */
  AWS_ERR_NULL_PARAM,
  /** An error indicating that a provided parameter value is invalid. */
  AWS_ERR_PARAM,
  /** An error indicating that a resource could not be found */
  AWS_ERR_NOT_FOUND,
  /** An error indicating that a timeout has occured */
  AWS_ERR_TIMEOUT,
  /** An error indicating that an event or action has already occured */
  AWS_ERR_ALREADY,
  /** An error indicating that a component is not yet initialized */
  AWS_ERR_NOT_INITIALIZED,
  /** An error indicating that there is not enough space or allocated resources to handle an
     action/event */
  AWS_ERR_NOT_ENOUGH_SPACE,
  /** An error indicating that a data structure is empty. */
  AWS_ERR_EMPTY,
  /** The feature is not supported. */
  AWS_ERR_NOT_SUPPORTED,
};
}  // namespace Aws
