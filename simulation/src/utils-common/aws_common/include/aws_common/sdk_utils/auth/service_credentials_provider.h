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

#include <aws/core/Aws.h>
#include <aws/core/auth/AWSAuthSigner.h>
#include <aws/core/auth/AWSCredentialsProvider.h>
#include <aws/core/auth/AWSCredentialsProviderChain.h>
#include <aws/core/utils/json/JsonSerializer.h>
#include <aws_common/sdk_utils/parameter_reader.h>

#include <mutex>

namespace Aws {
namespace Auth {

/// \brief Default number of milliseconds to wait before timing out when connecting to retrieve
/// credentials from IoT
static const long DEFAULT_AUTH_CONNECT_TIMEOUT_MS = 5000; // NOLINT(google-runtime-int)
/// \brief Default number of milliseconds to wait before timing out when retrieving credentials from
/// IoT
static const long DEFAULT_AUTH_TOTAL_TIMEOUT_MS = 10000;  // NOLINT(google-runtime-int)

static const char CFG_CAFILE[] = "cafile";
static const char CFG_CERTFILE[] = "certfile";
static const char CFG_KEYFILE[] = "keyfile";
static const char CFG_ENDPOINT[] = "endpoint";
static const char CFG_ROLE[] = "role";
static const char CFG_THING_NAME[] = "thing_name";
static const char CFG_CONNECT_TIMEOUT_MS[] = "connect_timeout_ms";
static const char CFG_TOTAL_TIMEOUT_MS[] = "total_timeout_ms";

/**
 * \brief Auth configuration needed to retrieve AWS credentials via the IoT service
 *
 * The AWS IoT service can be used to retrieve AWS service credentials. Please refer
 * to https://docs.aws.amazon.com/iot/latest/developerguide/authorizing-direct-aws.html
 * for more information.
 *
 * All configuration elements needed to retrieve credentials are contained in this
 * struct, which is used by the IotRoleCredentialsProvider to retrieve the actual
 * credentials.
 */
struct IotRoleConfig
{
  IotRoleConfig() = default;

  IotRoleConfig(const char * _cafile,
                const char * _certfile,
                const char * _keyfile,
                const char * _host,
                const char * _role,
                const char * _name,
                const int _connect_timeout_ms,
                const int _total_timeout_ms)
    : cafile(_cafile),
      certfile(_certfile),
      keyfile(_keyfile),
      host(_host),
      role(_role),
      name(_name),
      connect_timeout_ms(_connect_timeout_ms),
      total_timeout_ms(_total_timeout_ms) {}

  /// Path to the Root CA for the endpoint
  Aws::String cafile;
  /// Path to the certificate which identifies the device
  Aws::String certfile;
  /// Path to the related private key for the certificate
  Aws::String keyfile;
  /// Host name of the iot:CredentialProvider endpoint
  Aws::String host;
  /// Name of the AWS IoT Role Alias for the device
  Aws::String role;
  /// Thing name for the device
  Aws::String name;
  /// Number of ms to wait before timing out when connecting to the endpoint
  long connect_timeout_ms = 0;  // NOLINT(google-runtime-int)
  /// Total number of ms to wait for the entire connect/request/response transaction
  long total_timeout_ms = 0;    // NOLINT(google-runtime-int)
};

/**
 * \brief Auth configuration for ROS AWS service integration
 *
 * In order for devices to make calls to AWS services, credentials need to be
 * loaded that can sign requests. There are various methods to obtain credentials
 * and this struct contains all configuration needed, regardless of the method used.
 */
struct ServiceAuthConfig
{
  /// IoT-specific configuration
  IotRoleConfig iot;
};

/**
 * \brief Retrieves service authorization data from a ParameterReaderInterface and
 * populates the ServiceAuthConfig structure from the available parameters. If it was
 * able to load configuration data and successfully populate the structure, the function
 * will return true, otherwise false.
 * @return True if configuration was read and the struct was populated with information, otherwise
 * false
 */
bool GetServiceAuthConfig(ServiceAuthConfig & config,
                          const std::shared_ptr<Aws::Client::ParameterReaderInterface> & parameters);

/**
 * \brief AWSCredentialsProvider that obtains credentials using the AWS IoT Core service
 *
 * Implements the Aws::Auth::AWSCredentialsProvider interface to retrieve credentials
 * from the AWS IoT Core service. In order to be able to retrieve credentials, the
 * provider makes an HTTP request to the iot:CredentialProvider endpoint. Please refer
 * to https://docs.aws.amazon.com/iot/latest/developerguide/authorizing-direct-aws.html
 * for more information.
 */
class IotRoleCredentialsProvider : public Aws::Auth::AWSCredentialsProvider
{
public:
  /**
   * @param config Configuration for connecting to the AWS IoT endpoint
   */
  // NOLINTNEXTLINE(google-explicit-constructor, hicpp-explicit-conversions)
  IotRoleCredentialsProvider(const IotRoleConfig & config);
  IotRoleCredentialsProvider(const IotRoleCredentialsProvider & other) = delete;
  IotRoleCredentialsProvider & operator=(const IotRoleCredentialsProvider & other) = delete;

  ~IotRoleCredentialsProvider() override;

  AWSCredentials GetAWSCredentials() override;

protected:
  // Visible for testing

  /// \brief Refreshes the cached AWS credentials
  void Refresh();
  /// \brief Sets the cached credentials
  void SetCredentials(AWSCredentials & creds_obj);
  /// \brief Validates the json response from the AWS IoT service
  bool ValidateResponse(Aws::Utils::Json::JsonValue & value);
  /// \brief Returns true if the credentials have expired
  bool IsTimeExpired();
  /// Current cached credentials
  Aws::Auth::AWSCredentials cached_;

private:
  /// Configuration for connecting to IoT
  IotRoleConfig config_;
  /// Mutex to ensure only a single request is outstanding at any given time
  std::mutex creds_mutex_;
  /// Future epoch when the cached credentials will expire
  std::atomic<double> expiry_;
};

/**
 * \brief Credentials provider chain for ROS AWS service integrations
 *
 * The ServiceCredentialsProviderChain class extends the DefaultAWSCredentialsProviderChain,
 * which has a list of methods to retrieve credentials using various methods. The
 * class adds the IotRoleCredentialsProvider to the end of the chain if there is a
 * valid configuration for retrieving credentials from AWS IoT.
 */
class ServiceCredentialsProviderChain : public DefaultAWSCredentialsProviderChain
{
public:
  ServiceCredentialsProviderChain();
  /**
   * @param config Configuration for available credential providers
   */
  // NOLINTNEXTLINE(google-explicit-constructor, hicpp-explicit-conversions)
  ServiceCredentialsProviderChain(const ServiceAuthConfig & config);
  ~ServiceCredentialsProviderChain() override = default;
};

} /* namespace Auth */
} /* namespace Aws */
