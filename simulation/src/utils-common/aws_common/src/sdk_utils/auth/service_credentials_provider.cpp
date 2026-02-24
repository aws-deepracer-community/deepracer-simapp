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

#include "aws_common/sdk_utils/auth/service_credentials_provider.h"

#include <aws/core/Aws.h>
#include <aws/core/utils/Array.h>
#include <aws/core/utils/DateTime.h>
#include <aws/core/utils/json/JsonSerializer.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <curl/curl.h>

#include "aws_common/sdk_utils/aws_error.h"
#include "aws_common/sdk_utils/parameter_reader.h"

/// Name of the credentials object in the json response from IoT
#define FIELD_CREDENTIALS "credentials"
/// Name of the field containing the expiration date in the json response from IoT
#define FIELD_EXPIRATION "expiration"
/// Name of the field containing the access key id in the json response from IoT
#define FIELD_ACCESS_KEY "accessKeyId"
/// Name of the field containing the secret key in the json response from IoT
#define FIELD_SECRET_KEY "secretAccessKey"
/// Name of the field containing the session token in the json response from IoT
#define FIELD_SESSION_TOKEN "sessionToken"

/// Header name to append to the request containing the thing name.
#define HEADER_THING_NAME "x-amzn-iot-thingname"

#ifndef MAX_IOT_CREDENTIAL_BYTES
/// The maximum size of a credentials response from the IoT service that will be tolerated by the
/// provider
#define MAX_IOT_CREDENTIAL_BYTES 65535
#endif

/// Logging tag used for all messages emitting from this module
static const char * AWS_LOG_TAG = "ServiceCredentialsProviderChain";

/// Go ahead and try to refresh credentials 30s before expiration
static const double EXPIRATION_GRACE_BUFFER = 30.0;


namespace Aws {
namespace Auth {

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// Helper functions
//

template <typename T>
static bool SetCurlOpt(CURL * curl, CURLoption opt, T lvalue);
static bool AppendHeader(struct curl_slist ** headers, const char * name, const char * value);
static bool IsIotConfigValid(const IotRoleConfig & config);
static bool ParseConfigVale(std::string & value, Aws::String & result);
static bool ParseConfigValue(std::string & value, int32_t & result);
template <typename T>
static bool GetConfigValue(std::map<std::string, std::string> & data, const char * name, T & result,
                           bool optional = false);

/**
 * \brief Helper to set a libcurl option or log an error if a problem occurred
 * @param curl Handle to the curl object to update
 * @param opt The curl option to set, see https://curl.haxx.se/libcurl/c/curl_easy_setopt.html for
 * more information.
 * @param lvalue the value of the option
 * @return True if the option was set successfully, otherwise false
 */
template <typename T>
static bool SetCurlOpt(CURL * curl, CURLoption opt, T lvalue)
{
  CURLcode res = curl_easy_setopt(curl, opt, lvalue);
  if (res != CURLE_OK) {
    AWS_LOG_ERROR(AWS_LOG_TAG, "Error setting curl option: %s", curl_easy_strerror(res));
    return false;
  }

  return true;
}

/**
 * \brief Appends a name/value pair to a list of curl headers
 * The libcurl API potentially returns new list pointer when a value is
 * appended but also returns NULL as an error. This function tries to append
 * the header value, and returns false if an error occurs and leaves the headers
 * pointer untouched.
 * @param headers Pointer to curl list containing the current headers
 * @param name The name of the header to append
 * @param value The value of the header to append
 * @return True if the header was appended and headers was updated, otherwise false
 */
static bool AppendHeader(struct curl_slist ** headers, const char * name, const char * value)
{
  Aws::StringStream stream;
  stream << name << ": " << value;

  struct curl_slist * next = curl_slist_append(*headers, stream.str().c_str());
  if (next == nullptr) {
    AWS_LOG_ERROR(AWS_LOG_TAG, "Error setting header[%s]: %s", name, value);
    return false;
  }

  *headers = next;
  return true;
}

/**
 * \brief Validates an instance of an IotRoleConfig struct
 * @param config The struct to validate
 * @return True if the struct is valid, meaning all the config needed to connect is there
 */
static bool IsIotConfigValid(const IotRoleConfig & config)
{
  return config.cafile.length() > 0 && config.certfile.length() > 0 &&
         config.keyfile.length() > 0 && config.host.length() > 0 && config.role.length() > 0 &&
         config.name.length() > 0 && config.connect_timeout_ms > 0 && config.total_timeout_ms > 0;
}

/**
 * \brief Helper to parse a configuration value into an Aws::String variable
 * @param value The configuration value to parse
 * @param result The variable to place the value into
 * @return True if the value was successfully parsed
 */
static bool ParseConfigValue(std::string & value, Aws::String & result)
{
  result = Aws::String(value.c_str(), value.size());
  return true;
}

/**
 * \brief Helper to parse a configuration value into an Aws::String variable
 * This function assumes that the value will be greater than zero, otherwise it
 * will assume an error (since it's used for timeouts)
 * @param value The configuration value to parse
 * @param result The variable to place the value into
 * @return True if the value was successfully parsed
 */
static bool ParseConfigValue(std::string & value, int32_t & result)
{
  int32_t tmp = Aws::Utils::StringUtils::ConvertToInt32(value.c_str());
  if (tmp > 0) {
    result = tmp;
    return true;
  }
  return false;
}

/**
 * \brief Helper to retrieve a specific config value from a map
 * Simple helper to try to retrieve data from a map and log a message if it
 * was not available. If no value is found, result is unchanged.
 * @param data The std::map to search for a value
 * @param name The name of the field to find
 * @param result Instance of Aws::String to store the result, if found
 * @return True if the value was successfully retrieved
 */
template <typename T>
static bool GetConfigValue(std::map<std::string, std::string> & data, const char * name, T & result,
                           bool optional)
{
  auto it = data.find(name);
  if (it != data.end()) {
    if (ParseConfigValue(it->second, result) && !optional) {
      return true;
    }
  }

  if (!optional) {
    AWS_LOG_DEBUG(AWS_LOG_TAG, "IoT provider: Missing %s configuration value", name);
  }
  return false;
}

/**
 * Retrieves all available auth parameters from the ParameterReaderInterface and
 * populates the ServiceAuthConfig struct with the data.
 * @param config The ServiceAuthConfig struct to store the config parameters
 * @param parameters The ParamReaderInterface to retrieve the param values
 * @return True if a valid configuration was loaded into the ServiceAuthConfig object
 * @see ServiceAuthConfig
 * @see ParameterReaderInterface
 */
bool GetServiceAuthConfig(ServiceAuthConfig & config,
                          const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameters)
{
  bool failed = false;
  Aws::String cafile, certfile, keyfile, host, role, name;
  int connect_timeout_ms = DEFAULT_AUTH_CONNECT_TIMEOUT_MS;
  int total_timeout_ms = DEFAULT_AUTH_TOTAL_TIMEOUT_MS;

  std::map<std::string, std::string> data;
  if (parameters->ReadParam(Aws::Client::ParameterPath("iot"), data) != AWS_ERR_OK) {
    failed = true;
  } else {
    if (!GetConfigValue(data, CFG_CAFILE, cafile)) { failed = true; }
    if (!GetConfigValue(data, CFG_CERTFILE, certfile)) { failed = true; }
    if (!GetConfigValue(data, CFG_KEYFILE, keyfile)) { failed = true; }
    if (!GetConfigValue(data, CFG_ENDPOINT, host)) { failed = true; }
    if (!GetConfigValue(data, CFG_ROLE, role)) { failed = true; }
    if (!GetConfigValue(data, CFG_THING_NAME, name)) { failed = true; }

    if (!GetConfigValue(data, CFG_CONNECT_TIMEOUT_MS, connect_timeout_ms, true)) {
      AWS_LOG_INFO(AWS_LOG_TAG, "Could not find config value %s, using default %d",
                   CFG_CONNECT_TIMEOUT_MS, DEFAULT_AUTH_CONNECT_TIMEOUT_MS);
    }

    if (!GetConfigValue(data, CFG_TOTAL_TIMEOUT_MS, total_timeout_ms, true)) {
      AWS_LOG_INFO(AWS_LOG_TAG, "Could not find config value %s, using default %d",
                   CFG_TOTAL_TIMEOUT_MS, DEFAULT_AUTH_TOTAL_TIMEOUT_MS);
    }
  }

  if (!failed) {
    config.iot.cafile = cafile;
    config.iot.certfile = certfile;
    config.iot.keyfile = keyfile;
    config.iot.host = host;
    config.iot.role = role;
    config.iot.name = name;
    config.iot.connect_timeout_ms = connect_timeout_ms;
    config.iot.total_timeout_ms = total_timeout_ms;

    AWS_LOG_INFO(AWS_LOG_TAG,
                 "IoT provider config: ca=%s,cert=%s,key=%s,ep=%s,role=%s,thing_name=%s,"
                 "connect_timeout=%d,total_timeout=%d",
                 config.iot.cafile.c_str(), config.iot.certfile.c_str(), config.iot.keyfile.c_str(),
                 config.iot.host.c_str(), config.iot.role.c_str(), config.iot.name.c_str(),
                 config.iot.connect_timeout_ms, config.iot.total_timeout_ms);
    return true;
  }

  AWS_LOG_INFO(AWS_LOG_TAG,
               "Missing or incomplete 'iot' parameters, skipping IoT credential provider");
  return false;
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// RequestContext Class
//

/**
 * \brief Encapsulates the response from a curl request
 * Curl uses a callback when performing a request for chunks of data.
 * This class serves as a the request data, using an Aws::StringStream to
 * accumulate data in memory. At any time, the GetValue() method can be
 * used to return a Json::JsonValue respresentation of the currently
 * buffered data.
 *
 * WARNING: This class will accumulate data, without
 */
class RequestContext
{
public:
  RequestContext() = default;
  ~RequestContext() = default;

  /**
   * \brief Curl callback for CURLOPT_WRITEFUNCTION
   * Recieves callbacks from the curl library any time data is received from a request.
   * See https://curl.haxx.se/libcurl/c/CURLOPT_WRITEFUNCTION.html for more info.
   * @param ptr Pointer to the data that was received
   * @param size Ignored as per documentation
   * @param nmemb Number of bytes read
   * @param userdata Context pointer, which happens to be an instance of RequestContext
   * @return Zero if an error occurs, otherwise nmemb
   */
  static size_t WriteData(char * ptr, size_t  /*size*/, size_t nmemb, void * userdata)
  {
    auto * ctx = static_cast<RequestContext *>(userdata);
    size_t current = ctx->stream_.tellp();

    // Returning less than nmemb to curl indicates an error
    if ((current + nmemb) > MAX_IOT_CREDENTIAL_BYTES) {
      AWS_LOG_ERROR(AWS_LOG_TAG,
                    "IoT response was too large, current:%d bytes, read:%d bytes, max:%d bytes",
                    current, nmemb, MAX_IOT_CREDENTIAL_BYTES);
      return 0;
    }

    ctx->stream_.write(ptr, nmemb);
    return nmemb;
  }

  /**
   * \brief Create a Json::JsonValue object from the current value of the buffer
   * @return An instance of Json::JsonValue containing the contents of the buffer
   */
  Aws::Utils::Json::JsonValue GetValue()
  {
    return Aws::Utils::Json::JsonValue(stream_.str()); 
  }

private:
  /// Current data that has be received so far
  Aws::StringStream stream_;
};

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// IotRoleCredentialsProvider Class
//

IotRoleCredentialsProvider::IotRoleCredentialsProvider(const IotRoleConfig & config)
  : cached_("", ""), config_(config), expiry_(0.0)
{
  if (config_.connect_timeout_ms <= 0) { config_.connect_timeout_ms = DEFAULT_AUTH_CONNECT_TIMEOUT_MS; }
  if (config_.total_timeout_ms <= 0) { config_.total_timeout_ms = DEFAULT_AUTH_TOTAL_TIMEOUT_MS; }
}

IotRoleCredentialsProvider::~IotRoleCredentialsProvider() = default;

AWSCredentials IotRoleCredentialsProvider::GetAWSCredentials()
{
  if (IsTimeExpired()) {
    AWS_LOG_DEBUG(AWS_LOG_TAG, "Timer has expired, refreshing AWS IoT Role credentials");
    Refresh();
  }

  return cached_;
}

/**
 * \brief Simple helper to check against the system clock if the epoch time has past
 * @return True if the expiration time has passed
 */
bool IotRoleCredentialsProvider::IsTimeExpired()
{
  // 30s grace buffer so requests have time to finish before expiry.
  return Aws::Utils::DateTime::Now().SecondsWithMSPrecision() > (expiry_.load() - EXPIRATION_GRACE_BUFFER);
}

void IotRoleCredentialsProvider::SetCredentials(AWSCredentials & creds_obj)
{
  cached_ = creds_obj;
}

/**
 * Validates a JsonValue response from an IoT credentials endpoint. Ensuring
 * that it contains a full set of credentials and logs any problems encountered.
 * @return True if the credentials are ok, otherwise false
 */
bool IotRoleCredentialsProvider::ValidateResponse(Aws::Utils::Json::JsonValue & value)
{
  if (!value.WasParseSuccessful()) {
    AWS_LOG_ERROR(AWS_LOG_TAG, "Unable to parse JSON response from AWS IoT credential provider");
    return false;
  }

  auto value_view = value.View();
  if (!value_view.ValueExists(FIELD_CREDENTIALS)) {
    AWS_LOG_ERROR(AWS_LOG_TAG, "Unable to find %s field in AWS IoT credential provider response",
                  FIELD_CREDENTIALS);
    return false;
  }

  auto creds = value_view.GetObject(FIELD_CREDENTIALS);

  if (!creds.IsObject()) {
    AWS_LOG_ERROR(AWS_LOG_TAG, "Expected object for %s in AWS IoT credential provider response",
                  FIELD_CREDENTIALS);
    return false;
  }

  if (!creds.ValueExists(FIELD_EXPIRATION)) {
    AWS_LOG_ERROR(AWS_LOG_TAG, "Unable to find %s field in AWS IoT credential provider response",
                  FIELD_EXPIRATION)
    return false;
  }

  if (!creds.ValueExists(FIELD_ACCESS_KEY)) {
    AWS_LOG_ERROR(AWS_LOG_TAG, "Unable to find %s field in AWS IoT credentials", FIELD_ACCESS_KEY);
    return false;
  }

  if (!creds.ValueExists(FIELD_SECRET_KEY)) {
    AWS_LOG_ERROR(AWS_LOG_TAG, "Unable to find %s in AWS IoT credentials", FIELD_SECRET_KEY);
    return false;
  }

  if (!creds.ValueExists(FIELD_SESSION_TOKEN)) {
    AWS_LOG_ERROR(AWS_LOG_TAG, "Unable to find %s in AWS IoT credentials", FIELD_SESSION_TOKEN);
    return false;
  }

  AWS_LOG_INFO(AWS_LOG_TAG, "Found valid credentials response from IoT");

  return true;
}

/**
 * Refreshes the cached credentials for the provider. The function uses
 * curl to make an HTTP request from the IoT credentials provider endpoint, using
 * client authentication via the cert/key pair. This function acquires the mutex
 * in order to make sure only a single request is outstanding at any given time.
 *
 * If the timer has not expired, this function will not update the cached creds.
 */
void IotRoleCredentialsProvider::Refresh()
{
  CURLcode res;
  CURL * curl = nullptr;
  struct curl_slist * headers = nullptr;

  AWS_LOG_INFO(AWS_LOG_TAG, "Retrieving IOT credentials!");

  std::lock_guard<std::mutex> lock(creds_mutex_);
  if (!IsTimeExpired()) { return; }

  // Make at most 1 tps for new creds, we also have a 30s grace buffer
  expiry_.store(Aws::Utils::DateTime::Now().SecondsWithMSPrecision() + EXPIRATION_GRACE_BUFFER + 1.0);

  curl = curl_easy_init();
  if (curl == nullptr) {
    AWS_LOG_ERROR(AWS_LOG_TAG, "Could not initialize curl!");
  } else {
    Aws::StringStream url_stream;
    url_stream << "https://" << config_.host << "/role-aliases/" << config_.role << "/credentials";

    // Setup SSL options
    if (!SetCurlOpt(curl, CURLOPT_SSL_VERIFYPEER, 1L)) { goto cleanup_curl; }
    if (!SetCurlOpt(curl, CURLOPT_SSL_VERIFYHOST, 2L)) { goto cleanup_curl; }
    if (!SetCurlOpt(curl, CURLOPT_HTTPGET, 1L)) { goto cleanup_curl; }
    if (!SetCurlOpt(curl, CURLOPT_HEADER, 0L)) { goto cleanup_curl; }
    if (!SetCurlOpt(curl, CURLOPT_CONNECTTIMEOUT_MS, config_.connect_timeout_ms)) { goto cleanup_curl; }
    if (!SetCurlOpt(curl, CURLOPT_TIMEOUT_MS, config_.total_timeout_ms)) { goto cleanup_curl; }

    if (!SetCurlOpt(curl, CURLOPT_URL, url_stream.str().c_str())) { goto cleanup_curl; }

    // Configure client auth
    if (!SetCurlOpt(curl, CURLOPT_CAINFO, config_.cafile.c_str())) { goto cleanup_curl; }

    if (!SetCurlOpt(curl, CURLOPT_SSLCERTTYPE, "PEM")) { goto cleanup_curl; }
    if (!SetCurlOpt(curl, CURLOPT_SSLCERT, config_.certfile.c_str())) { goto cleanup_curl; }

    if (!SetCurlOpt(curl, CURLOPT_SSLKEYTYPE, "PEM")) { goto cleanup_curl; }
    if (!SetCurlOpt(curl, CURLOPT_SSLKEY, config_.keyfile.c_str())) { goto cleanup_curl; }

    // Setup response handler
    RequestContext ctx;
    if (!SetCurlOpt(curl, CURLOPT_WRITEFUNCTION, &RequestContext::WriteData)) { goto cleanup_curl; }
    if (!SetCurlOpt(curl, CURLOPT_WRITEDATA, &ctx)) { goto cleanup_curl; }

    // Setup request headers
    if (!AppendHeader(&headers, "Accept", "application/json")) { goto cleanup_curl; }
    if (!AppendHeader(&headers, "Host", config_.host.c_str())) { goto cleanup_curl; }
    if (!AppendHeader(&headers, HEADER_THING_NAME, config_.name.c_str())) { goto cleanup_curl; }

    if (!SetCurlOpt(curl, CURLOPT_HTTPHEADER, headers)) { goto cleanup_curl; }

    // Make the request
    res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
      AWS_LOG_ERROR(AWS_LOG_TAG, "Error when curling endpoint: %s", curl_easy_strerror(res));
      goto cleanup_curl;
    }

    // Parse JSON response
    auto value = ctx.GetValue();
    if (!ValidateResponse(value)) { goto cleanup_curl; }

    auto creds_obj = value.View().GetObject(FIELD_CREDENTIALS);

    // Retrieve expiration date
    auto expires_str = creds_obj.GetString(FIELD_EXPIRATION);

    Aws::Utils::DateTime expiration(expires_str, Aws::Utils::DateFormat::ISO_8601);
    if (!expiration.WasParseSuccessful()) {
      AWS_LOG_ERROR(AWS_LOG_TAG, "Unable to parse expiration: %s", expires_str.c_str());
      goto cleanup_curl;
    }
    AWS_LOG_INFO(AWS_LOG_TAG, "Retrieved AWS creds from IoT, next expiration %s",
                 expires_str.c_str());

    cached_.SetAWSAccessKeyId(creds_obj.GetString(FIELD_ACCESS_KEY));
    cached_.SetAWSSecretKey(creds_obj.GetString(FIELD_SECRET_KEY));
    cached_.SetSessionToken(creds_obj.GetString(FIELD_SESSION_TOKEN));

    expiry_.store(expiration.SecondsWithMSPrecision());
  }

cleanup_curl:
  if (headers != nullptr) {
    curl_slist_free_all(headers);
    headers = nullptr;
  }

  if (curl != nullptr) {
    curl_easy_cleanup(curl);
    curl = nullptr;
  }
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// ServiceCredentialsProviderChain Class
//

ServiceCredentialsProviderChain::ServiceCredentialsProviderChain() = default;

ServiceCredentialsProviderChain::ServiceCredentialsProviderChain(const ServiceAuthConfig & config)
{
  if (IsIotConfigValid(config.iot)) {
    AWS_LOG_INFO(AWS_LOG_TAG, "Found valid IoT auth config, adding IotRoleCredentialsProvider");
    auto provider = Aws::MakeShared<IotRoleCredentialsProvider>(__func__, config.iot);
    AddProvider(provider);
  } else {
    AWS_LOG_INFO(AWS_LOG_TAG, "No valid IoT auth config, skipping IotRoleCredentialsProvider");
  }
}

} /* namespace Auth */
} /* namespace Aws */
