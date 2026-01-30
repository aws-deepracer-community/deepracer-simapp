# AWSSDKConfig.cmake
# This file is used to find the AWS SDK components

# Find the system-installed AWS SDK
find_package(AWSSDK REQUIRED COMPONENTS core logs kinesis)

# Set variables for downstream packages
set(AWSSDK_INCLUDE_DIRS ${AWSSDK_INCLUDE_DIRS})
set(AWSSDK_LIBRARIES ${AWSSDK_LIBRARIES})
