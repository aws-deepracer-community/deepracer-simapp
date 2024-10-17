#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

SINGLE_RACER_INFO_JSON_SCHEMA = {
    "$schema": "http://json-schema.org/draft-04/schema#",
    "type": "object",
    "properties": {
        "racerAlias": {
            "type": "string"
        },
        "carConfig": {
            "type": "object",
            "properties": {
                "carColor": {
                    "type": "string"
                },
                "bodyShellType": {
                    "type": "string"
                }
            }
        },
        "inputModel": {
            "type": "object",
            "properties": {
                "s3BucketName": {
                    "type": "string"
                },
                "s3KeyPrefix": {
                    "type": "string"
                },
                "s3KmsKeyArn": {
                    "type": "string"
                }
            },
            "required": [
                "s3BucketName",
                "s3KeyPrefix"
            ]
        },
        "outputMetrics": {
            "type": "object",
            "properties": {
                "s3BucketName": {
                    "type": "string"
                },
                "s3KeyPrefix": {
                    "type": "string"
                },
                "s3KmsKeyArn": {
                    "type": "string"
                }
            },
            "required": [
                "s3BucketName",
                "s3KeyPrefix"
            ]
        },
        "outputStatus": {
            "type": "object",
            "properties": {
                "s3BucketName": {
                    "type": "string"
                },
                "s3KeyPrefix": {
                    "type": "string"
                },
                "s3KmsKeyArn": {
                    "type": "string"
                }
            },
            "required": [
                "s3BucketName",
                "s3KeyPrefix"
            ]
        },
        "outputSimTrace": {
            "type": "object",
            "properties": {
                "s3BucketName": {
                    "type": "string"
                },
                "s3KeyPrefix": {
                    "type": "string"
                },
                "s3KmsKeyArn": {
                    "type": "string"
                }
            },
            "required": [
                "s3BucketName",
                "s3KeyPrefix"
            ]
        },
        "outputMp4": {
            "type": "object",
            "properties": {
                "s3BucketName": {
                    "type": "string"
                },
                "s3KeyPrefix": {
                    "type": "string"
                },
                "s3KmsKeyArn": {
                    "type": "string"
                }
            },
            "required": [
                "s3BucketName",
                "s3KeyPrefix"
            ]
        }
    },
    "required": [
        "racerAlias",
        "inputModel",
        "outputMetrics",
        "outputStatus",
        "outputSimTrace",
        "outputMp4"
    ]
}

LIST_OF_RACERS_INFO_JSON_SCHEMA = {
    "$schema": "http://json-schema.org/draft-04/schema#",
    "type": "array",
    "minItems": 1,
    "maxItems": 2,
    "additionalProperties": False,
    "items": {
        "properties": {
            "racerAlias": {
                "type": "string"
            },
            "carConfig": {
                "type": "object",
                "properties": {
                    "carColor": {
                        "type": "string"
                    },
                    "bodyShellType": {
                        "type": "string"
                    }
                }
            },
            "inputModel": {
                "type": "object",
                "properties": {
                    "s3BucketName": {
                        "type": "string"
                    },
                    "s3KeyPrefix": {
                        "type": "string"
                    },
                    "s3KmsKeyArn": {
                        "type": "string"
                    }
                },
                "required": [
                    "s3BucketName",
                    "s3KeyPrefix"
                ]
            },
            "outputMetrics": {
                "type": "object",
                "properties": {
                    "s3BucketName": {
                        "type": "string"
                    },
                    "s3KeyPrefix": {
                        "type": "string"
                    },
                    "s3KmsKeyArn": {
                        "type": "string"
                    }
                },
                "required": [
                    "s3BucketName",
                    "s3KeyPrefix"
                ]
            },
            "outputStatus": {
                "type": "object",
                "properties": {
                    "s3BucketName": {
                        "type": "string"
                    },
                    "s3KeyPrefix": {
                        "type": "string"
                    },
                    "s3KmsKeyArn": {
                        "type": "string"
                    }
                },
                "required": [
                    "s3BucketName",
                    "s3KeyPrefix"
                ]
            },
            "outputSimTrace": {
                "type": "object",
                "properties": {
                    "s3BucketName": {
                        "type": "string"
                    },
                    "s3KeyPrefix": {
                        "type": "string"
                    },
                    "s3KmsKeyArn": {
                        "type": "string"
                    }
                },
                "required": [
                    "s3BucketName",
                    "s3KeyPrefix"
                ]
            },
            "outputMp4": {
                "type": "object",
                "properties": {
                    "s3BucketName": {
                        "type": "string"
                    },
                    "s3KeyPrefix": {
                        "type": "string"
                    },
                    "s3KmsKeyArn": {
                        "type": "string"
                    }
                },
                "required": [
                    "s3BucketName",
                    "s3KeyPrefix"
                ]
            }
        },
        "required": [
            "racerAlias",
            "inputModel",
            "outputMetrics",
            "outputStatus",
            "outputSimTrace",
            "outputMp4"
        ]
    },
}
