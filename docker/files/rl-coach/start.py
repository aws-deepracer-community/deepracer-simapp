#!/usr/bin/env python
# coding: utf-8
from sagemaker.estimator import Estimator
import sagemaker
import boto3
import sys
import os
import json
import io

from time import gmtime

sys.path.append("common")


def str2bool(v):
    return v.lower() in ("yes", "true", "t", "1")


# S3 bucket
boto_session = boto3.session.Session(
    region_name=os.environ.get("AWS_REGION", "us-east-1")
)
endpoint_url = os.environ.get("S3_ENDPOINT_URL", None)

if endpoint_url == "":
    s3Client = boto_session.client("s3")
else:
    s3Client = boto_session.client("s3", endpoint_url=endpoint_url)
sage_session = sagemaker.local.LocalSession(
    boto_session=boto_session, s3_endpoint_url=endpoint_url
)
aws_region = sage_session.boto_region_name

s3_bucket = os.environ.get("MODEL_S3_BUCKET", None)
s3_prefix = os.environ.get("MODEL_S3_PREFIX", None)

pretrained = str2bool(os.environ.get("PRETRAINED", "False"))
s3_pretrained_bucket = os.environ.get("PRETRAINED_S3_BUCKET", "bucket")
s3_pretrained_prefix = os.environ.get("PRETRAINED_S3_PREFIX", "rl-deepracer-pretrained")

# SDK appends the job name and output folder
s3_output_path = "s3://{}/".format(s3_bucket)

# Hyperparameters
hyperparameter_file = os.environ.get(
    "HYPERPARAMETER_FILE_S3_KEY", "custom_files/hyperparameters.json"
)

# Model Metadata
modelmetadata_file = os.environ.get(
    "MODELMETADATA_FILE_S3_KEY", "custom_files/model_petadata.json"
)

# ### Define Variables
# create unique job name
job_name = s3_prefix

# Duration of job in seconds (5 hours)
job_duration_in_seconds = 24 * 60 * 60

print(
    "Model checkpoints and other metadata will be stored at: {}{}".format(
        s3_output_path, job_name
    )
)

s3_location = "s3://%s/%s" % (s3_bucket, s3_prefix)
print("Uploading to " + s3_location)

# We use the Estimator for training RL jobs.

sagemaker_image = os.environ.get("SAGEMAKER_IMAGE", None)
# 'local' for cpu, 'local_gpu' for nvidia gpu (and then you don't have to set default runtime to nvidia)
instance_type = "local_gpu" if (sagemaker_image.find("gpu") >= 0) else "local"
if sagemaker_image:
    print("Using image %s" % sagemaker_image)
else:
    print("ERROR: No defined image.")
    exit(1)

# Prepare hyperparameters
hyperparameters_core = {
    "s3_bucket": s3_bucket,
    "s3_prefix": s3_prefix,
    "aws_region": aws_region,
    "model_metadata_s3_key": "s3://{}/{}".format(s3_bucket, modelmetadata_file),
}

if pretrained:
    hyperparameters_core["pretrained_s3_bucket"] = "{}".format(s3_pretrained_bucket)
    hyperparameters_core["pretrained_s3_prefix"] = s3_pretrained_prefix
    hyperparameters_core["pretrained_checkpoint"] = os.environ.get(
        "PRETRAINED_CHECKPOINT", "best"
    )

# Downloading the hyperparameter file from our bucket.
hyperparameter_data = io.BytesIO()
s3Client.download_fileobj(s3_bucket, hyperparameter_file, hyperparameter_data)
hyperparameters_nn = json.loads(hyperparameter_data.getvalue().decode("utf-8"))
hyperparameters = {**hyperparameters_core, **hyperparameters_nn}
print(f"Configured following hyperparameters:\n{hyperparameters}")

# Define metrics
model_metrics = (
    [
        {"Name": "Entropy", "Regex": "Entropy=(.*?),"}
    ],
)

# Define environment variables
env_var = {}
env_var["RUN_ID"] = os.environ.get("RUN_ID","Unknown")

telegraf_host = os.environ.get("TELEGRAF_HOST", None)
if telegraf_host is not None:
    env_var["TELEGRAF_HOST"] = telegraf_host
    env_var["TELEGRAF_PORT"] = os.environ.get("TELEGRAF_PORT","8092")

max_memory_steps = os.environ.get("MAX_MEMORY_STEPS", "")
if max_memory_steps.isdigit():
    env_var["MAX_MEMORY_STEPS"] = max_memory_steps

if instance_type == "local_gpu":
    sagemaker_cuda_visible_devices = os.environ.get("CUDA_VISIBLE_DEVICES", "")
    if len(sagemaker_cuda_visible_devices) > 0:
        env_var["CUDA_VISIBLE_DEVICES"] = sagemaker_cuda_visible_devices


estimator = Estimator(
    sagemaker_session=sage_session,
    # bypass sagemaker SDK validation of the role
    role="aaa/",
    instance_type=instance_type,
    instance_count=1,
    output_path=s3_output_path,
    image_uri=sagemaker_image,
    hyperparameters=hyperparameters,
    max_run=job_duration_in_seconds,  # Maximum runtime in seconds
    metric_definitons=model_metrics,
    environment=env_var,
)

# Will block until job is stopped.
estimator.fit(job_name=job_name, wait=True)
