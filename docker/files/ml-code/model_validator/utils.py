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

import subprocess

def build_validation_worker_cmd(s3_bucket, s3_prefix, aws_region):
    """
    Function used to build the validation worker python command.
    Arguments:
        s3_bucket (string): S3 bucket to fetch model from.
        s3_prefix (string): S3 prefix to fetch model from.
        aws_region (string): AWS region.
    Returns:
        (string) - Validation worker python command
    """
    return ['python', '/var/task/markov/validation_worker.py',
            '--s3_bucket', s3_bucket,
            '--s3_prefix', s3_prefix,
            '--aws_region', aws_region]


def run_cmd(cmd_args, change_working_directory="./", shell=False,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            env=None):
    """
    Function used to execute the shell commands

    Arguments:
        cmd_args (list): This is the list of commands to be appended in case
                         of shell=True, else executed as pipes.
        change_working_directory (string): This is to execute the command in different
                                           directory
        shell (bool): This is to say if the command to be executed as shell or not
        stdout (int): stdout stream target
        stderr (int): stderr stream target
        env (dict): environment variables
    Returns:
        (returncode, stdout, stderr) - tuples of returncode, stdout, and stderr
    """
    cmd = " ".join(map(str, cmd_args))
    process = subprocess.Popen(
        cmd if shell else cmd_args,
        cwd=change_working_directory,
        shell=shell,
        stdout=stdout,
        stderr=stderr,
        env=env
    )
    stdout, stderr = process.communicate()
    return process.returncode, stdout, stderr