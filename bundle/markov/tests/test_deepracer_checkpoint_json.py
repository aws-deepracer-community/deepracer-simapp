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

"""Tests for DeepracerCheckpointJson.get_deepracer_checkpoint, which picks the best or last checkpoint
for EVAL_CHECKPOINT and --pretrained_checkpoint."""

import json
import unittest.mock as mock

import pytest

from markov.boto.s3.files.checkpoint_files import deepracer_checkpoint_json
from markov.boto.s3.files.checkpoint_files.deepracer_checkpoint_json import DeepracerCheckpointJson

CHECKPOINTS = {
    "best_checkpoint": {"name": "12_Step-3456.ckpt", "avg_eval_metric": 100.0, "time_stamp": 1234.5},
    "last_checkpoint": {"name": "15_Step-4100.ckpt", "avg_eval_metric": 87.5, "time_stamp": 1300.2},
}


@pytest.fixture
def checkpoint_json(tmp_path):
    """A DeepracerCheckpointJson whose S3 download writes the given checkpoints file locally."""
    def make(content):
        s3_client = mock.MagicMock()
        s3_client.download_file.side_effect = \
            lambda bucket, s3_key, local_path: open(local_path, "w").write(json.dumps(content))
        with mock.patch.object(deepracer_checkpoint_json, "S3Client", return_value=s3_client):
            return DeepracerCheckpointJson(bucket="bucket", s3_prefix="prefix", local_dir=str(tmp_path / "agent"))
    return make


@pytest.mark.robomaker
@pytest.mark.sagemaker
@pytest.mark.parametrize("selection, expected", [
    ("best", "12_Step-3456.ckpt"),
    ("last", "15_Step-4100.ckpt"),
    ("LAST", "15_Step-4100.ckpt"),
    (None, "12_Step-3456.ckpt"),
    ("unknown", "12_Step-3456.ckpt"),
])
def test_get_deepracer_checkpoint_selects_best_unless_last(checkpoint_json, selection, expected):
    """Only "last" selects the last checkpoint; anything else keeps the best, the behavior before."""
    assert checkpoint_json(CHECKPOINTS).get_deepracer_checkpoint(selection) == expected


@pytest.mark.robomaker
@pytest.mark.sagemaker
def test_get_deepracer_checkpoint_returns_none_without_a_last_checkpoint(checkpoint_json):
    """A file with only a best checkpoint, as some imported models have, gives no last checkpoint."""
    only_best = {"best_checkpoint": CHECKPOINTS["best_checkpoint"]}
    assert checkpoint_json(only_best).get_deepracer_checkpoint("last") is None
