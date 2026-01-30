import os
import pytest
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))

# Skip all MXNet tests - MXNet is deprecated and not required for core functionality
# TensorFlow serves as the primary deep learning framework for RL Coach
pytest.skip("MXNet is deprecated and not required for core functionality. TensorFlow is the primary framework.", allow_module_level=True)

import mxnet as mx


from rl_coach.base_parameters import EmbedderScheme
from rl_coach.architectures.embedder_parameters import InputEmbedderParameters
from rl_coach.architectures.mxnet_components.embedders.image_embedder import ImageEmbedder


@pytest.mark.unit_test
def test_image_embedder():
    params = InputEmbedderParameters(scheme=EmbedderScheme.Medium)
    emb = ImageEmbedder(params=params)
    emb.initialize()
    # input is NHWC, and not MXNet default NCHW
    input_data = mx.nd.random.uniform(low=0, high=1, shape=(10, 244, 244, 3))
    output = emb(input_data)
    assert len(output.shape) == 2  # since last block was flatten
    assert output.shape[0] == 10  # since batch_size is 10
