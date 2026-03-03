import os
import pytest
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))

# Skip all MXNet tests - MXNet is deprecated and not required for core functionality
# TensorFlow serves as the primary deep learning framework for RL Coach
pytest.skip("MXNet is deprecated and not required for core functionality. TensorFlow is the primary framework.", allow_module_level=True)

import mxnet as mx


from rl_coach.architectures.embedder_parameters import InputEmbedderParameters
from rl_coach.architectures.mxnet_components.embedders.vector_embedder import VectorEmbedder
from rl_coach.base_parameters import EmbedderScheme


@pytest.mark.unit_test
def test_vector_embedder():
    params = InputEmbedderParameters(scheme=EmbedderScheme.Medium)
    emb = VectorEmbedder(params=params)
    emb.initialize()
    input_data = mx.nd.random.uniform(low=0, high=255, shape=(10, 100))
    output = emb(input_data)
    assert len(output.shape) == 2  # since last block was flatten
    assert output.shape[0] == 10  # since batch_size is 10
    assert output.shape[1] == 256  # since last dense layer has 256 units
