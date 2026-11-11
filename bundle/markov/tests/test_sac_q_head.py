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

"""Tests for markov/multi_agent_coach/architectures/tensorflow_components/heads/sac_q_head.py"""

import numpy as np
import pytest
import tensorflow as tf

from rl_coach.spaces import BoxActionSpace, SpacesDefinition, StateSpace, VectorObservationSpace

from markov.multi_agent_coach.agents.sac_agent import SoftActorCriticAgentParameters
from markov.multi_agent_coach.architectures.tensorflow_components.heads.sac_q_head import SACQHead

EMBEDDING_SIZE = 8


@pytest.fixture
def sac_q_head_graph():
    """Build the SAC Q head for a 2-dimensional continuous action space (steering, speed),
    the shape every DeepRacer SAC model uses.

    Returns:
        tuple: (graph, head, observation placeholder, policy action placeholder)
    """
    graph = tf.Graph()
    with graph.as_default():
        spaces = SpacesDefinition(
            state=StateSpace({"observation": VectorObservationSpace(EMBEDDING_SIZE)}),
            goal=None,
            action=BoxActionSpace(2, low=np.array([-30.0, 0.5]), high=np.array([30.0, 4.0])),
            reward=None)
        # The SAC agent passes the policy network's sampled action as P_action
        policy_action = tf.compat.v1.placeholder(tf.float32, [None, 2], name="policy_action")
        observation = tf.compat.v1.placeholder(tf.float32, [None, EMBEDDING_SIZE], name="embedding")
        head = SACQHead(agent_parameters=SoftActorCriticAgentParameters(), spaces=spaces,
                        network_name="q", P_action=policy_action)
        head(observation)
    return graph, head, observation, policy_action


@pytest.mark.sagemaker
def test_sac_q_head_builds_for_continuous_action_space(sac_q_head_graph):
    """The head builds with the action space shape as an array. With NumPy 2,
    int() of a 1-element array raised TypeError and every SAC training job failed at startup."""
    _, head, _, _ = sac_q_head_graph
    # action placeholder carries the actions plus one blend column for P_action
    assert head.actions_ph.shape.as_list() == [None, 3]
    assert head.actions.shape.as_list() == [None, 2]


@pytest.mark.sagemaker
def test_sac_q_head_blends_policy_action(sac_q_head_graph):
    """actions = actions_ph[:, :2] + actions_ph[:, 2:3] * P_action"""
    graph, head, observation, policy_action = sac_q_head_graph
    with tf.compat.v1.Session(graph=graph) as sess:
        sess.run(tf.compat.v1.global_variables_initializer())
        actions = sess.run(head.actions, feed_dict={
            head.actions_ph: np.array([[1.0, 2.0, 0.0], [1.0, 2.0, 1.0]]),
            policy_action: np.array([[10.0, 20.0], [10.0, 20.0]]),
            observation: np.zeros((2, EMBEDDING_SIZE))})
    np.testing.assert_allclose(actions, [[1.0, 2.0], [11.0, 22.0]])
