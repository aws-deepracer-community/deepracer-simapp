import random
import pickle

import pytest
import tensorflow as tf
tf.compat.v1.disable_eager_execution()
tf.compat.v1.disable_resource_variables()
import numpy as np

from rl_coach.architectures.tensorflow_components.savers import GlobalVariableSaver


def random_name():
    return "%032x" % random.randrange(16 ** 32)


@pytest.fixture
def name():
    return random_name()


@pytest.fixture
def variable(shape, name):
    tf.compat.v1.reset_default_graph()
    return tf.Variable(tf.zeros(shape), name=name)


@pytest.fixture
def shape():
    return (3, 5)


def assert_arrays_ones_shape(arrays, shape, name):
    assert list(arrays.keys()) == [name]
    assert len(arrays) == 1
    assert np.all(list(arrays[name][0]) == np.ones(shape))


@pytest.mark.unit_test
def test_global_variable_saver_to_arrays(variable, name, shape):
    with tf.compat.v1.Session() as session:
        session.run(tf.compat.v1.global_variables_initializer())
        session.run(variable.assign(tf.ones(shape)))

        saver = GlobalVariableSaver("name")
        arrays = saver.to_arrays(session)
        assert_arrays_ones_shape(arrays, shape, name)


@pytest.mark.unit_test
def test_global_variable_saver_from_arrays(variable, name, shape):
    with tf.compat.v1.Session() as session:
        session.run(tf.compat.v1.global_variables_initializer())

        saver = GlobalVariableSaver("name")
        saver.from_arrays(session, {name: np.ones(shape)})
        arrays = saver.to_arrays(session)
        assert_arrays_ones_shape(arrays, shape, name)


@pytest.mark.unit_test
def test_global_variable_saver_to_string(variable, name, shape):
    with tf.compat.v1.Session() as session:
        session.run(tf.compat.v1.global_variables_initializer())
        session.run(variable.assign(tf.ones(shape)))

        saver = GlobalVariableSaver("name")
        string = saver.to_string(session)
        arrays = pickle.loads(string)
        assert_arrays_ones_shape(arrays, shape, name)


@pytest.mark.unit_test
def test_global_variable_saver_from_string(variable, name, shape):
    with tf.compat.v1.Session() as session:
        session.run(tf.compat.v1.global_variables_initializer())

        saver = GlobalVariableSaver("name")
        saver.from_string(session, pickle.dumps({name: np.ones(shape)}, protocol=-1))
        arrays = saver.to_arrays(session)
        assert_arrays_ones_shape(arrays, shape, name)


# ROS1 to ROS2 Checkpoint Compatibility Tests
# Based on actual variable names observed in checkpoints:
# - ROS1 SAC: test9-ros1
# - ROS2 SAC: test09-ros2
# - ROS1 PPO: test001-ros1
# - ROS2 PPO: basic-test-ros2

@pytest.mark.unit_test
def test_scopes_match_real_sac_patterns():
    """Test actual SAC variable name patterns from real checkpoints"""
    tf.compat.v1.reset_default_graph()
    saver = GlobalVariableSaver("test")
    
    # Real pattern 1: q1_head/dense → q1_head/dense_2
    assert saver._scopes_match("q_values_head_0/q1_head/dense", 
                               "q_values_head_0/q1_head/dense_2")
    
    # Real pattern 2: q2_head/dense_1 → q2_head/dense_5
    assert saver._scopes_match("q_values_head_0/q2_head/dense_1", 
                               "q_values_head_0/q2_head/dense_5")
    
    # Verify full paths from actual checkpoints
    assert saver._scopes_match(
        "main_level/agent/q/online/network_0/q_values_head_0/q2_head/dense_1",
        "main_level/agent/q/online/network_0/q_values_head_0/q2_head/dense_5"
    )


@pytest.mark.unit_test
def test_scopes_match_preserves_head_numbers():
    """Test that q1_head and q2_head are preserved and don't cross-match"""
    tf.compat.v1.reset_default_graph()
    saver = GlobalVariableSaver("test")
    
    # q1_head should preserve the "1"
    assert saver._scopes_match("q1_head/dense", "q1_head/dense_2")
    
    # q2_head should preserve the "2"
    assert saver._scopes_match("q2_head/dense_1", "q2_head/dense_5")
    
    # q1_head and q2_head should NOT match each other
    assert not saver._scopes_match("q1_head/dense", "q2_head/dense")
    assert not saver._scopes_match("q1_head/dense_2", "q2_head/dense_5")


@pytest.mark.unit_test
def test_scopes_match_preserves_conv2d():
    """Test that Conv2d layers preserve the '2d' part"""
    tf.compat.v1.reset_default_graph()
    saver = GlobalVariableSaver("test")
    
    # Conv2d_0 appears in both ROS1 and ROS2 with same name
    assert saver._scopes_match("FRONT_FACING_CAMERA/Conv2d_0", 
                               "FRONT_FACING_CAMERA/Conv2d_0")
    
    # Conv2d_4 appears in both ROS1 and ROS2 with same name
    assert saver._scopes_match("FRONT_FACING_CAMERA/Conv2d_4", 
                               "FRONT_FACING_CAMERA/Conv2d_4")


@pytest.mark.unit_test
def test_from_arrays_exact_match_common_variables():
    """Test variables that have identical names in ROS1 and ROS2"""
    tf.compat.v1.reset_default_graph()
    
    # These variables are identical in both ROS1 and ROS2 checkpoints
    with tf.compat.v1.variable_scope("test"):
        conv = tf.compat.v1.get_variable('Conv2d_0/bias', shape=[32], dtype=tf.float32,
                                        initializer=tf.compat.v1.constant_initializer(0))
        dense = tf.compat.v1.get_variable('Dense_0/bias', shape=[256], dtype=tf.float32,
                                         initializer=tf.compat.v1.constant_initializer(0))
    
    saver = GlobalVariableSaver("test")
    
    # Checkpoint with exact matching names
    checkpoint = {
        'test/Conv2d_0/bias': np.ones(32, dtype=np.float32),
        'test/Dense_0/bias': np.ones(256, dtype=np.float32) * 2
    }
    
    with tf.compat.v1.Session() as sess:
        sess.run(tf.compat.v1.global_variables_initializer())
        saver.from_arrays(sess, checkpoint)
        
        # Verify exact matches work
        assert np.allclose(sess.run(conv), np.ones(32))
        assert np.allclose(sess.run(dense), np.ones(256) * 2)


@pytest.mark.unit_test
def test_from_arrays_real_sac_ros1_to_ros2():
    """Test the actual ROS1→ROS2 SAC checkpoint compatibility scenario
    
    This tests the 2 variables that differ between ROS1 and ROS2:
    1. q_values_head_0/q1_head/dense/bias → dense_2/bias
    2. q_values_head_0/q2_head/dense_1/kernel → dense_5/kernel
    """
    tf.compat.v1.reset_default_graph()
    
    # Create ROS2-style SAC network structure
    with tf.compat.v1.variable_scope("main_level/agent/q/online/network_0"):
        with tf.compat.v1.variable_scope("q_values_head_0"):
            with tf.compat.v1.variable_scope("q1_head"):
                # ROS2 has dense_2
                q1_dense_2 = tf.compat.v1.get_variable('dense_2/bias', shape=[256], 
                                                      dtype=tf.float32,
                                                      initializer=tf.compat.v1.constant_initializer(0))
            with tf.compat.v1.variable_scope("q2_head"):
                # ROS2 has dense_5
                q2_dense_5 = tf.compat.v1.get_variable('dense_5/kernel', shape=[256, 256], 
                                                      dtype=tf.float32,
                                                      initializer=tf.compat.v1.constant_initializer(0))
    
    saver = GlobalVariableSaver("main_level/agent/q/online/network_0")
    
    # ROS1 checkpoint data (dense and dense_1)
    ros1_checkpoint = {
        'main_level/agent/q/online/network_0/q_values_head_0/q1_head/dense/bias': 
            np.ones(256, dtype=np.float32) * 1.5,
        'main_level/agent/q/online/network_0/q_values_head_0/q2_head/dense_1/kernel': 
            np.ones((256, 256), dtype=np.float32) * 2.5
    }
    
    # Restore ROS1 checkpoint into ROS2 model
    with tf.compat.v1.Session() as sess:
        sess.run(tf.compat.v1.global_variables_initializer())
        saver.from_arrays(sess, ros1_checkpoint)
        
        # Verify shape-based matching worked
        q1_val = sess.run(q1_dense_2)
        q2_val = sess.run(q2_dense_5)
        
        assert np.allclose(q1_val, np.ones(256) * 1.5), \
            "ROS1 dense/bias should map to ROS2 dense_2/bias"
        assert np.allclose(q2_val, np.ones((256, 256)) * 2.5), \
            "ROS1 dense_1/kernel should map to ROS2 dense_5/kernel"
