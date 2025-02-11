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

''' This module contains all custom layers not included in rl_coach
'''
import tensorflow as tf

class Attention(object):
    '''Attention layer implementation. This layer needs to be placed after a 2D Convolutional,
       layer.
    '''
    def __init__(self, units: int):
        '''units - number of hidden units to use in the in the score wights '''
        self.units = units

    def __call__(self, input_layer, name: str = None, kernel_initializer=None,
                 activation=None, is_training=None):
        '''input_layer - Input layer to the attention layer, this should be a conv2D layer
           name - Base name for the dense layers
           kernel_initializer = Initializer for the weights
           activation - Activation function to use
           is_training - This is to adhere to rl_coach's expected function signature, it is
                         not used in this layer.
        '''
        score_weights = tf.compat.v1.layers.dense(input_layer, self.units, name="Score_{}".format(name),
                                        kernel_initializer=kernel_initializer, activation=None)
        score_activation = tf.nn.tanh(score_weights)
        attention_weights = tf.compat.v1.layers.dense(score_activation, 1, name="Attention_{}".format(name),
                                            kernel_initializer=kernel_initializer, activation=None)
        attention_weights_activation = tf.nn.softmax(attention_weights)
        context_vector = tf.multiply(attention_weights_activation, input_layer)
        context_conv = tf.reduce_sum(context_vector, axis=1, keepdims=True)
        return context_conv

    def __str__(self):
        '''Returns a string with the number of hidden units for the score weights'''
        return "Conv2dWithAttention (num outputs = {})".format(self.units)
