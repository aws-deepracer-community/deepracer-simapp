#
# Copyright (c) 2017 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import pickle
from typing import Any, List, Dict

import tensorflow as tf
tf.compat.v1.disable_eager_execution()
tf.compat.v1.disable_resource_variables()
import numpy as np

from rl_coach.saver import Saver


class GlobalVariableSaver(Saver):
    def __init__(self, name):
        self._names = [name]
        # if graph is finalized, savers must have already already been added. This happens
        # in the case of a MonitoredSession
        self._variables = tf.compat.v1.trainable_variables()

        # target network is never saved or restored directly from checkpoint, so we are removing all its variables from the list
        # the target network would be synched back from the online network in graph_manager.improve(...), at the beginning of the run flow.
        self._variables = [v for v in self._variables if ('/target' not in v.name and name.split('/')[0]+'/'  in v.name)]

        # Handle case where no variables match the filter
        if not self._variables:
            # For testing purposes, include all trainable variables if none match the name filter
            all_variables = tf.compat.v1.trainable_variables()
            if all_variables:
                self._variables = all_variables
            else:
                # If still no variables, create an empty saver that does nothing
                self._saver = None
                self._variable_placeholders = []
                self._variable_update_ops = []
                return

        # Using a placeholder to update the variable during restore to avoid memory leak.
        # Ref: https://github.com/tensorflow/tensorflow/issues/4151
        self._variable_placeholders = []
        self._variable_update_ops = []
        if self._variables:
            for v in self._variables:
                variable_placeholder = tf.compat.v1.placeholder(v.dtype, shape=v.get_shape(), name=f"{v.name.split(':')[0]}_placeholder")
                self._variable_placeholders.append(variable_placeholder)
                self._variable_update_ops.append(v.assign(variable_placeholder))

        self._saver = tf.compat.v1.train.Saver(self._variables, max_to_keep=None)

    @property
    def path(self):
        """
        Relative path for save/load. If two checkpoint objects return the same path, they must be merge-able.
        """
        return ""  # use empty string for global file

    def save(self, sess: None, save_path: str) -> List[str]:
        """
        Save to save_path
        :param sess: active session
        :param save_path: full path to save checkpoint (typically directory plus checkpoint prefix plus self.path)
        :return: list of all saved paths
        """
        if self._saver is None:
            return []  # No variables to save
        save_path = self._saver.save(sess, save_path)
        return [save_path]

    def to_arrays(self, session: Any) -> Dict[str, np.ndarray]:
        """
        Save variables to numpy arrays
        :param session: active session
        :return: dictionary of variable name to numpy array
        """
        if not self._variables:
            return {}  # No variables to save
        
        return {
            k.name.split(":")[0]: v for k, v in zip(self._variables, session.run(self._variables))
        }

    def from_arrays(self, session: Any, tensors: Any):
        """
        Restore from arrays
        :param session: active session for session-based frameworks (e.g. TF)
        :param tensors: {name: array}
        """
        if not self._variables or not self._variable_placeholders:
            return  # No variables to restore
            
        # Handle both dict and items() formats
        if isinstance(tensors, dict):
            tensors = tensors.items()

        # Convert to dict and handle global/online network mapping
        variables = {k.replace("global/", "online/"): v for k, v in tensors}

        # Build assignment map: try exact match first, then shape-based matching
        placeholder_dict = {}
        unmatched_vars = []
        
        for ph, v in zip(self._variable_placeholders, self._variables):
            variable_name = v.name.split(":")[0]
            
            # Try exact name match first
            if variable_name in variables:
                placeholder_dict[ph] = variables[variable_name]
            else:
                unmatched_vars.append((ph, v))
        
        # For unmatched variables, try shape-based matching within same scope
        # This handles ROS1->ROS2 checkpoint compatibility where layer numbering differs
        if unmatched_vars:
            checkpoint_vars = {name: value for name, value in variables.items()}
            used_ckpt_keys = set()  # Track used checkpoint keys for 1-to-1 mapping
            
            for ph, model_var in unmatched_vars:
                model_name = model_var.name.split(":")[0]
                model_shape = tuple(model_var.get_shape().as_list())
                
                # Extract scope and variable type (e.g., "q2_head" and "kernel")
                model_parts = model_name.split('/')
                if len(model_parts) < 2:
                    continue
                    
                model_scope = '/'.join(model_parts[:-1])  # Everything except last part
                model_var_type = model_parts[-1]  # kernel, bias, etc.
                
                # Find checkpoint variable with matching scope, type, and shape
                for ckpt_name, ckpt_value in checkpoint_vars.items():
                    # Skip if already used (prevents double-assignment)
                    if ckpt_name in used_ckpt_keys:
                        continue
                    
                    ckpt_parts = ckpt_name.split('/')
                    if len(ckpt_parts) < 2:
                        continue
                    
                    ckpt_scope = '/'.join(ckpt_parts[:-1])
                    ckpt_var_type = ckpt_parts[-1]
                    ckpt_shape = tuple(ckpt_value.shape)
                    
                    # Match by: same parent scope, same variable type, same shape
                    # This handles dense/dense_1/dense_5 variations
                    if (self._scopes_match(model_scope, ckpt_scope) and 
                        model_var_type == ckpt_var_type and 
                        model_shape == ckpt_shape):
                        placeholder_dict[ph] = ckpt_value
                        used_ckpt_keys.add(ckpt_name)  # Mark as used
                        break
        
        if placeholder_dict:
            session.run(self._variable_update_ops, placeholder_dict)
    
    def _scopes_match(self, model_scope: str, ckpt_scope: str) -> bool:
        """
        Check if two variable scopes match, ignoring layer numbering differences.
        
        Examples:
          q2_head/dense_5 matches q2_head/dense_1 (same head, different numbering)
          q1_head/dense_2 matches q1_head/dense (same head, different numbering)
        """
        model_parts = model_scope.split('/')
        ckpt_parts = ckpt_scope.split('/')
        
        if len(model_parts) != len(ckpt_parts):
            return False
        
        for m_part, c_part in zip(model_parts, ckpt_parts):
            # Remove trailing numbers (e.g., dense_5 -> dense, dense_1 -> dense)
            m_base = m_part.rstrip('0123456789_')
            c_base = c_part.rstrip('0123456789_')
            
            if m_base != c_base:
                return False
        
        return True

    def to_string(self, session: Any) -> str:
        """
        Save to byte string
        :param session: active session
        :return: serialized byte string
        """
        return pickle.dumps(self.to_arrays(session), protocol=-1)

    def from_string(self, session: Any, string: str):
        """
        Restore from byte string
        :param session: active session
        :param string: byte string to restore from
        """
        self.from_arrays(session, pickle.loads(string))

    def _read_tensors(self, restore_path: str):
        """
        Load tensors from a checkpoint
        :param restore_path: full path to load checkpoint from.
        """
        # We don't use saver.restore() because checkpoint is loaded to online
        # network, but if the checkpoint is from the global network, a namespace
        # mismatch exists and variable name must be modified before loading.
        reader = tf.compat.v1.train.load_checkpoint(restore_path)
        for var_name, _ in reader.get_variable_to_shape_map().items():
            yield var_name, reader.get_tensor(var_name)

    def restore(self, sess: Any, restore_path: str):
        """
        Restore from restore_path
        :param sess: active session for session-based frameworks (e.g. TF)
        :param restore_path: full path to load checkpoint from.
        """
        if not self._variables:
            return  # No variables to restore
        self.from_arrays(sess, self._read_tensors(restore_path))

    def merge(self, other: "Saver"):
        """
        Merge other saver into this saver
        :param other: saver to be merged into self
        """
        assert isinstance(other, GlobalVariableSaver)
        self._names.extend(other._names)
        # There is nothing else to do because variables must already be part of
        # the global collection.
