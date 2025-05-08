import json
import numpy as np
import cv2
import base64
import io
import time
from typing import Dict, Any, List, Optional

from markov.agents.agent import Agent
from markov.boto.bedrock.handler_factory import HandlerFactory
from markov.log_handler.logger import Logger
from markov.agents.utils import RunPhaseSubject
from markov.constants import DEFAULT_COLOR
import logging

from PIL import Image

LOG = Logger(__name__, logging.INFO).get_logger()

class LLMAgent(Agent):
    """Agent that uses Amazon Bedrock LLM for decision making"""
    
    def __init__(self, sensor, ctrl, metrics, name="llm_agent", 
                 run_phase_subject=None):
        """Initialize LLM Agent
        
        Args:
            sensor: Sensor(s) providing observations to the agent
            ctrl: Controller for executing actions
            metrics: Metrics object for tracking agent performance
            name (str): Name of the agent
            run_phase_subject: Subject to notify about phase changes
        """
        super().__init__(sensor, ctrl, metrics)
        self.name = name
        self._run_phase_subject = run_phase_subject
        
        # Extract LLM configuration from model metadata
        model_metadata = self.ctrl.model_metadata
        
        # Initialize from model metadata
        self.model_id = model_metadata.model_id
        self.max_tokens = model_metadata.max_tokens
        self.system_prompt = model_metadata.system_prompt
        self.repeated_prompt = model_metadata.repeated_prompt
        self.context_window = model_metadata.context_window
        
        LOG.info(f"Initializing LLM Agent with model: {self.model_id}")
        
        # Get action space from model metadata
        self.action_space_info = self._get_action_space_info()
        
        # Create appropriate model handler
        system_prompt_text = "\n".join(self.system_prompt) if isinstance(self.system_prompt, list) else self.system_prompt
        self.handler = HandlerFactory.create_handler(
            model_id=self.model_id,
            action_space=self.action_space_info,
            action_space_type=self.ctrl.model_metadata.action_space_type,
            system_prompt=system_prompt_text,
            max_context_messages=self.context_window
        )
        
        LOG.info(f"LLM Agent initialized successfully with {self.handler.model_class} handler")
    
    def _get_action_space_info(self):
        """Get action space information from model metadata"""
        action_space = self.ctrl.model_metadata.action_space
        
        # For continuous action space
        if self.ctrl.model_metadata.action_space_type == "continuous":
            speed_range = action_space.get("speed", {})
            steering_range = action_space.get("steering_angle", {})
            
            return {
                "speed": {
                    "low": speed_range.get("low", 0.0),
                    "high": speed_range.get("high", 3.0)
                },
                "steering_angle": {
                    "low": steering_range.get("low", -30.0),
                    "high": steering_range.get("high", 30.0)
                }
            }
        # For discrete action space
        else:
            return {"actions": action_space}
    
    def _encode_image_to_base64(self, image_array):
        """Convert image array to base64 string"""
        # Convert to RGB if needed
        if len(image_array.shape) == 2:
            # Convert grayscale to RGB
            image_array = cv2.cvtColor(image_array, cv2.COLOR_GRAY2RGB)
        elif image_array.shape[2] == 4:
            # Convert RGBA to RGB
            image_array = cv2.cvtColor(image_array, cv2.COLOR_RGBA2RGB)
        
        # Convert to PIL Image
        pil_image = Image.fromarray(image_array)
        
        # Save to bytes
        buffered = io.BytesIO()
        pil_image.save(buffered, format="JPEG")
        
        # Encode to base64
        img_str = base64.b64encode(buffered.getvalue()).decode('utf-8')
        return img_str
    
    def _construct_prompt(self, observation):
        """Construct prompt for the LLM based on observation"""
        # Get image data from observation
        image = None
        for key, value in observation.items():
            if key in ['FRONT_FACING_CAMERA', 'CAMERA', 'observation', 'left_camera', 'stereo', 'front_facing_camera']:
                image = value
                break
        
        if image is None:
            LOG.error("No image found in observation")
            return "No image available. Please provide steering_angle and speed values."
        
        # Encode image to base64
        img_base64 = self._encode_image_to_base64(image)
        
        # Use the repeated prompt from model metadata or a default
        prompt_text = self.repeated_prompt or "Analyze the image and provide a driving command."
        
        # In the actual LLM agent, we don't need to construct a full prompt since the handler does that
        # We just need to provide the main instruction
        return prompt_text, img_base64
    
    def act(self, observation, reward, done, info):
        """Choose an action based on observation"""
        try:
            start_time = time.time()
            
            # Get prompt and image data
            prompt_text, image_data = self._construct_prompt(observation)
            
            # Process with model handler
            action_dict = self.handler.process(prompt_text, image_data)
            inference_time = time.time() - start_time
            
            LOG.info(f"LLM inference time: {inference_time:.2f}s")
            
            # Extract action values with proper keys
            steering_angle = float(action_dict.get("steering_angle", 0.0))
            speed = float(action_dict.get("speed", 0.0))
            
            # Ensure values are within bounds
            speed = max(self.action_space_info['speed']['low'], 
                       min(speed, self.action_space_info['speed']['high']))
                       
            steering_angle = max(self.action_space_info['steering_angle']['low'],
                                min(steering_angle, self.action_space_info['steering_angle']['high']))
            
            LOG.info(f"Driving action: speed={speed:.2f}, steering_angle={steering_angle:.2f}")
            
            if "reasoning" in action_dict:
                LOG.info(f"Reasoning: {action_dict['reasoning']}")
            
            # Convert to appropriate action format based on action space type
            if self.ctrl.model_metadata.action_space_type == "continuous":
                # For continuous action space, return [steering_angle, speed]
                action = [steering_angle, speed]
            else:
                # For discrete action space, find closest discrete action
                discrete_actions = self.action_space_info["actions"]
                best_action_idx = 0
                min_distance = float('inf')
                
                for idx, discrete_action in enumerate(discrete_actions):
                    steer_diff = abs(discrete_action["steering_angle"] - steering_angle)
                    speed_diff = abs(discrete_action["speed"] - speed)
                    distance = steer_diff + speed_diff  # Simple distance metric
                    
                    if distance < min_distance:
                        min_distance = distance
                        best_action_idx = idx
                
                action = best_action_idx
                
            return action
            
        except Exception as e:
            LOG.error(f"Error in LLM agent action selection: {e}")
            # Return safe default action
            if self.ctrl.model_metadata.action_space_type == "continuous":
                return [0.0, self.action_space_info['speed']['low']]
            else:
                return 0  # Default discrete action index
    
    def reset(self):
        """Reset the agent for a new episode"""
        LOG.info(f"Resetting LLM agent {self.name}")
        # Clear conversation history
        if hasattr(self, 'handler'):
            self.handler.clear_conversation()
        return True