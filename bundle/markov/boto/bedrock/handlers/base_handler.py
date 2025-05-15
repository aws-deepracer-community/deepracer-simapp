import json
import logging
import os
from abc import ABC, abstractmethod
from typing import Dict, Any, Optional, List, Union

import boto3
from botocore.exceptions import ClientError
from markov.boto.bedrock.utils.json_extractor import extract_json_from_llm_response
from markov.boto.s3.constants import ActionSpaceTypes
from markov.log_handler.logger import Logger


class ModelHandler(ABC):
    """
    Base abstract class for all Bedrock model handlers.
    Provides common functionality and defines the interface that all handlers must implement.
    """

    def __init__(self, model_id: str, model_class: str, region: Optional[str] = None):
        """
        Initialize the model handler

        Args:
            model_id: The Bedrock model ID
            region: AWS region (optional - uses boto3 default otherwise)
        """
        self.model_id = model_id
        self.model_class = model_class
        self.region = region or os.environ.get(
            "APP_REGION", "us-east-1")

        # Set up logger
        self.logger = Logger(self.__class__.__name__, logging.INFO).get_logger()

        # Initialize token counters
        self.input_tokens = 0
        self.output_tokens = 0

        # Initialize Bedrock client
        self.client = boto3.client('bedrock-runtime', region_name=self.region)

        # Common properties that subclasses will use
        self.system_prompt = "You are an AI assistant."
        self.max_context_messages = 0
        self.conversation_context = []
        self.action_space = None
        self.action_space_type = None

    def set_system_prompt(self, prompt: Union[str, List[str]]) -> None:
        """Set the system prompt for the model"""
        if isinstance(prompt, list):
            self.system_prompt = "\n".join(prompt)
        else:
            self.system_prompt = prompt

    def set_max_context_messages(self, max_messages: int) -> None:
        """Set maximum number of conversation context messages to maintain"""
        self.max_context_messages = max_messages

    def set_action_space(self, action_space: Dict[str, Any]) -> None:
        """Set the action space for the model"""
        self.action_space = action_space

    def set_action_space_type(self, action_space_type: ActionSpaceTypes) -> None:
        """Set the action space type for the model"""
        self.action_space_type = action_space_type

    def clear_conversation(self) -> None:
        """Clear the conversation context"""
        self.conversation_context = []

    def invoke_model(self, request_body: Dict[str, Any]) -> Dict[str, Any]:
        """
        Invoke the Bedrock model with the given request body

        Args:
            request_body: The request body to send to Bedrock

        Returns:
            Dict containing the parsed response from Bedrock
        """
        try:
            # Convert request body to JSON
            request_json = json.dumps(request_body)

            # Log the request (truncated for brevity)
            self.logger.debug(
                f"Invoking {self.model_id} with request: {request_json[:200]}...")

            # Invoke the model
            response = self.client.invoke_model(
                modelId=self.model_id,
                contentType="application/json",
                accept="application/json",
                body=request_json
            )

            # Parse and return the response
            response_body = json.loads(response['body'].read())

            # Update token counts if the model supports it
            self.update_token_count(response_body)

            return response_body

        except Exception as e:
            self.logger.error(f"Error invoking {self.model_id}: {str(e)}")
            raise

    @abstractmethod
    def prepare_prompt(self, text_prompt: str, image_data: Optional[str] = None) -> Dict[str, Any]:
        """
        Prepare the prompt in the format required by the specific model

        Args:
            text_prompt: The text prompt to send to the model
            image_data: Optional base64-encoded image data

        Returns:
            Dict containing the formatted prompt for the model
        """
        pass

    @abstractmethod
    def extract_response_text(self, response_body: Dict[str, Any]) -> str:
        """
        Extract the text response from the model's response body

        Args:
            response_body: The parsed JSON response from the model

        Returns:
            The extracted text response
        """
        pass

    @abstractmethod
    def update_token_count(self, response_body: Dict[str, Any]) -> None:
        """
        Update token counts based on the model's response

        Args:
            response_body: The parsed JSON response
        """
        pass

    def extract_driving_action(self, response_text: str) -> Dict[str, Any]:
        """
        Extract the driving action from the model's text response. Override in subclasses.

        Args:
            response_text: The text response from the model

        Returns:
            Dict containing the driving action
        """
        return extract_json_from_llm_response(response_text, self.logger, self.model_class)

    def process(self, prompt: str, image_data: Optional[str] = None) -> Dict[str, Any]:
        """
        Process a prompt with image and return a driving action

        Args:
            prompt: The text prompt
            image_data: Base64-encoded image data

        Returns:
            Dict containing the driving action
        """
        # Prepare the prompt
        request_body = self.prepare_prompt(prompt, image_data)

        # Invoke the model
        response_body = self.invoke_model(request_body)

        # Extract the text response
        response_text = self.extract_response_text(response_body)

        # Extract and return the driving action
        return self.extract_driving_action(response_text)

    def get_token_usage(self) -> Dict[str, int]:
        """
        Get the current token usage statistics

        Returns:
            Dict with input and output token counts
        """
        return {
            "input_tokens": self.input_tokens,
            "output_tokens": self.output_tokens,
            "total_tokens": self.input_tokens + self.output_tokens
        }

    def reset_token_count(self) -> None:
        """Reset token counters to zero"""
        self.input_tokens = 0
        self.output_tokens = 0
