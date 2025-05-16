from typing import Dict, Any, Optional, List
import json
import os

from markov.boto.bedrock.handlers.base_handler import ModelHandler


class ClaudeHandler(ModelHandler):
    """Handler for Anthropic Claude models on AWS Bedrock"""

    def __init__(self, model_id: str, region: Optional[str] = None):
        """
        Initialize the Claude handler

        Args:
            model_id: The Claude model ID (e.g., "anthropic.claude-3-sonnet-20240229-v1:0")
            region: AWS region (optional)
        """
        super().__init__(model_id, "Claude", region)

        # Claude-specific settings
        self.anthropic_version = "bedrock-2023-05-31"

        # Override default system prompt
        self.system_prompt = "You are an AI driver assistant."

    def _create_user_message(self, prompt: str, image_data: Optional[str]) -> Dict[str, Any]:
        """
        Create a user message with optional image for Claude format

        Args:
            prompt: The text prompt
            image_data: Optional base64-encoded image

        Returns:
            Message in Claude format
        """
        user_content = [{
            "type": "text",
            "text": prompt
        }]

        if image_data:
            user_content.append({
                "type": "image",
                "source": {
                    "type": "base64",
                    "media_type": "image/jpeg",
                    "data": image_data
                }
            })

        return {
            "role": "user",
            "content": user_content
        }

    def prepare_prompt(self, text_prompt: str, image_data: Optional[str] = None) -> Dict[str, Any]:
        """
        Prepare a prompt for Claude in the format it expects

        Args:
            text_prompt: The text prompt to send to Claude
            image_data: Optional base64-encoded image data

        Returns:
            Dict containing the formatted prompt for Claude
        """
        messages = []

        # Add the action space and type
        if self.action_space is not None and self.action_space_type is not None:
            messages.append({
                "role": "user",
                "content": [{
                    "type": "text",
                    "text": json.dumps({
                        "action_space_type": self.action_space_type.value,
                        "action_space": self.action_space
                    })
                }]
            })

        # Add conversation context if available
        if self.conversation_context and self.max_context_messages > 0:
            # Add conversation context, limiting to max messages
            messages.extend(
                self.conversation_context[-self.max_context_messages:])

        # Create the user message with image
        user_message = self._create_user_message(text_prompt, image_data)
        messages.append(user_message)

        # Add the user message to conversation context
        if self.max_context_messages > 0:
            self.conversation_context.append(user_message)

        # Claude message format
        return {
            "anthropic_version": self.anthropic_version,
            "max_tokens": self.max_tokens,
            "temperature": 0.3,
            "top_p": 1.0,
            "top_k": 0,
            "system": self.system_prompt,
            "messages": messages
        }

    def extract_response_text(self, response_body: Dict[str, Any]) -> str:
        """
        Extract the text response from Claude's response body

        Args:
            response_body: The parsed JSON response from Claude

        Returns:
            The extracted text response
        """
        # Claude response format has content as a list of blocks
        content = response_body.get("content", [])

        # Extract text from all text blocks
        response_text = ""
        for block in content:
            if block.get("type") == "text":
                response_text += block.get("text", "")

        # Store conversation history if tracking context
        if self.max_context_messages > 0:
            # Add the assistant message to history
            assistant_message = {
                "role": "assistant",
                "content": [{
                    "type": "text",
                    "text": response_text
                }]
            }

            self.conversation_context.append(assistant_message)

            # Limit conversation context if needed
            # *2 for user/assistant pairs
            if len(self.conversation_context) > self.max_context_messages * 2:
                self.conversation_context = self.conversation_context[-self.max_context_messages*2:]

        return response_text

    def update_token_count(self, response_body: Dict[str, Any]) -> None:
        """
        Update token counts based on Claude's response

        Args:
            response_body: The parsed JSON response from Claude
        """
        usage = response_body.get("usage", {})
        self.input_tokens += usage.get("input_tokens", 0)
        self.output_tokens += usage.get("output_tokens", 0)
