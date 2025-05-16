from typing import Dict, Any, Optional, List
import json
import os

from markov.boto.bedrock.handlers.base_handler import ModelHandler


class NovaHandler(ModelHandler):
    """Handler for Amazon Nova models on AWS Bedrock"""

    def __init__(self, model_id: str, region: Optional[str] = None):
        """
        Initialize the Nova handler

        Args:
            model_id: The Nova model ID (e.g., "amazon.nova-pro-v1")
            region: AWS region (optional)
        """
        super().__init__(model_id, "Nova", region)

        # Nova-specific settings
        self.system_prompt = "You are an AI driver assistant."

    def _create_user_message(self, prompt: str, image_data: Optional[str]) -> Dict[str, Any]:
        """
        Create a user message with optional image for Nova format

        Args:
            prompt: The text prompt
            image_data: Optional base64-encoded image

        Returns:
            Message in Nova format
        """
        content = [{"text": prompt}]

        if image_data:
            content.append({
                "image": {
                    "format": "jpeg",
                    "source": {
                        "bytes": image_data
                    }
                }
            })

        return {
            "role": "user",
            "content": content
        }

    def prepare_prompt(self, text_prompt: str, image_data: Optional[str] = None) -> Dict[str, Any]:
        """
        Prepare a prompt for Nova in the format it expects

        Args:
            text_prompt: The text prompt to send to Nova
            image_data: Optional base64-encoded image data

        Returns:
            Dict containing the formatted prompt for Nova
        """
        # Initial system message with action space
        messages = [{
            "role": "user",
            "content": [
                {"text": self.system_prompt},
                {"text": json.dumps({
                    "action_space_type": self.action_space_type.value,
                    "action_space": self.action_space
                })}
            ]
        }]

        # Add conversation context if available
        if self.conversation_context and self.max_context_messages > 0:
            # Add conversation context, limiting to max messages
            messages.extend(
                self.conversation_context[-self.max_context_messages:])

        # Add the new user message
        user_message = self._create_user_message(text_prompt, image_data)
        messages.append(user_message)

        # Add the user message to conversation context
        if self.max_context_messages > 0:
            self.conversation_context.append(user_message)

        # Nova payload structure
        return {
            "inferenceConfig": {
                "max_new_tokens": self.max_tokens
            },
            "messages": messages
        }

    def extract_response_text(self, response_body: Dict[str, Any]) -> str:
        """
        Extract the text response from Nova's response body

        Args:
            response_body: The parsed JSON response from Nova

        Returns:
            The extracted text response
        """
        response_text = ""

        # Extract text from the response
        if (response_body.get("output") and
            response_body["output"].get("message") and
                response_body["output"]["message"].get("content")):
            response_text = response_body["output"]["message"]["content"][0].get(
                "text", "")
        else:
            self.logger.error(
                f"Unexpected Nova response structure: {json.dumps(response_body)[:200]}")
            raise ValueError("Unexpected Nova response structure")

        # Store conversation history if tracking context
        if self.max_context_messages > 0:
            # Create assistant message
            assistant_message = {
                "role": "assistant",
                "content": [{"text": response_text}]
            }

            # Add to conversation context
            self.conversation_context.append(assistant_message)

            # Limit conversation context if needed
            if len(self.conversation_context) > self.max_context_messages:
                self.conversation_context = self.conversation_context[-self.max_context_messages:]

        return response_text

    def update_token_count(self, response_body: Dict[str, Any]) -> None:
        """
        Update token counts based on Nova's response

        Args:
            response_body: The parsed JSON response from Nova
        """
        if response_body.get("usage"):
            usage = response_body["usage"]
            self.input_tokens += usage.get("inputTokens", 0)
            self.output_tokens += usage.get("outputTokens", 0)
        else:
            self.logger.debug(
                "Could not determine token usage from Nova response")


