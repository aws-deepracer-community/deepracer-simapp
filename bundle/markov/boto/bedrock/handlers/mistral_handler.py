from typing import Dict, Any, Optional, List
import json
import os

from markov.boto.bedrock.handlers.base_handler import ModelHandler


class MistralHandler(ModelHandler):
    """Handler for Mistral AI models on AWS Bedrock"""

    def __init__(self, model_id: str, region: Optional[str] = None):
        """
        Initialize the Mistral handler

        Args:
            model_id: The Mistral model ID (e.g., "mistral.mistral-large-2402-vision-v1:0")
            region: AWS region (optional)
        """
        super().__init__(model_id, "Mistral", region)

        # Mistral-specific settings
        self.temperature = 0.0  # Deterministic for DeepRacer

        # Override default system prompt
        self.system_prompt = "You are an AI driver assistant."

    def _create_user_message(self, prompt: str, image_data: Optional[str]) -> Dict[str, Any]:
        """
        Create a user message with optional image for Mistral format

        Args:
            prompt: The text prompt
            image_data: Optional base64-encoded image

        Returns:
            Message in Mistral format
        """
        content = [{"type": "text", "text": prompt}]

        if image_data:
            content.append({
                "type": "image_url",
                "image_url": {
                    "url": f"data:image/jpeg;base64,{image_data}"
                }
            })

        return {
            "role": "user",
            "content": content
        }

    def prepare_prompt(self, text_prompt: str, image_data: Optional[str] = None) -> Dict[str, Any]:
        """
        Prepare a prompt for Mistral in the format it expects

        Args:
            text_prompt: The text prompt to send to Mistral
            image_data: Optional base64-encoded image data

        Returns:
            Dict containing the formatted prompt for Mistral
        """

        # Create system message
        system_message = {
            "role": "system",
            "content": self.system_prompt
        }

        # Create action space message
        action_space_message = {
            "role": "user",
            "content": [
                {
                    "type": "text",
                    "text": json.dumps({
                        "action_space_type": self.action_space_type.value,
                        "action_space": self.action_space
                    })
                }
            ]
        }

        # Build the messages array
        messages = [system_message, action_space_message]

        # Add conversation context if available
        if self.conversation_context and self.max_context_messages > 0:
            # Add conversation context, limiting to max messages
            messages.extend(
                self.conversation_context[-self.max_context_messages:])

        # Add the new user message
        user_message = self._create_user_message(text_prompt, image_data)
        messages.append(user_message)

        # Add the user message to conversation context if tracking
        if self.max_context_messages > 0:
            self.conversation_context.append(user_message)

        # Mistral payload structure
        return {
            "messages": messages,
            "max_tokens": self.max_tokens
        }

    def extract_response_text(self, response_body: Dict[str, Any]) -> str:
        """
        Extract the text response from Mistral's response body

        Args:
            response_body: The parsed JSON response from Mistral

        Returns:
            The extracted text response
        """
        response_text = ""

        # Extract text from choices format
        if response_body.get("choices") and len(response_body["choices"]) > 0:
            if response_body["choices"][0].get("message") and response_body["choices"][0]["message"].get("content"):
                response_text = response_body["choices"][0]["message"]["content"]
        else:
            # Fallback to other possible response formats
            self.logger.debug("Using fallback response extraction for Mistral")
            outputs = response_body.get("outputs", [])
            response_text = outputs[0] if outputs else response_body.get(
                "output", "")

        # Store conversation history if tracking context
        if self.max_context_messages > 0:
            # Add the assistant message to history
            self.conversation_context.append({
                "role": "assistant",
                "content": response_text
            })

            # Limit conversation context if needed
            if len(self.conversation_context) > self.max_context_messages:
                self.conversation_context = self.conversation_context[-self.max_context_messages:]

        return response_text

    def update_token_count(self, response_body: Dict[str, Any]) -> None:
        """
        Update token counts based on Mistral's response

        Args:
            response_body: The parsed JSON response from Mistral
        """
        usage = response_body.get("usage", {})

        # Handle Mistral API format
        if usage.get("prompt_tokens") is not None:
            self.input_tokens += usage.get("prompt_tokens", 0)
            self.output_tokens += usage.get("completion_tokens",
                                            0) or usage.get("generation_tokens", 0)
        # Handle case with only total_tokens
        elif usage.get("total_tokens") is not None:
            total_tokens = usage.get("total_tokens", 0)
            # Default to assuming 2/3 are prompt tokens if we don't know better
            self.input_tokens += int(total_tokens * 0.67)
            self.output_tokens += total_tokens - int(total_tokens * 0.67)
        else:
            self.logger.debug(
                "Could not determine token usage from Mistral response")

