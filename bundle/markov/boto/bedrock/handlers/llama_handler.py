from typing import Dict, Any, Optional, List
import json
import os

from markov.boto.bedrock.handlers.base_handler import ModelHandler


class LlamaHandler(ModelHandler):
    """Handler for Meta's Llama models on AWS Bedrock"""

    def __init__(self, model_id: str, region: Optional[str] = None):
        """
        Initialize the Llama handler

        Args:
            model_id: The Llama model ID (e.g., "meta.llama3-70b-instruct-v1:0")
            region: AWS region (optional)
        """
        super().__init__(model_id, "Llama", region)

        # Llama-specific settings
        self.max_tokens = int(os.environ.get("MAX_TOKENS", "1024"))
        self.temperature = 0.0  # Deterministic for DeepRacer

        # Override default system prompt
        self.system_prompt = "You are an AI driver assistant."

    def _create_user_message(self, prompt: str, image_data: Optional[str]) -> Dict[str, Any]:
        """
        Create a user message object for Llama format

        Args:
            prompt: The text prompt
            image_data: Optional base64-encoded image

        Returns:
            Message in Llama format
        """
        # For Llama we don't need a complex message object since we format everything
        # into a single prompt string in prepare_prompt
        return {
            "role": "user",
            "content": prompt,
            "image": image_data if image_data else None
        }

    def prepare_prompt(self, text_prompt: str, image_data: Optional[str] = None) -> Dict[str, Any]:
        """
        Prepare a prompt for Llama in the format it expects

        Args:
            text_prompt: The text prompt to send to Llama
            image_data: Optional base64-encoded image data

        Returns:
            Dict containing the formatted prompt for Llama
        """
        # Llama uses a different formatting style for system prompts and conversation
        full_prompt = f"<|system|>\n{self.system_prompt}\n"

        # Add action space information if available
        if self.action_space is not None and self.action_space_type is not None:
            full_prompt += f"\nAction space type: {self.action_space_type}\n"
            full_prompt += f"Action space: {json.dumps(self.action_space)}\n"

        full_prompt += "</s>\n"

        # Add conversation context if available
        if self.conversation_context and self.max_context_messages > 0:
            # Format the conversation history in Llama's expected format
            # *2 for user/assistant pairs
            history = self.conversation_context[-self.max_context_messages*2:]
            for entry in history:
                role = entry.get('role', '')
                content = entry.get('content', '')

                if role == 'user':
                    full_prompt += f"<|user|>\n{content}\n</s>\n"
                elif role == 'assistant':
                    full_prompt += f"<|assistant|>\n{content}\n</s>\n"

        # Add the current prompt
        full_prompt += f"<|user|>\n{text_prompt}\n</s>\n"
        full_prompt += "<|assistant|>\n"

        # Llama request format
        request_body = {
            "prompt": full_prompt,
            "max_gen_len": self.max_tokens,
            "temperature": self.temperature,
        }

        # Add image if provided
        if image_data:
            request_body["image_data"] = [image_data]

        # Store the user message in conversation context
        if self.max_context_messages > 0:
            user_message = self._create_user_message(text_prompt, image_data)
            self.conversation_context.append(user_message)

        return request_body

    def extract_response_text(self, response_body: Dict[str, Any]) -> str:
        """
        Extract the text response from Llama's response body

        Args:
            response_body: The parsed JSON response from Llama

        Returns:
            The extracted text response
        """
        # Llama typically returns a generation field
        response_text = response_body.get("generation", "")

        # Store conversation history if tracking context
        if self.max_context_messages > 0:
            # Add the assistant message to history
            assistant_message = {
                "role": "assistant",
                "content": response_text
            }

            self.conversation_context.append(assistant_message)

            # Limit conversation context if needed
            # *2 for user/assistant pairs
            if len(self.conversation_context) > self.max_context_messages * 2:
                self.conversation_context = self.conversation_context[-self.max_context_messages*2:]

        return response_text

    def update_token_count(self, response_body: Dict[str, Any]) -> None:
        """
        Update token counts based on Llama's response

        Args:
            response_body: The parsed JSON response from Llama
        """
        # Llama may have a different format for token usage
        usage = response_body.get("usage", {})

        # Various formats seen in different Llama versions
        self.input_tokens += usage.get("input_tokens",
                                       0) or usage.get("prompt_tokens", 0)
        self.output_tokens += usage.get("output_tokens",
                                        0) or usage.get("generated_tokens", 0)
