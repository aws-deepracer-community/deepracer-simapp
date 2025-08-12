import re
from typing import Optional, Dict, Any

from markov.boto.bedrock.handlers.base_handler import ModelHandler
from markov.boto.bedrock.handlers.mistral_handler import MistralHandler
from markov.boto.bedrock.handlers.claude_handler import ClaudeHandler
from markov.boto.bedrock.handlers.llama_handler import LlamaHandler
from markov.boto.bedrock.handlers.nova_handler import NovaHandler
from markov.boto.s3.constants import ActionSpaceTypes
from markov.log_handler.logger import Logger
import logging


class HandlerFactory:
    """Factory class for creating appropriate model handlers based on model ID"""
    
    @staticmethod
    def create_handler(model_id: str, 
                      action_space: Dict[str, Any], 
                      action_space_type: str,
                      region: Optional[str] = None,
                      system_prompt: Optional[str] = None,
                      max_context_messages: int = 0,
                      max_tokens: int = 1000) -> ModelHandler:
        """
        Create appropriate handler for the given model ID
        
        Args:
            model_id: Bedrock model ID or ARN
            action_space: Action space configuration
            action_space_type: Type of action space (continuous/discrete)
            region: AWS region
            system_prompt: Optional system prompt to use
            max_context_messages: Number of context messages to keep
            max_tokens: Maximum number of tokens to generate
            
        Returns:
            Appropriate model handler instance
        """
        logger = Logger(__name__, logging.INFO).get_logger()
        logger.info(f"Creating handler for model ID: {model_id}")
        
        model_id_lower = model_id.lower()
        
        # Check if model_id is an ARN
        if "arn:aws:bedrock" in model_id_lower:
            # Extract model name from ARN
            if "anthropic.claude" in model_id_lower or "claude" in model_id_lower:
                handler = ClaudeHandler(model_id, region)
            elif "mistral" in model_id_lower:
                handler = MistralHandler(model_id, region)
            elif "meta.llama" in model_id_lower or "llama" in model_id_lower:
                handler = LlamaHandler(model_id, region)
            elif "amazon.titan" in model_id_lower or "amazon.nova" in model_id_lower or "nova" in model_id_lower:
                handler = NovaHandler(model_id, region)
            else:
                logger.warning(f"Could not determine model type from ARN: {model_id}, defaulting to Claude")
                handler = ClaudeHandler(model_id, region)
        else:
            # Direct model ID - identify by prefix pattern
            if model_id_lower.startswith(("anthropic.", "claude")):
                handler = ClaudeHandler(model_id, region)
            elif model_id_lower.startswith(("mistral.")):
                handler = MistralHandler(model_id, region)
            elif model_id_lower.startswith(("meta.", "llama")):
                handler = LlamaHandler(model_id, region)
            elif model_id_lower.startswith(("amazon.titan", "amazon.nova", "titan", "nova")):
                handler = NovaHandler(model_id, region)
            else:
                # Default to Claude if unknown
                logger.warning(f"Unknown model type: {model_id}, defaulting to Claude")
                handler = ClaudeHandler(model_id, region)
        
        # Configure the handler
        if system_prompt:
            handler.set_system_prompt(system_prompt)
            
        handler.set_max_context_messages(max_context_messages)
        handler.set_action_space(action_space)
        handler.set_action_space_type(ActionSpaceTypes(action_space_type))
        handler.set_max_tokens(max_tokens)
        
        logger.info(f"Created {handler.__class__.__name__} for model {model_id}")
        return handler