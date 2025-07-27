import boto3
import json
import logging
from botocore.config import Config
from markov.log_handler.logger import Logger

LOG = Logger(__name__, logging.INFO).get_logger()

class BedrockClient:
    """Client for interacting with AWS Bedrock models"""
    
    def __init__(self, model_id, region_name="us-east-1", max_tokens=1000, 
                 temperature=0.7, top_p=0.9):
        """Initialize the Bedrock client
        
        Args:
            model_id (str): ARN or ID of Bedrock model
            region_name (str): AWS region name
            max_tokens (int): Maximum tokens to generate
            temperature (float): Temperature for sampling
            top_p (float): Top-p sampling parameter
        """
        self.model_id = model_id
        self.max_tokens = max_tokens
        self.temperature = temperature
        self.top_p = top_p
        
        # Configure retry behavior
        boto_config = Config(
            retries=dict(
                max_attempts=3
            )
        )
        
        # Initialize Bedrock client
        try:
            self.bedrock_client = boto3.client(
                service_name="bedrock-runtime",
                region_name=region_name,
                config=boto_config
            )
            LOG.info(f"Initialized Bedrock client for model: {model_id}")
        except Exception as e:
            LOG.error(f"Failed to initialize Bedrock client: {e}")
            raise
        
        # Determine model provider from model_id
        self.model_provider = self._get_model_provider()
        LOG.info(f"Detected model provider: {self.model_provider}")
    
    def _get_model_provider(self):
        """Determine the model provider from the model ID"""
        model_id_lower = self.model_id.lower()
        
        if "anthropic" in model_id_lower or "claude" in model_id_lower:
            return "anthropic"
        elif "mistral" in model_id_lower:
            return "mistral"
        elif "llama" in model_id_lower:
            return "meta"
        elif "amazon.titan" in model_id_lower:
            return "amazon"
        else:
            return "unknown"
    
    def invoke_model(self, prompt, system_prompt=None):
        """Invoke the Bedrock model with the given prompt
        
        Args:
            prompt (str): The prompt to send to the model
            system_prompt (str, optional): System prompt for models that support it
            
        Returns:
            str: The model's response text
        """
        try:
            # Format request body based on model provider
            if self.model_provider == "anthropic":
                body = json.dumps({
                    "anthropic_version": "bedrock-2023-05-31",
                    "max_tokens": self.max_tokens,
                    "temperature": self.temperature,
                    "system": system_prompt if system_prompt else "",
                    "messages": [
                        {"role": "user", "content": prompt}
                    ]
                })
            elif self.model_provider == "mistral":
                body = json.dumps({
                    "prompt": prompt,
                    "max_tokens": self.max_tokens,
                    "temperature": self.temperature,
                    "top_p": self.top_p
                })
            elif self.model_provider == "meta":
                body = json.dumps({
                    "prompt": prompt,
                    "max_gen_len": self.max_tokens,
                    "temperature": self.temperature,
                    "top_p": self.top_p
                })
            else:
                # Default format for other models
                body = json.dumps({
                    "inputText": prompt,
                    "textGenerationConfig": {
                        "maxTokenCount": self.max_tokens,
                        "temperature": self.temperature,
                        "topP": self.top_p
                    }
                })
            
            # Call the model
            response = self.bedrock_client.invoke_model(
                modelId=self.model_id,
                body=body
            )
            
            # Parse response based on model provider
            response_body = json.loads(response.get("body").read())
            
            if self.model_provider == "anthropic":
                return response_body.get("content", [{}])[0].get("text", "")
            elif self.model_provider == "mistral":
                return response_body.get("outputs", [{}])[0].get("text", "")
            elif self.model_provider == "meta":
                return response_body.get("generation", "")
            else:
                return response_body.get("results", [{}])[0].get("outputText", "")
                
        except Exception as e:
            LOG.error(f"Error invoking Bedrock model: {e}")
            return ""