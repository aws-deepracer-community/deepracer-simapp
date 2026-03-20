import json
import re
import logging
from typing import Dict, Any, Optional


def extract_json_from_llm_response(text: str, logger: Optional[logging.Logger] = None, 
                                  model_class: str = "Unknown") -> Dict[str, Any]:
    """
    Extract JSON from LLM text response, handling various formats

    Args:
        text: Response text from LLM
        logger: Optional logger for debug information
        model_class: Model class name for logging purposes

    Returns:
        Extracted JSON as dictionary, or empty dict if extraction fails
    """
    if logger is None:
        logger = logging.getLogger("JsonExtractor")
    
    # Try to parse the entire response as JSON first
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        pass

    # Try to find JSON blocks with regex
    patterns = [
        r'```json\s*(.*?)\s*```',  # Markdown JSON code blocks
        r'```\s*(.*?)\s*```',      # Any code block
        r'\{(?:[^{}]|(?:\{[^{}]*\}))*\}'  # Any JSON-like structure (less reliable)
    ]

    for pattern in patterns:
        matches = re.findall(pattern, text, re.DOTALL)
        if matches:
            for match in matches:
                try:
                    return json.loads(match)
                except json.JSONDecodeError:
                    continue

    # If we're still here, try to extract a JSON-like structure using a more lenient approach
    try:
        start_idx = text.find('{')
        end_idx = text.rfind('}')
        
        if start_idx != -1 and end_idx != -1 and end_idx > start_idx:
            json_str = text[start_idx:end_idx+1]
            return json.loads(json_str)
    except (json.JSONDecodeError, ValueError):
        pass

    # Log the failure and return empty dict
    if logger:
        logger.warning(f"Failed to extract JSON from {model_class} response: {text[:100]}...")
    
    return {}