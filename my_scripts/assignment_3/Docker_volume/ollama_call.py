import requests
import json

def ask_llm(query):
    """
    Send a query to Ollama's Gemma 3:4b model and get a response.
    
    Args:
        query (str): The problem or question to solve
        
    Returns:
        str: The model's response
    """
    url = "http://localhost:11434/api/generate"
    
    # Set up the payload with the model and the query
    payload = {
        "model": "gemma3:4b",
        "prompt": query,
        "stream": False
    }
    
    # Send the request to Ollama
    try:
        response = requests.post(url, json=payload)
        response.raise_for_status()  # Raise an exception for HTTP errors
        
        # Parse the JSON response
        result = response.json()
        return result.get("response", "No response received")
    
    except requests.exceptions.RequestException as e:
        return f"Error communicating with Ollama: {e}"

# Example usage
if __name__ == "__main__":
    problem = "You need to think and response. put you thinking in <think><\think> How many r in word strawberrrrry?"
    print(f"Query: {problem}")
    
    solution = ask_llm(problem)
    print(f"Solution:\n{solution}")