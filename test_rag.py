#!/usr/bin/env python3
"""
Test script to verify the RAG pipeline with the query "What is Physical AI?"
"""

import requests
import json
import uuid

def test_rag():
    # The API endpoint for the RAG service
    url = "http://localhost:8000/rag/book-wide"

    # Prepare the request payload
    payload = {
        "query": "What is Physical AI?",
        "session_id": str(uuid.uuid4()),  # Generate a unique session ID
        "metadata": {}
    }

    headers = {
        "Content-Type": "application/json"
    }

    print("Testing RAG with query: 'What is Physical AI?'")
    print(f"Sending request to: {url}")
    print(f"Payload: {json.dumps(payload, indent=2)}")

    try:
        response = requests.post(url, headers=headers, json=payload)
        print(f"Response status code: {response.status_code}")
        print(f"Response: {json.dumps(response.json(), indent=2)}")

        if response.status_code == 200:
            result = response.json()
            answer = result.get("answer", "No answer field in response")
            print(f"\nFinal answer to 'What is Physical AI?': {answer}")
        else:
            print(f"Error: {response.text}")

    except requests.exceptions.ConnectionError:
        print("Error: Could not connect to the RAG API server.")
        print("Make sure the FastAPI server is running on http://localhost:8000")
    except Exception as e:
        print(f"Error during test: {e}")

if __name__ == "__main__":
    test_rag()