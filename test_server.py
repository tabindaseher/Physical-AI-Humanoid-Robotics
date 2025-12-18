import requests
import time
import subprocess
import sys
import os

def test_server():
    """
    Test script to start the server and check if it's running properly
    """
    # Start the server in a subprocess
    print("Starting the server...")
    server_process = subprocess.Popen([
        sys.executable, "-c", 
        "import uvicorn; from main import app; uvicorn.run(app, host='127.0.0.1', port=8000)"
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    # Wait a moment for the server to start
    time.sleep(5)
    
    try:
        # Test the health endpoint
        print("Testing health endpoint...")
        response = requests.get("http://localhost:8000/api/v1/health", timeout=10)
        print(f"Health check status: {response.status_code}")
        print(f"Response: {response.json()}")
        
        # Test the root endpoint
        print("\nTesting root endpoint...")
        try:
            response = requests.get("http://localhost:8000", timeout=10)
            print(f"Root endpoint status: {response.status_code}")
        except:
            print("Root endpoint not found (expected 404)")
        
    except requests.exceptions.ConnectionError:
        print("Could not connect to server. Is it running on port 8000?")
    except Exception as e:
        print(f"Error testing server: {e}")
    finally:
        # Kill the server process
        server_process.terminate()
        try:
            server_process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            server_process.kill()

if __name__ == "__main__":
    test_server()