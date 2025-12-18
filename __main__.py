# This file allows the backend directory to be run as a module
# When running: python -m backend
# This will execute the main application

import sys
import os
from pathlib import Path

# Add the parent directory to Python path to properly resolve backend imports
parent_dir = Path(__file__).parent.parent
if str(parent_dir) not in sys.path:
    sys.path.insert(0, str(parent_dir))

# Import and run the main application
from .main import app

def run():
    """Run the application using uvicorn programmatically"""
    try:
        import uvicorn
        uvicorn.run(app, host="0.0.0.0", port=8000, reload=True)
    except ImportError:
        print("Uvicorn not installed. Please install it with: pip install uvicorn")
        sys.exit(1)

if __name__ == "__main__":
    run()