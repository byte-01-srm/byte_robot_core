#!/usr/bin/env python3
import requests
import sys

# The URL where your FastAPI server is running
API_URL = "http://127.0.0.1:8000/change-state"

def send_command(target_state: str, sender: str = "cli_tester"):
    """Sends a POST request to the quadruped API."""
    payload = {
        "new_state": target_state,
        "sender_id": sender
    }
    
    print(f"\n📡 Sending '{target_state}' command to robot...")
    
    try:
        # Send the POST request to the FastAPI server
        response = requests.post(API_URL, json=payload)
        
        # Check how the server responded
        if response.status_code == 200:
            print(f"✅ Success: {response.json()}")
        elif response.status_code == 429:
            print(f"⏳ Rejected: Robot is currently busy! ({response.json().get('detail')})")
        else:
            print(f"❌ Error {response.status_code}: {response.text}")
            
    except requests.exceptions.ConnectionError:
        print("🚨 Connection Error: Could not reach the API. Is Uvicorn running?")
    except requests.exceptions.Timeout:
        print("⏱️ Timeout: The server took too long to respond.")
    except Exception as e:
        print(f"⚠️ Unexpected Error: {e}")

def main():
    print("=" * 50)
    print("🤖 Quadruped API Client")
    print("Available states: sit, stand, walk, climb")
    print("Type 'quit' or 'exit' to close this client.")
    print("=" * 50)

    while True:
        try:
            # Prompt the user for a command
            cmd = input("\nEnter state > ").strip().lower()
            
            if cmd in ['quit', 'exit']:
                print("Closing client...")
                sys.exit(0)
            elif cmd == "":
                continue
                
            # Send the command
            send_command(cmd)
            
        except KeyboardInterrupt:
            print("\nClosing client...")
            sys.exit(0)

if __name__ == "__main__":
    main()