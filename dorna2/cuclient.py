import requests

class cuda_client:
    def __init__(self, server_url = None):
        self.server_url = server_url
        self.session_id = None

    def connect(self, server_url = None):
        """Start a new session."""
        if server_url:
            self.server_url = server_url
        response = requests.post(f"{self.server_url}/start_session")
        if response.status_code == 200:
            data = response.json()
            self.session_id = data["session_id"]
            print("Session started. ID:", self.session_id)
        else:
            print("Failed to start session:", response.text)

    def clear_env():
        self.close()
        self.connect(self.server_url)

    def upload_file(self, file_path):
        """Upload a file to the server."""
        if not self.session_id:
            print("No active session. Start a session first.")
            return

        try:
            with open(file_path, 'rb') as file:
                files = {
                    "file": file
                }
                data = {
                    "session_id": self.session_id
                }
                response = requests.post(f"{self.server_url}/upload_config", files=files, data=data)

            if response.status_code == 200:
                print("File uploaded successfully:", response.json())
            else:
                print("Failed to upload file:", response.text)
        except FileNotFoundError:
            print(f"File not found: {file_path}")
        except Exception as e:
            print(f"An error occurred: {e}")

    def play(self, command):
        """Send a command to the server for a specific session and handle the response."""
        if not self.session_id:
            print("No active session. Start a session first.")
            return

        payload = {
            "session_id": self.session_id,
            "command": command
        }
        response = requests.post(f"{self.server_url}/process_command", json=payload)
        if response.status_code == 200:
            response_data = response.json()
            #print("Command processed successfully. Response:", response_data)
            self.act_on_response(command, response_data)
        else:
            print("Failed to process command:", response.text)

    def close(self):
        """End the session."""
        if not self.session_id:
            print("No active session to end.")
            return

        payload = {
            "session_id": self.session_id
        }
        response = requests.post(f"{self.server_url}/end_session", json=payload)
        if response.status_code == 200:
            print("Session ended successfully:", response.json())
        else:
            print("Failed to end session:", response.text)
        self.session_id = None

"""
if __name__ == "__main__":
    SERVER_URL = "http://127.0.0.1:5000"
    client = ConfigClient(SERVER_URL)

    # Start a session
    client.start_session()

    # Upload a file
    file_path = input("Enter the path to the configuration file: ")
    client.upload_file(file_path)

    # Send commands and act on responses
    while True:
        command = input("Enter a command for the server (or 'end' to terminate the session): ")
        if command.lower() == 'end':
            break

        client.send_command(command)

    # End the session
    client.end_session()
"""