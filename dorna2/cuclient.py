import requests
import numpy as np

class cuda_client:
    def __init__(self, dorna_in, server_url = None):
        self.server_url = server_url
        self.session_id = None
        self.dorna = dorna_in

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
            return response_data


        return None


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

    def motion_gen(self, command, init_joints):
        xyzquat = [0,0,0,1,0,0,0]
        if command["cmd"] == "jmove":

            joint = [init_joints[0],init_joints[1],init_joints[2],init_joints[3],init_joints[4],init_joints[5]]

            for i in range(6):
                key = "j" + str(i)
                if key in command:
                    joint[i] = command[key]

                joint[i] = joint[i] * np.pi / 180.0

            T = self.dorna.kinematic.t_flange_r_world(joint)
            xyzabc =self.dorna.kinematic.mat_to_xyzabc(T) 
            xyzquat = self.dorna.kinematic.xyzabc_to_xyzquat(xyzabc)


        if command["cmd"] == "lmove":
            xyzabc = [command["x"]/1000, command["y"]/1000, command["z"]/1000, command["a"], command["b"], command["c"]]
            T = self.dorna.kinematic.xyzabc_to_mat(xyzabc)
            T = np.matmul(T, self.dorna.kinematic.inv_T_tcp_r_flange)
            xyzabc = self.dorna.kinematic.mat_to_xyzabc(T)
            xyzquat = self.dorna.kinematic.xyzabc_to_xyzquat(xyzabc)

        init_joints[5] = init_joints[5] - 180 #correct a 180 value
        init_joints[5] = (init_joints[5] + 180) % 360 - 180 #normalize vector between -180 to 180

        cmd = {}
        cmd["cmd"] = "motion"
        cmd["init"] = init_joints
        cmd["goal"] = xyzquat

        res = self.play(cmd)
        return res["ret"]

    def list_motion_gen(self, points, init_joints):
        if not self.session_id:
            print("No active session to send commands.")
            return

        last_joints = init_joints
        total_list = []
        sizes = []
        for command in points:
            res = self.motion_gen(command, last_joints)
            last_joints = res[-1]
            total_list = total_list + res
            sizes = sizes + [len(res)]
        print("sizes: ",sizes)
        return total_list

    def cuda_run_motion(self, points, init_joints):
        if not self.session_id:
            print("No active session to send commands.")
            return

        res = self.list_motion_gen(points, init_joints)
         for r in res:
            command = {"cmd":"jmove", "j0":r[0], "j1":r[1], "j2":r[2], "j3":r[3], "j4":r[4], "j5":r[5], "cont":1}
            self.dorna.play_dict(cmd=command, timeout=-1) 


        return True


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