import requests
import numpy as np
import json

class Motion:
    def __init__(self, dorna_in):
        self.server_url = None
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

    def gen_scene(self, world, flange):
        """Upload a file to the server."""
        if not self.session_id:
            print("No active session. Start a session first.")
            return

        try:
            result = {"world":[],"payload":[]}

            file_counter = 0 
            files = {}

            if world:
                for item in world:
                    file = open(item['file_path'], 'rb')
                    pose = self.dorna.kinematic.xyzabc_to_xyzquat(item['tvec'] + item['rvec'])
                    normalized_pose = [float(x) for x in pose]
                    result["world"].append({"file":"file"+str(file_counter),"pose":normalized_pose})
                    files["file"+str(file_counter)] = file
                    file_counter = file_counter + 1

            if flange:
                for item in flange:
                    file = open(item['file_path'], 'rb')
                    pose = self.dorna.kinematic.xyzabc_to_xyzquat(item['tvec'] + item['rvec'])
                    normalized_pose = [float(x) for x in pose]
                    result["payload"].append({"file":"file"+str(file_counter),"pose":normalized_pose})
                    files["file"+str(file_counter)] = file
                    file_counter = file_counter + 1

            result["session_id"] = self.session_id
            response = requests.post(f"{self.server_url}/gen_scene", data={"json": json.dumps(result)}, files = files)

            #for file in files:
            #    file.close()

            if response.status_code == 200:
                print("Scene generated successfully:", response.json())
            else:
                print("Failed to generate scene:", response.text)

        except FileNotFoundError:
            print(f"File not found: {file_path}")
        except Exception as e:
            print(f"An error occurred: {e}")

    def play(self, command):
        """Sends a command to the server for a specific session and handles the response."""
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


    def run(self, pose = [0,0,0,0,0,0], joint = None, tcp = [0, 0, 0, 0, 0, 0], current_joint = None, time = None , scene = None):
        
        xyzquat = [0,0,0,1,0,0,0]
        
        if not scene:
            scene = 1

        if joint is not None:  
            for i in range(6):
                joint[i] = joint[i] * np.pi / 180.0

            T = self.dorna.kinematic.t_flange_r_world(joint)
            xyzabc = self.dorna.kinematic.mat_to_xyzabc(T) 
            xyzquat = self.dorna.kinematic.xyzabc_to_xyzquat(xyzabc)

        else:
            xyzabc = pose
            T = self.dorna.kinematic.xyzabc_to_mat(xyzabc)

            #reversing the tcp effect
            inv_tcp_T = self.dorna.kinematic.xyzabc_to_mat([-x for x in tcp])
            T = np.matmul(T, inv_tcp_T)

            xyzabc = self.dorna.kinematic.mat_to_xyzabc(T)
            xyzquat = self.dorna.kinematic.xyzabc_to_xyzquat(xyzabc)

        #creating command for sending to the cuda server
        cmd = {}
        cmd["cmd"] = "motion"
        cmd["init"] = current_joint
        cmd["goal"] = xyzquat
        cmd["scene"] = scene
        res = self.play(cmd)
        
        if res:
            return res["ret"]
        else:
            return None

    def tcp_collision_cube_set(self, cubes):

        spheres = []

        for item in cubes:
            min_size = min(min(item.scale[0], item.scale[1]),item.scale[2])
            sphere_R = min_size/4.0
            nx = np.ceil(item.scale[0]/sphere_R/2)
            ny = np.ceil(item.scale[1]/sphere_R/2)
            nz = np.ceil(item.scale[2]/sphere_R/2)

            for i in range(nx):
                for j in range(ny):
                    for k in range(nz):
                        local_pos = np.array([i,j,k])

        return



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