# Dorna2 
This is a Python API for [Dorna 2][dorna] robotic arm.

## Installation
Notice that the program is compatible with Python 3.7+.

### Download
First, use `git clone` to download the repository:  
```bash
git clone https://github.com/dorna-robotics/dorna2-python.git
```
Or simply download the zip file, and uncompress the file.  

### Install
Next, go to the downloaded directory, where the `setup.py` file is located, and run:
```bash
python setup.py install
```
## Getting started
Import `dorna2` module.
``` python
from dorna2 import dorna

robot = dorna()

# use the robot WebSocket URL to establish a connection
if robot.connect("ws://dorna:443"):
	# helper function for jmove
	# robot.jmove({"rel": 0, "id": 100, "j0": 0, "j1": 0, "j2": 0, "j3": 0, "j4": 0}) 
	robot.jmove(rel = 0, id = 100, j0 = 0, j1 = 0, j2 = 0, j3 = 0, j4 = 0)

	# send any command via play() method
	# robot.play({"cmd": "jmove", "rel": 0, "id":101, "j1": 90, "j2": -90})
	robot.play(cmd = "jmove", rel = 0, id = 101, j1 = 90, j2 = -90)
```  
## Connect to the WebSocket server
The URL of the robot controller WebSocket server is `ws://robot_ip_address:443`. Where `robot_ip_address` is the IP address of the robot, and `443` is the port number. 
```python
# example: if ip = dorna
ws_url = "ws://dorna:443"

# example: if ip = 192.168.1.2
ws_url = "ws://192.168.1.2:443"
```
Use the WebSocket URL as a parameter and the `connect` method to establish a connection. The `connect` method returns `True` on a successful connection and `False` otherwise. 
``` python
from dorna2 import dorna

robot = dorna()

# use the robot WebSocket URL to establish a connection
if robot.connect("ws://dorna:443"):
	print("Connection was successful")
else:
	print("Connection was not successful")
```  
`ws.sock` is the core `WebSocket` object. After a successful connection use `ws.sock.connected` to keep track of the connection status.  
```python
from dorna2 import dorna

robot = dorna()

# use the robot WebSocket URL to establish a connection
if robot.connect("ws://dorna:443"):
	print(robot.ws.sock.connected) # True
else:
	print("Connection was not successful")
``` 
## Send command
There are many helper methods available to send commands to the robot, like `play, jmove, lmove, cmove, ... `. Assign the variable key and its value as parameter or submit a command as a dictionary. Use `play` method to send a valid command
``` python
robot.play(cmd = "jmove", rel = 0, id = 100, j1 = 90, j2 = -90)
# or
# robot.play({"cmd": "jmove", "rel": 0, "id":100, "j1": 90, "j2": -90})
``` 
Use `ws.send` method to directly send a JSON string command.
``` python
robot.ws.send('{"cmd": "jmove", "rel": 0, "id":100, "j1": 90, "j2": -90}')
``` 
Use `wait` method to wait for the completion of a command with an `id`. 
``` python
# command id = 100
robot.play(cmd = "jmove", rel = 0, id = 100, j1 = 90, j2 = -90)
robot.wait(100)

# {"id": 100, "stat": 2} has been received
print("command with id = 100 has been completed")
``` 
## Receive message
`sys` is a dictionary that holds the messages received by the API. Notice that, `sys` initialized with an empty dictionary. Every time a new JSON message sent by the controller and received by the API, `sys` updates itself according to the received message.
``` python
print(robot.sys) # {}

# command id = 100
robot.play(cmd = "jmove", rel = 0, id = 100, j1 = 90, j2 = -90)
robot.wait(100)

print(robot.sys) # {"id": 100, "stat": 2, ...}
``` 
The last 100 messages received by the API are stored in `msg` queue. `msg` is a queue object of size 100, `queue.Queue(100)`, and whenever it gets full it can not put any new message inside of it, unless some messages get out of it. So, it is the user responsibility to consume (`get()`) the `msg` in a safe way.   
``` python
# print received messages 
while True:
	if not robot.msg.empty()
		print(robot.msg.get())
``` 
Every time, a new message is sent by the WS server and received by the API, `connect_loop.on_message` method is triggered.

[dorna]: https://dorna.ai/
