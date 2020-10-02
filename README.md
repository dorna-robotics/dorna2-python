# Dorna2 
This is a Python API for [Dorna 2][dorna] robotic arm.

## Installation
Notice that the program is compatible with Python 3.7+.

**Clone**  
First, use `git clone` to download the repository:  
```bash
git clone https://github.com/dorna-robotics/dorna2-python.git
```
Or simply download the zip file, and uncompressed the file.  
Next, go to the downloaded directory, where `setup.py` file is located, and run:
```bash
python setup.py install
```
## Getting started
Import `dorna2` module.
``` python
from dorna2 import dorna

robot = dorna()

# use the robot WebSocket URL to establish a connection
robot.connect("ws://dorna:443")

# helper function for jmove
# robot.jmove({"rel": 0, "id": 100, "j0": 0, "j1": 0, "j2": 0, "j3": 0, "j4": 0}) 
robot.jmove(rel = 0, id = 100, j0 = 0, j1 = 0, j2 = 0, j3 = 0, j4 = 0)

# send any command via play() method
# robot.paly({"cmd": "jmove", "rel": 0, "id":101, "j1": 90, "j2": -90})
robot.paly(cmd = "jmove", rel = 0, id = 101, j1 = 90, j2 = -90)
```  

## Send command
There are many helper methods available to send commands to the robot, like `play, jmove, lmove, cmove, ... `. Assign the variable key and its value as parameter or submit a commands as a dictionary. Use `play` method to send a valid command
``` python
robot.paly(cmd = "jmove", rel = 0, id = 100, j1 = 90, j2 = -90)
# or
# robot.paly({"cmd": "jmove", "rel": 0, "id":100, "j1": 90, "j2": -90})
``` 
Use `ws.send` method to directly send a string command.
``` python
robot.ws.send('{"cmd": "jmove", "rel": 0, "id":100, "j1": 90, "j2": -90}')
``` 
Use `wait` method to wait for a completion of a command with `id`. 
``` python
# command id = 100
robot.paly(cmd = "jmove", rel = 0, id = 100, j1 = 90, j2 = -90)
robot.wait(100)
# command with id = 100 has been completed, i.e., {"id": 100, "stat": 2} has been received
``` 
## Receive message
`sys` is a dictionary that holds the messages received by the API. Notice that, `sys` initialized with an empty dictionary. Every time a new JSON message sent by the controller and received by the API, `sys` updates itself according to the received message.
``` python
print(robot.sys)
# {}

# command id = 100
robot.paly(cmd = "jmove", rel = 0, id = 100, j1 = 90, j2 = -90)
robot.wait(100)
# command with id = 100 has been completed, i.e., {"id": 100, "stat": 2} has been received

print(robot.sys)
# {"id": 100, "stat": 2, ...}
``` 
The last 100 messages received by the API is stored in `msg` queue. `msg` is a queue object of size 100, `queue.Queue(100)`, and once it is full it can not put any new message inside of it, messages get out of it. So, it is the user responsibility to consume the `msg` messages in a safe way.   
``` python
# print received messages 
while True:
	if not robot.msg.empty()
		print(robot.msg.get())
``` 
Notice that, `connect_loop.on_message` method is triggered when a new message received by the API.

[dorna]: https://dorna.ai/
