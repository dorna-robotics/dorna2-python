# Dorna2 
This is a Python API for [Dorna 2][dorna] robotic arm.

## Installation
Notice that the program has been tested only on Python 3.7+.

### Download
First, use `git clone` to download the repository:  
```bash
git clone https://github.com/dorna-robotics/dorna2-python.git
```
Or simply download the [zip file](https://github.com/dorna-robotics/dorna/archive/master.zip), and uncompress the file.  

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
ip = "127.0.0.1"
host = 443
robot.connect(ip, host)

# your code

robot.close() # always close the socket when you are done
```  

## Connection
The robot WebSocket server runs on `ws://robot_ip_address:443`, where `robot_ip_address` is the IP address of the robot, and `443` is the port number.   
```python
# example: if ip = dorna
ws_url = "ws://dorna:443"

# example: if ip = 192.168.1.2
ws_url = "ws://192.168.1.2:443"
```

`.connect(host, port, time_out=1, init=True)`  
Connect to the robot WebSocket server at `ws://host:port`. The `host` and `port` arguments are similar to the Python `socket.connect((host, port))` method.


`.close()`  
Use this method to close a WS connection. Notice that `.close()` instantly closes the socket and terminates the communication loop. After this the `dorna` object is not able to send or receive any message from the robot WS server.  
> It is a good practice to close an open socket connection when your task is over and the connection is no longer required.  
``` python
from dorna2 import dorna

robot = dorna()
robot.connect("192.168.1.10", 443) # connect to ws://192.168.1.10:443

# your code

robot.close() # always close the socket when you are done
``` 

## Send message
Once you connected to the robot, you can start sending valid messages (commands) to the robot in JSON format.

`.play(message=None, **arg)`  
Send a message to the robot. There are multiple ways to send a message via `.play()`. For a better understanding, we send a simple `alarm` status command in three different ways:
1. JSON string format: `play('{"cmd": "alarm", "id": 100}')`
2. Python dictionary format: `play({'cmd': 'alarm', 'id': 100})` 
3. Key and value format: `play(cmd='alarm', id=100})`  

We have other helper methods to send a message:
- `.jmove(**arg)`: A helper function to send a `jmove` command. `jmove(j0=0, id=100)` is equivalent to `play(cmd='jmove', j0=0, id=100)`. Basically the `"cmd"` key is set to `"jmove"`.  
- `.lmove(**arg)`: Similar to `.jmove()` but the command key is equal to `"lmove"`.
- `.cmove(**arg)`: Similar to `.cmove()` but the command key is equal to `"cmove"`.

`.play_script(script_path="")`
Send all the messages that are stored in a script file to the robot. The method opens the script file located at `script_path`, read the file line by line and send each line as a command.  
> Notice that each message has to occupy exactly one line. Multiple messages in one line or one message in multiple line is not a valid format. Here are a valid and invalid script format:
``` python
# valid format: Each message occupy exactly one line
{"cmd":"jmove","rel":0,"j0":0}
{"cmd":"jmove","rel":0,"j0":10}
{"cmd":"jmove","rel":0,"j0":-10}

# invalid format: Multiple commands in one line or one command in multiple line
{"cmd":"jmove","rel":0,"j0":0}{"cmd":"jmove","rel":0,"j0":10}
{"cmd":"jmove","rel":0,
"j0":-10}
```     

## Receive message
After a successful WS connection, the robot starts to send messages in JSON format to the API.  

`sys` is a dictionary that holds the messages keys and values received by the API.  
> Notice that, `sys` initialized with an empty dictionary. Every time a new JSON received by the API, `sys` updates itself according to the received data.
``` python
print(robot.sys) # {}

# command id = 100
robot.play(cmd="jmove", rel=0, id=100, j1=90, j2=-90)
robot.wait(id=100, stat=2)

print(robot.sys) # {"id": 100, "stat": 2, ...}
``` 

`msg` queue (`queue.Queue(100)`) holds the last 100 JSON messages received from the robot.
``` python
# print received messages 
while True:
	if not robot.msg.empty()
		print(robot.msg.get())
``` 

`.wait(time_out=0, **arg)`  
Run a while loop and wait until a given pattern of keys and values in `arg` parameter appears in the `.sys`, and then it will break the loop and returns. If `time_out` is set to `0`, then the while loop runs (forever) until the pattern appears in the `sys` and then it will return `True`. When `time_out > 0`, the while loop runs for maximum of `time_out` seconds. If the pattern appears before `time_out` then the method returns `True`. Otherwise, it will return `False`.  
The `wait()` method is useful in many scenarios. Here we mention two of them.  
- **Wait for a command completion:** Let say you send a motion command to the robot and want to know exactly when the motion is completed to do something else after that. Notice that when the execution of a command (with an `id` field) is completed, the robot sends a confirmation message with `stat = 2` and the same `id`. So, the following code prints `completed` right after the motion command with `id = 100` is over.
``` python
# command id = 100
robot.play(cmd="jmove", rel=0, id=100, j1=90, j2=-90)
robot.wait(id=100, stat=2)
print("completed")
``` 
- **Wait for inputs:** Let say you are waiting for a pattern of inputs to appear an do something after that. Simply call `wait()` and your input pattern as the input argument. 
``` python
# waiting for input 0 to turn on and input 3 to turn off, and then do something else after that
robot.wait(in0=1, in3=0)
# do something
```

## Example
To learn more about the API, navigate to the main directory and check the `example` folder for some examples.

[dorna]: https://dorna.ai
