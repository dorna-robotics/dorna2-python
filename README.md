# Dorna 2 Python API
This is the Python API tutorial for [Dorna][dorna] robotic arm.

## Setup
Notice that the program has been tested only on Python 3.7+.

### Repository
You can find the code repositry on [GitHub](https://github.com/dorna-robotics/dorna2-python), and report your technical issues [there](https://github.com/dorna-robotics/dorna2-python/issues).

### Download
First, use `git clone` to download the repository:  
```bash
git clone https://github.com/dorna-robotics/dorna2-python.git
```
Or simply download the [zip file](https://github.com/dorna-robotics/dorna2-python/archive/master.zip), and uncompress the file.  

### Install
Next, go to the downloaded directory, where the `setup.py` file is located, and run:
```bash
python setup.py install --force
```
Notice that, on UNIX systems you need to use `sudo` prefix for admin previliages and installing the requirements:
```bash
sudo python3 setup.py install --force
```

### Getting started
Import `dorna2` module.
``` python
from dorna2 import Dorna

host = "127.0.0.1"
port = 443

# create the Dorna object
robot = Dorna()

# connect to the robot
robot.connect(host, port)

""" Your code goes here
"""

# always close the socket connection when your code is over
robot.close()
```  

## Connection
The robot socket server runs on `ws://robot_ip_address:443`, where `robot_ip_address` is the IP address of the robot, and `443` is the port number. Once the connection has been established between the robot and the client (user), they start communicating with each other by sending and receiving data in [JSON][json] format. 
```python
# for example: if ip = dorna
ws_url = "ws://dorna:443"

# for example: if ip = 192.168.1.2
ws_url = "ws://192.168.1.2:443"
```

### <code>.connect(*host, port=443*)</code> 
Connect to the robot socket server at `ws://host:port`.

#### Parameters
- *host*: (string) The controller host address.
- *port*: (int) The controller port number. The default value is `443`. 

> The `host` (string) and `port` (integer) arguments are similar to the Python `socket.connect((host, port))` method.

### `.close()` 
Use this method to close an opened connection. Notice that `.close()` instantly closes the socket and terminates the communication loop. After this the `Dorna` object is unable to send or receive any message from (to) the controller server.  
> It is a good practice to close an open socket connection when your task is over and the connection is no longer required.  
``` python
from dorna2 import Dorna

robot = Dorna()
robot.connect("10.0.0.10") # connect to ws://10.0.0.10:443

# your code

robot.close() # always close the socket when you are done
``` 

## Send message
Once you connected to the robot, you can start sending valid messages (commands) to the robot.
Here in [JSON][json] format.

### `.play(self, track=False, message=None, **arg)`  
Send a message to the robot. There are multiple ways to send a message via `.play()`. For a better understanding, we send a simple `alarm` status command in three different ways:
1. JSON string format: `play('{"cmd": "alarm", "id": 100}')`
2. Python dictionary format: `play({'cmd': 'alarm', 'id': 100})` 
3. Key and value format: `play(cmd='alarm', id=100})`  

The `track` key is used for tracking the completion of the command. By default `track` is set to `False`, but if you want to track your command and see when the execution of the command is over set it to `True`. Read more about this in the [Tracking command](#tracking-command)

The `play` method returns `None` if `track=False` and returns a `track_msg` object if `track=True`.  

We have few other helper methods to send a message:

### `.jmove(track=False, **arg)`
A helper function to send a `jmove` command. `jmove(j0=0, id=100)` is equivalent to `play(cmd='jmove', j0=0, id=100)`. Basically the `"cmd"` key is set to `"jmove"`.  

### `.rmove(track=False, **arg)`
Similar to `.jmove()` but the command key is equal to `"rmove"`.

### `.lmove(track=False, **arg)`
Similar to `.jmove()` but the command key is equal to `"lmove"`.

### `.cmove(track=False, **arg)`
Similar to `.jmove()` but the command key is equal to `"cmove"`.

### `.play_script(script_path="")`
Send all the messages that are stored in a script file to the robot. The method opens the script file located at `script_path`, read the file line by line and send each line as a command.  
> Notice that each message has to occupy exactly one line. Multiple messages in one line or one message in multiple line is not a valid format. As an example, here we show a valid and invalid script format:
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

### `.sys` 
This dictionary holds the messages keys and values received by the API.  
> Notice that, `sys` initialized with an empty dictionary. Every time a new JSON received by the API, `sys` updates itself according to the received data.
``` python
print(robot.sys) # {}

# command id = 100
trk = robot.play(True, cmd="jmove", rel=0, id=100, j1=90, j2=-90)
trk.complete()

print(robot.sys)  # {"id": 100, "stat": 2, ...}
``` 

### `.msg` 
A Python queue of size 100 (`queue.Queue(100)`) that holds the last 100 JSON messages received from the robot.  
> Notice that each element in `msg` is a Python dictionary, and it is the user responsibility to dequeue and process this queue according to their needs.
``` python
# print received messages 
while True:
    if not robot.msg.empty()
        print(robot.msg.get())
``` 

### `.wait(time_out=0, **arg)`  
Run a while loop and wait until a given pattern of keys and values in `arg` parameter appears in the `.sys`, and then it will break the loop and returns. If `time_out` is set to `0`, then the while loop runs (forever) until the pattern appears in the `sys` and then it will return `True`. When `time_out > 0`, the while loop runs for maximum of `time_out` seconds. If the pattern appears before `time_out` then the method returns `True`. Otherwise, it will return `False`.  
The `wait()` method is useful in many scenarios. Here we mention one.  

#### Wait for inputs
Let say you are waiting for a pattern of inputs to appear an do something after that. Simply call `wait()` and your input pattern as the input argument. 
``` python
# waiting for input 0 to turn on and input 3 to turn off, and then do something else after that
robot.wait(in0=1, in3=0)
# do something
```
## Tracking command
When the user sends a command to the robot, the robot starts to process the command and execute it. Depending on the command status the robot report messages to the user. These messages report the status of each command that is sent to the controller, from the time that the command is submitted, to the time that the execution of the command is completed. These messages will only be sent to the user if the corresponding command has an `id` field with a positive integer value. The returned message will have the same id as the command itself. An example of such a message is as follows:
```
{"id":12, "stat":2}
```
It means that the command with `"id":12`, is in status 2. The `stat` field can take different values with the following interpretations:

| Stat      | Description |
| ----------- | ----------- |
| 0           |The command has been received by the controller|
| 1           |The command execution has begun|
| 2           |The execution of the command is completed|
| 2           |The command has an error and it will not be executed|

To track a command set the `track` parameter in `.play()` method to `True`. For better understanding, let say we send a `jmove` command to the robot and we want to wait for the completion of this command.
```python
cmd = {"cmd":"jmove", "rel":1, "j0": 90, "id":100}
trk = robot.play(True, **cmd)  # set the track to True for tracking the completion
trk.complete()
print("command execution is completed")
```
The `trk` variable returned by the `.play()` method is a `track_cmd` object and it has multiple multiple useful methods. Here we go over one of them.

### `.complete(time_out=0)`
Use this method to wait for the completion of a command sent by the `.play()` method. If `time_out` is `0` then the method waits until the status of the command is 2 (completed) or negative (error happened). If the `time_out > 0` then the method waits for the maximum of `time_out` seconds for the completion or any error in the execution of the command. This method returns the status of the command. For example, if it returns `2` then it means that the execution of the command is completed.

## Example
To learn more about the API, navigate to the main directory of the repository and check the `example` folder for more examples.  
https://github.com/dorna-robotics/dorna2-python/tree/master/example

[dorna]: https://dorna.ai
[json]: https://www.json.org
