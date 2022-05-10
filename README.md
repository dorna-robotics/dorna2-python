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

### `.connect(host="localhost", port=443, time_out=5)` 
Connect to the robot socket server at `ws://host:port`.

#### Parameters
- *host*: (string) The controller host address. The default value is `"localhost"`.
- *port*: (int) The controller port number. The default value is `443`.
- *time_out*: (float > 0) Waits maximum of `time_out` seconds to establish a connection to the robot controller. The default value is 5 seconds.

#### Returns
Returns `True` on a sucessful connection, otherwise `False`.

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
Once you connected to the controller, you can start sending valid messages (commands) to the robot.

### Command status and execution 
We need to first get familiar with the status of a command first. When sending a valid command to the robot, the controller reports the status of the command from the time that the command is submitted, to the time that the execution of the command is completed using a `stat` key and the unique `id` associated to the command. There are four main stages in a life cycle of a command:
- `stat = 0`: The command has been received by the controller with no error.
- `stat = 1`: The command execution has begun.
- `stat = 2`: The execution of the command is now completed.
- `stat < 0`: An eeror happened during the execution of the command and it will not be executed.

### `.play(time_out=-1, msg=None, **kwargs):`  
Send a message to the robot. There are multiple ways to send a message via `.play()`. For a better understanding, we send a simple `alarm` status command in three different ways:
1. Key and value format: `play(cmd='alarm', id=100})`
2. Python dictionary format: `play({'cmd': 'alarm', 'id': 100})` 
3. JSON string format: `play('{"cmd": "alarm", "id": 100}')`

#### `time_out`
The `time_out` key is used for tracking the completion or any error during the execution of the command. 
- `time_out < 0`: The function waits for the `stat = 2` or `stat < 0` of the command and then returns.
- `time_out >= 0`: The function waits maximum of `time_out` seconds for the  `stat = 2` or `stat < 0` of the command to arrive.

Here we explain the `time_out` parameter with few scenarios:
##### Scenario 1
Lets say we want to command the robot to get to an especific position and then enables output 0. In this case, in it important for us that the robot has achieved the desired position before enabling the output. This is how we do it:
``` python
""" scenario 1
time_out < 0: run the print after the motion comaand is completed.
"""
robot.play(time_out=-1, cmd="jmove", rel=1, j0=45)
print("motion is completed")

""" scenario 2
time_out = 0: run the print right after sending the motion command.
"""
robot.play(time_out=0, cmd="jmove", rel=1, j0=45)
print("motion is still in progress")

""" scenario 3
time_out > 0: wait for the completion of the motion command for certain amount of time, and then run the print. 
"""
robot.play(time_out=10, cmd="jmove", rel=1, j0=45)
print("We have waited maximum of 10 seconds for the completion of the above command")
``` 
### `play_script(script_path, time_out=0)`
Send all the messages that are stored in a script file to the robot controller. The method opens the script file located at `script_path`, read the file line by line and send each line as a command. 
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
Use the `time_out` parameter for waiting or not waiting for the execution of the commands in the script file.

### `.jmove(**kwargs)`
A helper function to send a `jmove` command. `jmove(j0=0, id=100)` is equivalent to `play(cmd='jmove', j0=0, id=100)`. Basically the `cmd` key is set to `"jmove"`.  

### `.rmove(**kwargs)`
Similar to `.jmove()` but the command key is equal to `"rmove"`.

### `.lmove(**kwargs)`
Similar to `.jmove()` but the command key is equal to `"lmove"`.

### `.cmove(**kwargs)`
Similar to `.jmove()` but the command key is equal to `"cmove"`.

### `.output(index=None, val=None, **kwargs)`
Set (enable or disable) or get the value of an output pin.
``` python
robot.output(0) # return the value of the out0
robot.output(0, 1) # set the value of the out0 to 1 and return its value
robot.output() # return the value of all the 16 outputs in a list of size 16
robot.output(out0=1, out2=0) # set the value of out0 to 1 and out2 to 0, and return the value of all the 16 outputs in a list 
``` 
#### Parameters
- *index*: (None or 0 <= int < 16) The index of the output pin that we are interested to set or get its value.
- *val*: (None or binary) The value we want to assign to the output pin `index`. If the `val` is not present or `None` then we are only getting (reading) the value of the output pin `index`.  
- *kwargs*: Other key and value parameters associated to this method. Including `time_out`, `queue`, `id`, etc. 

#### Return
Returns the value of output(s). If the `index` parameter is presented then the value of output pin `index` is returnred. Otherwise, the value of all the 16 output pins are returned in a list of size 16, where item `i` in the list is the value of `outi`.

### `.pwm(index=None, val=None, **kwargs)`
Set (enable or disable) or get the value of a pwm channel.
``` python
robot.pwm(0) # return the value of the pwm0
robot.pwm(0, 1) # set the value of the pwm0 to 1 and return its value
robot.pwm() # return the value of all the 5 pwms in a list of size 5
robot.pwm(pwm0=1, pwm2=0) # set the value of pwm0 to 1 and pwm2 to 0, and return the value of all the 5 pwms in a list 
``` 
#### Parameters
- *index*: (None or 0 <= int < 5) The index of the pwm channel that we are interested to set or get its value.
- *val*: (None or binary) The value we want to assign to the pwm channel `index`. If the `val` is not present or `None` then we are only getting (reading) the value of the pwm channel `index`.  
- *kwargs*: Other key and value parameters associated to this method. Including `time_out`, `queue`, `id`, etc. 

#### Return
Returns the value of pwm(s). If the `index` parameter is presented then the value of pwm channel `index` is returnred. Otherwise, the value of all the 5 pwm channels are returned in a list of size 5, where item `i` in the list is the value of `pwmi`.

### `.freq(index=None, val=None, **kwargs)`
Set or get the frequency value of a pwm channel.
``` python
robot.freq(0) # return the value of freq0
robot.freq(0, 10) # set the value of freq0 to 1 and return its value
robot.freq() # return the frequency value of all the 5 pwms in a list of size 5
robot.freq(freq0=10, freq2=20) # set the value of freq0 to 1 and freq2 to 0, and return the frequency value of all the 5 pwms in a list 
``` 
#### Parameters
- *index*: (None or 0 <= int < 5) The index of the pwm channel that we are interested to set or get its frequency value.
- *val*: (None or binary) The frequency value we want to assign to the pwm channel `index`. If the `val` is not present or `None` then we are only getting (reading) the frequency value of the pwm channel `index`.  
- *kwargs*: Other key and value parameters associated to this method. Including `time_out`, `queue`, `id`, etc. 

#### Return
Returns the value of freq(s). If the `index` parameter is presented then the frequency value of the pwm channel `index` is returnred. Otherwise, the frequency value of all the 5 pwm channels are returned in a list of size 5, where item `i` in the list is the value of `freqi`.

### `.duty(index=None, val=None, **kwargs)`
Set or get the duty cycle of a pwm channel.
``` python
robot.duty(0) # return the value of duty0
robot.duty(0, 10) # set the value of duty0 to 1 and return its value
robot.duty() # return the duty cycle of all the 5 pwms in a list of size 5
robot.duty(duty0=10, duty2=20) # set the duty0 to 1 and duty2 to 0, and return the duty cycles of all the 5 pwms in a list 
``` 
#### Parameters
- *index*: (None or 0 <= int < 5) The index of the pwm channel that we are interested to set or get its duty cycle.
- *val*: (None or binary) The duty cycle we want to assign to the pwm channel `index`. If the `val` is not present or `None` then we are only getting (reading) the duty cycle of the pwm channel `index`.  
- *kwargs*: Other key and value parameters associated to this method. Including `time_out`, `queue`, `id`, etc. 

#### Return
Returns the duty cycle(s). If the `index` parameter is presented then the duty cycle of the pwm channel `index` is returnred. Otherwise, the dudty cycle of all the 5 pwm channels are returned in a list of size 5, where item `i` in the list is the value of `dutyi`.

### `.input(index=None, **kwargs)`
Get the value of an input pin(s).
``` python
robot.input(0) # return the value of in0
robot.input() # return the value of all the 16 input pins in a list of size 16
``` 
#### Parameters
- *index*: (None or 0 <= int < 16) The index of the input pin that we are interested to get its value.
- *kwargs*: Other key and value parameters associated to this method. Including `time_out`, `queue`, `id`, etc. 

#### Return
Returns the value of input pin(s). If the `index` parameter is presented then the value of input pin `index` is returnred. Otherwise, the value of all the 16 input pins are returned in a list of size 16, where item `i` in the list is the value of `ini`.

### `.adc(index=None, **kwargs)`
Get the value of an adc channel(s).
``` python
robot.adc(0) # return the value of adc0
robot.adc() # return the value of all the 5 adc channels in a list of size 5
``` 
#### Parameters
- *index*: (None or 0 <= int < 5) The index of the adc channel that we are interested to get its value.
- *kwargs*: Other key and value parameters associated to this method. Including `time_out`, `queue`, `id`, etc. 

#### Return
Returns the value of adc channel(s). If the `index` parameter is presented then the value of adc channel `index` is returnred. Otherwise, the value of all the 5 adc channels are returned in a list of size 5, where item `i` in the list is the value of `adci`.

### `.probe(index=None, val=None, **kwargs)`
Set the probe input pin and return the the value of the joints when then input was matched the probe patteren.
``` python
robot.probe(1, 0) # return the value of joints, the moment in1 is 0
robot.probe(in0=0, in3=1) # return the value of joints the moment in0 is 0 and in3 is 1 
``` 
#### Parameters
- *index*: (None or 0 <= int < 16) The index of the input pin that we are interested to set for probe.
- *val*: (None or binary) The value we want to assign to the input pin `index` for the probe process. This basicall means to return the joints value of the robot when input pin `index` is equal to `val`.   
- *kwargs*: Other key and value parameters associated to this method. Including `time_out`, `queue`, `id`, etc. 

#### Return
Return the values of the robot joints the mooment the input pattern appears in a list of size 8. Where index `i` in the list is the value of joint joint `i` (`ji`).

### `.iprobe(index=None, val=None, **kwargs)`
This method is similar to the `probe` function but here we are matching an encoder index with a value, instead of an input pin. The encoder on the robot gets 1 8 times during one full rotation of the encoder, and we can locate this points by calling this function.
``` python
robot.iprobe(1, 1) # return the value of joints, the moment index1 (encoder 1 index) gets 1
robot.iprobe(in0=1, in3=1) # return the value of joints the moment index0 is 1 and index3 is 1 
``` 
#### Parameters
- *index*: (None or 0 <= int < 16) The index of the encoder that we are interested to set for iprobe.
- *val*: (None or binary) The value we want to assign to the encoder index `index` for the iprobe process. This basically means to return the joints value of the robot when encoder index `index` is equal to `val`.   
- *kwargs*: Other key and value parameters associated to this method. Including `time_out`, `queue`, `id`, etc. 

#### Return
Return the values of the robot joints the mooment the index pattern matches the value, in a list of size 8. Where index `i` in the list is the value of joint joint `i` (`ji`).


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
