# Dorna 2 Python API
This is the Python API tutorial for [Dorna][dorna] robotic arm.

## Setup
> Follow the [upgrade process](https://doc.dorna.ai/docs/guides/update-robot/) to make sure that you have the latest Firmware, API and software on your robot controller.
Notice that the program has been tested only on Python 3.7+.

### Repository
You can find the code repository on [GitHub](https://github.com/dorna-robotics/dorna2-python), and report your technical issues [here](https://github.com/dorna-robotics/dorna2-python/issues).

### Download
First, use `git clone` to download the repository:  
```text
git clone https://github.com/dorna-robotics/dorna2-python.git
```
Or simply download the [zip file](https://github.com/dorna-robotics/dorna2-python/archive/master.zip), and unzip the file.  

### Install
Next, go to the downloaded directory, where the `requirements.txt` file is located, and run:
```bash
# install requirements
pip install -r requirements.txt
```
Finally
```bash
pip install . --upgrade --force-reinstall
```

### Getting Started
First, import `Dorna` class from the `dorna2` module, and then create a `Dorna` object.
``` python
from dorna2 import Dorna

# create the Dorna object
robot = Dorna()
```  

## Communication
The robot server runs on `ws://host:443`, where `host` is the host address (IP) of the robot controller, and `443` is the default port number. Once the connection has been established between the robot and the client (user), they start communicating with each other by sending and receiving data in [JSON][json] format. 

### `.connect(host="localhost", port=443, handshake_timeout=5)`
Connect to the robot controller server at `ws://host:port`. Returns `True` on a successful connection, otherwise `False`.  
#### Parameter
- *host*: (string, default value = `"localhost"`) The controller host address.
- *port*: (int, default value = `443`) The controller port number.
- *handshake_timeout*: (float > 0, default value = `5`) Wait maximum of `handshake_timeout` seconds to establish a connection to the robot controller.

> The `host` (string) and `port` (integer) arguments are similar to the Python `socket.connect((host, port))` method.

### `.close()` 
Use this method to close an opened connection. This method instantly closes the socket and terminates the communication loop. After this the `Dorna` object is unable to send or receive any message from (to) the controller server.  

> It is required to close an open connection when your task is over and the connection is no longer required.  
``` python
from dorna2 import Dorna

robot = Dorna()
robot.connect("10.0.0.10") # connect to the robot server at ws://10.0.0.10:443

# your code

robot.close() # always close the socket when you are done
``` 
## Robot Messages and Command Status
Once you are  connected to the robot controller, you can start sending valid commands to the robot, and receive messages from the controller.   

### Command Status
We need to first get familiar with the status of a command sent to the robot. When sending a valid command to the robot, the controller reports the status of the command from the time that the command is submitted, to the time that the execution of that command is completed using a `stat` key and the unique `id` send initially with the command. An example of such a message is as follows:
Send a command to read the alarm status of the system, with `id=10`:
``` python
{"cmd":"alarm","id":10}
```
Receive the following messages one by one from the controller:
``` python
{"id":10,"stat":0}
{"id":10,"stat":1}
{"cmd":"alarm","id":10,"alarm":0}
{"id":10,"stat":2}
```
#### ID Key
As you can see in the above example, we sent a command to the robot controller to get the alarm status of the robot. 
The robot replies back by sending multiple messages. Our initial command had an `id` field equal to `10`. So, any messages from the robot associated to this command has the same `id` value.  

#### Stat Key
Another important field in the received massages (robot replies) is the `stat` key. The `stat` field can take different values with the following interpretations:
- `stat = 0`: The command has been received by the controller with no error.
- `stat = 1`: The command execution has begun.
- `stat = 2`: The execution of the command is now completed, with no error.
- `stat < 0`: An error happened during the execution of the command and it will not be executed.

> Notice, that we say that a command is completed if we receive `stat = 2` or `stat < 0` of that command. That basically means the command is completed, no longer running and its life cycle is over.

### `.track_cmd()`
Return the replies of the last commands sent to the robot.
This method returns a nested Python dictionary with three main keys as follow:
- `"cmd"`: The value assigned to this key, is a Python dictionary of the initial command sent to the robot.
- `"all"`: The value assigned to this key is a list of all replies from the robot controller, that has the same `id` as the initial command. Each element in the list is a Python dictionary. Elements in the list are also sorted in an ascending order, based on the time they have received by the API. So, first element in the list was received earlier than the last message in the list.
- `"merge"`: This is a Python dictionary formed by merging all the elements in the `"all"` list, and keep the most recent value for each key.  

Here is an example of showing the result of `.track_cmd()`, based on the replies we got from the `alarm` command we sent in the [Command Status](#command-status):
``` python
robot.track_cmd()
"""
{
    "cmd": {"cmd":"alarm","id":10},
    "all": [{"id":10,"stat":0},
            {"id":10,"stat":1},
            {"cmd":"alarm","id":10,"alarm":0},
            {"id":10,"stat":2}],
    "merge": {"id":10,"stat":2,"cmd":"alarm","alarm":0} 
}
```

## Send Commands 
In this section we cover two main methods to send commands to the robot.
### `.play(timeout=-1, msg=None, **kwargs):`  
Send a message to the robot, and return [`.track_cmd()`](#track_cmd).  
There are multiple ways to send a message via `.play()`. For a better understanding, we send a simple `alarm` status command in three different ways:
- **Case 1:** (*Recommended*) Key and value format: `play(cmd="alarm", id=10)`
- **Case 2:** Python dictionary format: `play({"cmd": "alarm", "id": 10})` 
- **Case 3:** JSON string format: `play('{"cmd": "alarm", "id": 10}')`

#### Parameter
- *timeout*: (float, default value = `-1`) We can assign different values to the `timeout` parameter depending on your scenario:
    - `timeout < 0`: Send a command and wait for the command completion (`stat = 2` or `stat < 0`) and then return from the function. At this moment we are sure that the command is no longer running.
    - `timeout >= 0`: Send a command and wait for A maximum of `timeout` seconds for its completion. Notice that in this case, we might have returned from the `.play` method but the command which was sent to the robot is still running or waiting inside the controller queue for its turn to get executed. If we do not want to wait for the execution of a command at all, then we can always set `timeout = 0`.
- *msg*: (Python dictionary or JSON string, default value = `None`) Use this parameter if you want to send your command in a Python dictionary format (**Case 2**), or in a JSON format (**Case 3**).
- *kwargs*: Use this to send your command in a key and value format (**Case 1**).

> Throughout this document we use and refer to the `timeout` key as an arguments inside methods that are sending command to the robot. These functions are using the `timeout` argument for tracking the completion or any error during the execution of the command that they are sending.
> The `.play()` method always includes a random `id` field to your command if it is not present.

For better understanding of the `timeout` parameter, we send a joint move command to the robot in four different ways.
``` python
# motion 1
start = time.time()
robot.play(timeout=-1, cmd="jmove", rel=1, j0=10, vel=1)
print(f"Motion 1 is completed, and took {time.time()-start} seconds.")

# motion 2
start = time.time()
robot.play(timeout=100, cmd="jmove", rel=1, j0=10, vel=1)
print(f"Motion 2 is completed, and took {time.time()-start} seconds.")

# motion 3
robot.play(timeout=2, cmd="jmove", rel=1, j0=10, vel=1)
print("2 second has passed and motion 3 is still running.")

# motion 4
robot.play(timeout=0, cmd="jmove", rel=1, j0=10, vel=1)
print("Motion 3 is still running and motion 4 is waiting for its execution.")

# Output:
#
#     The motion is completed
#     Motion 1 is completed, and took 10.199519157409668 seconds.
#     Motion 2 is completed, and took 10.207868814468384 seconds.
#     2 second has passed and motion 3 is still running.
#     Motion 3 is still running and motion 4 is waiting for its execution.
```


### `.play_script(script_path, timeout=-1)`
Send all the messages that are stored in a script file to the robot controller. The method opens the script file located at `script_path`, read the file line by line and send each line as a command, instantly.  
The `timeout` parameter acts similar to the [`.play()`](#playtimeout-1-msgnone-kwargs) method:
- `timeout < 0`: The method sends all the commands in the script, and returns when all those commands are completed.
- `timeout > 0`: The method sends all the commands in the script file and returns instantly
- `timeout > 0`: The method sends all the commands in the script file and waits maximum of `timeout` seconds for the completion of those commands before returning.  
This method returns:
- `2`: If all the commands in the script file are completed.   
- `0` or `1`: If the commands in the script file are still running.
- `Negative`: If there are some sort of error during the execution of the commands in the script file.

> Use this function to send multiple messages at once to the robot.
> Notice that each message has to occupy exactly one line. Multiple messages in one line or one message in multiple line is not a valid format. 
As an example, here we show a valid and invalid script format:
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
Here are two examples, on how to run a script file multiple times using a simple loop:
``` python
# case 1: run an script 10 times
for i in range(10):
    robot.play_script(script_path="test.txt")
    robot.log("Script is completed")
```
Another example (safe way):
``` python
# case 2: safe way of running an script in a for loop, by checking the return of the script
for i in range(10):
    result = robot.play_script(script_path="test.txt")
    if result != 2: # stat !=2
        robot.log("Error happened")
        break
    robot.log("Script is completed")
``` 

## Messages
### `.send()`
Return the last message sent to the controller, in a Python dictionary format.

### `.recv()`
Return the last message received from the controller, in a Python dictionary format.
``` python
print(robot.recv())

# Output:
#
#     {'cmd':'output','id':81513,'out0':1,'out1':0,'out2':0,'out3':0,'out4':0,'out5':0,'out6':0,'out7':0,'out8':0,'out9':0,'out10':0,'out11':0,'out12':0,'out13':0,'out14':0,'out15':0}
```
### `.sys()`
Return a Python dictionary, consists of all the keys and their most up to date values received from the controller, since the connection has been established with the robot. 

## Move
In this section we cover robot motion functions.

### `.jmove(**kwargs)`
A helper function to send a [joint move](https://doc.dorna.ai/docs/cmd/joint%20move/) (`jmove`) command to the robot, and return the stat of the motion command sent.  
This method is basically similar to the [`.play()`](#playtimeout-1-msgnone-kwargs) method but the `cmd` key is set to `"jmove"`. So, `.jmove(rel=1, j0=10, id=10, timeout=-1)` is equivalent to `.play(cmd='jmove', rel=1, j0=10, id=10, timeout=-1)`.
#### Parameter
- *kwargs*: The keys and values associated to a [joint move](https://doc.dorna.ai/docs/cmd/joint%20move/) command.  

> Notice that the `timeout` parameter exists here, and acts similar to the `timeout` parameter in [`.play()`](#playtimeout-1-msgnone-kwargs). The default value is `timeout=-1`. So, if you want to send a motion command without waiting for its completion, you have to explicitly set `timeout=0`.

### `.lmove(**kwargs)`
A helper function to send a [line move](https://doc.dorna.ai/docs/cmd/line%20move/) (`lmove`) command. This function is similar to the [`.jmove()`](#jmovekwargs) method, but this time the motion command is `lmove`.

### `.cmove(**kwargs)`
A helper function to send a [circle move](https://doc.dorna.ai/docs/cmd/circle%20move/) (`cmove`) command. This function is similar to the [`.jmove()`](#jmovekwargs) method, but this time the motion command is `cmove`.

## Stop
Series of helper function to send stop (halt) command, read and set the alarm status of the robot.

### `.halt(accel=None)`
A helper function to send a [halt](https://doc.dorna.ai/docs/cmd/halt/) command to the robot, with a given acceleration ratio (`accel`), and return the final status of the command (`stat`).
#### Parameter
- *accel*: (float > 1, default value = `None`) The acceleration ratio parameter associated to the `halt`. Larger `accel` means faster and sharper halt (stop). When this parameter is not present, the robot stops with the default acceleration.
``` python
robot.halt() # send a halt command to the controller
robot.halt(5) # send a halt  command with acceleration ratio equal to 5 
``` 

### `.get_alarm()`
Get the robot alarm status (0 for disabled and 1 for enabled).

### `.set_alarm(enable=None)`
Set the alarm status of the robot to `enable` (0 for disabling and 1 for enabling the alarm), and return the final status of the command (`stat`).
``` python
# Disable the alarm, if alarm exists
if robot.get_alarm():
    robot.set_alarm(0)
``` 

## Joint and TCP
In this section we cover methods that are related to the robot orientation.

### `.get_all_joint()`
Get the joint values of the robot, in a list of size 8. Where index `i` in the list is the value of joint `i` (`ji`).

### `.get_joint(index=None)`
Get the value of the joint `index` (0 <= int < 8).

### `.set_joint(index=None, val=None)`
Set the value of the joint `index` (0 <= int < 8) to `val` (float) and return the final status of the joint command (`stat`) sent to the robot.

``` python
robot.get_all_joint() # return the value of all the 8 joints of the robot
robot.get_joint(1) # return the value of j1
robot.set_joint(1, 30) # set the value of j1 to 30
``` 

### `.get_all_pose()`
Get the value of the robot toolhead (TCP) in Cartesian coordinate system (with respect to the robot base frame). in a list of size 8. Where indices 0 to 7 in this list are associated to `x`, `y`, `z`, `a`, `b`, `c`, `d` and `e`, respectively.

### `.get_pose(index=None)`
Get the value of the `index`th (0 <= int < 8) element in [`.get_all_pose()`](# get_all_pose).
``` python
robot.get_all_pose() # return [x, y, z, a, b, c, d, e] 
robot.get_pose(2) # return the value of the z coordinate 
``` 

### `.get_toollength()`
Get the value of the robot toollength in mm. The toollength is measured in the Z direction of the robot TCP frame.

### `.set_toollength(length=None)`
Set the robot toollength (mm) to `length` and return the final status of the toollength command (`stat`) sent to the robot.

``` python
robot.get_toollength() # get the robot toollength in mm
robot.set_toollength(10) # set the robot toollength to 10 mm  
``` 

## I/O
In this section we cover methods that are related to the robot inputs and outputs.

### `.get_all_output()`
Get the value of all the 16 output pins in a list of size 16. Where item `i` in the list is the value of `outi`.

### `.get_output(index=None)`
Get the value of output pin `index` (0 <= int < 16).

### `.set_output(index=None, val=None, queue=None)`
Set the value of the output pin `index` to `val`, and return the final status of the output command (`stat`) sent to the robot.
``` python
robot.get_all_output() # return the value of all the 16 outputs in a list of size 16
robot.get_output(0) # return the value of the out0
robot.set_output(0, 1) # set the value of the out0 to 1
``` 

### `.get_pwm(index=None)`
Get the value of the pwm channel `index` (0 <= int < 5). 

### `.set_pwm(index=None, enable=None, queue=None)`
Set the value of the pwm channel `index` to `enable` (0 for disable and 1 for enable), and return the final status of the pwm command (`stat`) sent to the robot.
``` python
robot.get_pwm(0) # return the value of the pwm0
robot.set_pwm(0, 1) # enable pwm channel 0
``` 

### `.get_freq(index=None)`
Get the frequency of a pwm channel `index` (0 <= int < 5).

### `.set_freq(index=None, freq=None, queue=None)`
Set the frequency value of the pwm channel `index` to `freq` (0 <= float <= 120,000,000), and return the final status of the pwm command (`stat`) sent to the robot.
``` python
robot.get_freq(0) # return the frequency value of the pwm channel 0 (freq0)
robot.set_freq(0, 1000) # set freq0 to 1000
``` 

### `.get_duty(index=None)`
Get the duty cycle of the pwm channel `index` (0 <= int < 5).

### `.set_duty(index=None, duty=None, queue=None)`
Set the duty cycle of the pwm channel `index` to `duty` (0 <= float <= 100), and return the final status of the pwm command (`stat`) sent to the robot.
``` python
robot.get_duty(0) # return the value of duty0
robot.set_duty(0, 10) # set the value of duty0 to 1 and return its value
``` 
### `.get_all_input()`
Get the value of all the input pins in a list of size 16, where index `i` in the list is the value of `ini`.

### `.get_input(index=None)`
Get the value of the input pin `index` (0 <= int < 16)
``` python
robot.get_all_input() # return the value of all the 16 input pins in a list of size 16
robot.get_input(0) # return the value of in0
``` 

### `.get_all_adc()`
Get the value of all the adc channels in a list of size 5, where item `i` in the list is the value of `adci`.

### `.get_adc(index=None)`
Get the value of the adc channel `index` (0 <= int < 5)
``` python
robot.get_all_adc() # return the value of all the 5 adc channels in a list of size 5
robot.get_adc(0) # return the value of adc0
``` 

## Wait
Wait for an input pins pattern, encoder indices or certain amount of time in your program.

### `probe(index=None, val=None)`
Return the joint values of the robot in a list of size 8 ([`.get_all_joint()`](#get_all_joint)), the moment that the input pin `index` (0 <= int < 16), is equal to the `val` (0 or 1).
> Use this method to wait for a pattern in an input pin.
``` python
robot.probe(1, 0) # return the joint values, the moment in1 gets equal to 0
``` 

### `iprobe(index=None, val=None)`
This method is similar to the `probe` function but here we are waiting for an specific pattern in the encoder indices, instead of the input pins.
Return the joint values of the robot in a list of size 8 ([`.get_all_joint()`](#get_all_joint)), the moment that the encoder index `index` (0 <= int < 8), is equal to the `val` (0 or 1).    
> Notice that the encoder on the motors gets high (1), 8 times during one full rotation of the encoder, and we can locate these points by calling the `.iprobe` function.
``` python
robot.iprobe(1, 1) # return the value of the joints, the moment that index1 (encoder 1 index) gets 1
``` 
### `.sleep(val=None)`
Sleep for `val` (float >= 0) seconds and return the status of the command.
``` python
robot.sleep(10) # the controller sleeps for 10 seconds
``` 

## Setting
### `.get_motor()`
Get the robot motors status (0 for disabled and 1 for enabled).

### `.set_motor(enable=None)`
Enable or disable the motors and return the final status of the motor command (`stat`) sent to the robot.
``` python
robot.get_motor() # get the robot motor status
robot.set_motor(0) # disable the motors  
``` 

### `.get_gravity()`
Get the gravity parameters of the robot in a list of size 5. The list consists of `[enable, mass, x, y, z]`. Where:
- `enable` (0 or 1): Indicates that if the gravity compensation feature is enabled (1) or disabled (0).
- `mass`: Mass of the payload in gram.
- `x` ,`y`, `z`: The coordinate of center of mass of the payload with respect to the robot flange frame in `mm`.

### `.set_gravity(enable=None, mass=None, x=None, y=None, z=None)`: 
Set and configure the gravity compensation parameters and return the final status of the gravity command (`stat`) sent to the robot.
``` python
robot.set_gravity(enable=1, mass=100, z=10) # enable gravity compensation, with 100 gram mass, located at z=10 mm far from the robot flange.    
``` 

### `.get_axis(index=None)`
Get the ratio (unit per motor turn) of the auxiliary axis `index` (5 <= int < 8)

### `.set_axis(index=None, ratio=None)`
Set the ratio ratio of the auxiliary axis `index` (5 <= int < 8) to `ratio` (int > 0), and return the final status of the axis command (`stat`) sent to the robot.
``` python
robot.get_axis(5) # get the ratio of axis 5
robot.set_axis(5, 1200) # 1 turn of the motor 5 is equal to 1200 unit.    
``` 

### `.get_pid()`
Return the PID parameters of the robot, in a list of size 2 (`[threshold, duration]`.
### `.set_pid(threshold=None, duration=None)`
Set the `threshold` (int > 0) and `duration` (int > 0) parameter of the robot, and return the final status of the pid command (`stat`) sent to the robot.
### `.reset_pid()`
Reset the PID of the robot to it's default configuration, and return the final status of the pid command (`stat`) sent to the robot.
``` python
robot.set_pid(20, 50) # set threshold to 20 and duration to 50.    
robot.reset_pid(20, 50) # reset the PID    
``` 

## Info
### `.version()`
Get the firmware version of the controller.
``` python
robot.version() # get the firmware version
``` 
### `.uid()`
Get the controller Universal Identification number.
``` python
robot.uid() # get the controller 
``` 
## Callback Event
Every time a message received from the robot controller, we can call (trigger) a function. This is useful when you want to create an event, based on the message received from the controller.

### `.register_callback(fn)`
Register an *asynchronous* function `fn` to be called every time a message received from the controller.  
#### Format of the Function `fn(msg, sys)`
As we mentioned `fn` is a [asynchronous python function](https://docs.python.org/3/library/asyncio.html), and it is important that we define it in `async` format, otherwise it will cause problem to the robot and API communication.  
This method takes two parameters `msg` and `sys`. 
- `msg` is the message received from the controller, the moment that `fn` was called.
- `sys` is the dictionary defined in [`.sys()`](#sys), the moment that `fn` was called.

### `.deregister_callback()`
This method acts opposite of[`.register_callback()`](#register_callbackfn) and it removes any function `fn` from the callback.  
> It is important to call this method, when we do not need the registered callback function `fn` anymore.

### Example
Assume that you have a program running a script in while loop. Meanwhile your robot input is connected to an external device.  
We want to stop the robot (put the robot in alarm mode) and end the while loop, one second after an input 0 is enabled (`in0 == 1`).  
So, we register a callback method, that looks at the received message from the controller, and send an alarm message if `in0 == 1`.   
``` python
from dorna2 import Dorna
import asyncio


class call_back(object):
    """docstring for call_back"""
    def __init__(self):
        super(call_back, self).__init__()
        self.robot = Dorna()
        self.robot.register_callback(self.alarm_condition)
        

    async def alarm_condition(self, msg, sys):
        # alarm condition
        if "in0" in msg and msg["in0"] == 1:
            # de register callback
            self.robot.deregister_callback()

            # sleep for 1 seconds
            await asyncio.sleep(1)

            # activate the alarm
            self.robot.log("activating alarm")
            self.robot.alarm(1, timeout=0)
        return 0

def main(robot_cb):
    while True:
        # run the script
        result = robot_cb.robot.play_script("script.txt")
        
        # The robot is in alarm mode
        if result < 0:
            break

    # close the connection at the end    
if __name__ == '__main__':
    # ip
    ip = "localhost"

    # create the object
    cb = call_back()
    
    # connect
    cb.robot.connect(ip)
    
    # main function
    main(cb)

    # close
    cb.robot.close()
```
Consider the following points in this example:
- We defined `alarm_condition` with arguments `msg` and `sys`.
- If `in0` is high then we first `deregister_callback` to avoid calling `alarm_condition` multiple times.
- Notice how we use `await asyncio.sleep(1)` instead of `time.sleep(1)`. Because `alarm_condition` is an `async` function and any blocking method like `sleep` should be awaited.
- For the same reason we used `timeout=0` in the `.alarm()` method to make it a non-blocking function. 

## Example
To learn more about the API, navigate to the following link for more examples:  
https://github.com/dorna-robotics/library_examples

[dorna]: https://dorna.ai
[json]: https://www.json.org
