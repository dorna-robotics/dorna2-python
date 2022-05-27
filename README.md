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

### Getting Started
First, import `Dorna` class from the `dorna2` module, and then create a `Dorna` object.
``` python
from dorna2 import Dorna

# create the Dorna object
robot = Dorna()
```  

## Connection
The robot websocket server runs on `ws://robot_host_address:443`, where `robot_host_address` is the host address (IP) of the robot controller, and `443` is the port number. Once the connection has been established between the robot and the client (user), they start communicating with each other by sending and receiving data in [JSON][json] format. 

### `.connect(host="localhost", port=443, handshake_timeout=5)` 
Connect to the robot controller server at `ws://host:port`, and returns `True` on a successful connection, otherwise `False`.

#### Parameter
- *host*: (string) The controller host address. The default value is `"localhost"`.
- *port*: (int) The controller port number. The default value is `443`.
- *handshake_timeout*: (float > 0) Wait maximum of `handshake_timeout` seconds to establish a connection to the robot controller. The default value is `5` seconds.

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

## Send and Receive Data
Once you are  connected to the robot controller, you can start sending valid commands to the robot, and receive messages from it.   

### Command Status and Execution
We need to first get familiar with the status of a command. When sending a valid command to the robot, the controller reports the status of the command from the time that the command is submitted, to the time that the execution of that command is completed using a `stat` key and the unique `id` send initially with the command. An example of such a message is as follows:
``` python
# send a command to read the alarm status of the system
{"cmd":"alarm","id":10}

# receive the following messages one by one from the controller
{"id":10,"stat":0}
{"id":10,"stat":1}
{"cmd":"alarm","id":10,"alarm":0}
{"id":10,"stat":2}
```
#### ID Key
As you can see in the above example, we sent a command to the robot controller to get the alarm status of the robot. 
The robot replies back by sending multiple messages. Our initial command had an `id` field equal to `10`. So, any messages from the robot related to this command has the same `id` value.  

#### Stat Key
Another important key in the replies is the `stat` key. The `stat` field can take different values with the following interpretations:
- `stat = 0`: The command has been received by the controller with no error.
- `stat = 1`: The command execution has begun.
- `stat = 2`: The execution of the command is now completed, with no error.
- `stat < 0`: An error happened during the execution of the command and it will not be executed.

> Notice, that we say that a command is completed if get `stat = 2` or `stat < 0` of that command.

### `timeout` variable
Throughout this document we use and refer to the `timeout` key as an arguments inside methods that are sending command to the robot.  
Functions that are sending command to the robot are using the `timeout` argument for tracking the completion or any error during the execution of the command that they are sending. 
- `timeout < 0`: Send a command and wait for the command to get done ( `stat = 2` or `stat < 0`) and then return. At this moment we are sure that the command is no longer running.
- `timeout >= 0`: Send a command and wait for A maximum of `timeout` seconds for its execution. Notice that in this case, we might have returned from the function but the command which was sent by the fuction to the robot is still runnning or wating inside the controller for its time to run. If we do not want to wait for the execution of a command at all we can always set `timeout = 0`.

### `.play(timeout=-1, msg=None, **kwargs):`  
Send a message to the robot, and return `.track()`.  
There are multiple ways to send a message via `.play()`. For a better understanding, we send a simple `alarm` status command in three different ways:
- **Case 1:** (*Recommended*) Key and value format: `play(cmd='alarm', id=10})`
- **Case 2:** Python dictionary format: `play({'cmd': 'alarm', 'id': 10})` 
- **Case 3:** JSON string format: `play('{"cmd": "alarm", "id": 10}')`

#### Parameter
- *timeout*: (float) Throughout this document we use and refer to the `timeout` key as an arguments inside methods that are sending command to the robot. Use the `timeout` argument for tracking the completion or any error during the execution of the command.
    - `timeout < 0`: Send a command and wait for the command completion ( `stat = 2` or `stat < 0`) and then exit the function and  return. At this moment we are sure that the command is no longer running.
    - `timeout >= 0`: Send a command and wait for A maximum of `timeout` seconds for its completion. Notice that in this case, we might have returned from the function but the command is still running or waiting inside the controller queue for its time to run. If we do not want to wait for the execution of a command at all we can always set `timeout = 0`.

- *msg*: (Python dictionary or JSON string) Use this parameter if you want to send your command in a Python dictionary format (**Case 2**), or in a JSON format (**Case 3**).
- *kwargs*: Use this to send your command in a key and value format (**Case 1**).

> Notice that the `.play()` method always include a random `id` filed to your command if it is not present.

#### Usage
For better understanding of the `timeout` parameter, we send a joint move command to the robot.
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


### `play_script(script_path, timeout=0)`
Send all the messages that are stored in a script file to the robot controller. The method opens the script file located at `script_path`, read the file line by line and send each line as a command.  
The function returns the number of commands which was sent to the robot controller.  
> Use this function to send multiple messages at once to the robot.
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
#### Parameter
- *script_path*: (string) The path to the script file. 
- *timeout*: (float) Similar to the `timeout` parameter of the `.play()` method. The default value is `0`, which means that the function just sends the commands in the script file and return.

## Move
In this section we cover functions which are used for the robot motion. 

### `.jmove(**kwargs)`
A helper function to send a joint move (`jmove`) command, and return a `.track()` object associated to the `jmove` command sent.  
This method is basically similar to the [`.play()`](#???) method but the `cmd` key is set to `"jmove"`. So, `jmove(rel=1, j0=10, id=10)` is equivalent to `play(cmd='jmove', rel=1, j0=10, id=10)`.

### `.lmove(**kwargs)`
A helper function to send a line move (`lmove`) command, and return a `.track()` object associated to the `lmove` command sent.  
This method is basically similar to the [`.play()`](#???) method but the `cmd` key is set to `"lmove"`. So, `lmove(rel=1, x=10, id=10)` is equivalent to `play(cmd='lmove', rel=1, x=10, id=10)`.

### `.cmove(**kwargs)`
A helper function to send a circle move (`cmove`) command, and return a `.track()` object associated to the `cmove` command sent.  
This method is basically similar to the [`.play()`](#???) method but the `cmd` key is set to `"cmove"`.

## Stop
Series of helper function to send halt, read and set the alarm.

### `.halt(accel=None, **kwargs)`
Send a halt command to the robot. with a given acceleration ratio (`accel`), and return the final status of the command (`stat`).

#### Parameter
- *accel*: (float >= 1) The acceleration ratio parameter associated to the `halt`. Larger `accel` means faster and sharper halt (stop).

#### Usage
- `.halt()`: Send a halt command to the robot. Return the status of the command. For example, `2` if the halt is completed properly, and negative number in case of any error.
- `.halt(accel)`: Similar to the `.halt()` but this time with the specified acceleration.
- `.halt(**kwargs)`: A helper function to send a `halt` command. It is similar to [`.play(cmd="halt", **kwargs)`](#???), and return `.halt()`.

``` python
robot.halt() # send a halt command to the controller
robot.halt(5) # send a halt  command with acceleration ratio equal to 5 
``` 

### `.alarm(self, val=None, **kwargs)`
Set (disable or enable) or get the alarm status.

#### Parameter
- *val*: (0 or 1) Use this parameter to set the robot alarm. 0 for disabling and 1 for enabling the alarm.

#### Usage
- `.alarm()`: Get the robot alarm status (0 for disabled and 1 for enabled).
- `.alarm(val)`: Set the alarm status of the robot to `val` (0 or 1), and return `.alarm()`.
- `.alarm(**kwargs)`: A helper function to send a `alarm` command. It is similar to [`.play(cmd="alarm", **kwargs)`](#???), and return `.halt()`.
``` python
robot.alarm() # get the alarm status of the controller
robot.alarm(0) # clear the alarm (set alarm to 0)  
``` 

## Joint and TCP
In this section we cover methods that are related to the robot orientation.

### `.joint(index=None, val=None, **kwargs)`
Set or get the value of an specific joint (or joints), and return its value (their values).

#### Parameter
- *index*: (0 <= int < 8) The index of the joint that we are interested in, to set or get its value.
- *val*: (float) The value we want to assign to the specific joint in degree.

#### Usage
- `.joint()`: Get the joint values of the robot, in a list of size 8. Where index `i` in the list is the value of joint `i` (`ji`). 
- `.joint(index)`: Get the value of the joint `index`. 
- `.joint(index, val)`: Set the value of the joint `index` to `val` and return `.joint(index)`. 
- `.joint(**kwargs)`: A helper function to send a `joint` command. It is similar to [`.play(cmd="joint", **kwargs)`](#???), and return `.joint()`.

``` python
robot.joint() # return the value of all the 8 joints of the robot
robot.joint(1) # return the value of j1
robot.joint(1, 30) # set the value of j1 to 30
robot.joint(j0=10, j2=20) # set the value of j0 to 10 and j2 to 20
``` 

### `.pose()`
Get the value of the robot toolhead (TCP) in Cartesian coordinate system (with respect to the robot base frame) in a list of size 8. Where indices 0 to 7 in this list are associated to `x`, `y`, `z`, `a`, `b`, `c`, `d` and `e`, respectively.

``` python
robot.pose() # return [x, y, z, a, b, c, d, e] 
``` 

### `.toollength(val=None, **kwargs)`
Set or get the value of the robot toollength (in the Z direction with respect to the TCP frame), and return its value.

#### Parameter
- *val*: (float >= 0) The length of the toolhead in the Z direction from the base of the robot flange to the tip of the toolhead in mm.

### Usage
- `.toollength()`: Get the robot toollength in mm.
- `.toollength(val)`: Set the robot toollength to `val` mm and return `.toollength()`.
- `.toollength(**kwargs)`: A helper function to send a `toollength` command. It is similar to [`.play(cmd="toollength", **kwargs)`](#???), and return `.toollength()`.

``` python
robot.toollength() # get the robot toollength in mm
robot.toollength(10) # set the robot toollength to 10 mm  
``` 

## I/O
In this section we cover methods that are related to the robot inputs and outputs.

### `.output(index=None, val=None, **kwargs)`
Set (enable or disable) or get the value of an output pin(s).

#### Parameter
- *index*: (0<= int < 16) The index of the output pin that we are interested in, to set or get its value.
- *val*: (binary 0 or 1) The value we want to assign to the specific output pin `index`.
- *kwargs*: ???

#### Usage
- `.output()`: Get the value of all the 16 output pins in a list of size 16. Where item `i` in the list is the value of `outi`.
- `.output(index)`: Get the value of output pin `index`.
- `.output(index, val)`: Set the value of output pin `index` to `val`, and return `.output(index)`.
- `.output(**kwargs)`: A helper function to send a `output` command. It is similar to the [`.play(cmd="output", **kwargs)`](#jointindexnone-valnone-kwargs), and return `.output()`. 

``` python
robot.output() # return the value of all the 16 outputs in a list of size 16
robot.output(0) # return the value of the out0
robot.output(0, 1) # set the value of the out0 to 1 and return its value
robot.output(out0=1, out2=0) # set the value of out0 to 1 and out2 to 0, and return the value of all the 16 outputs in a list 
``` 

### `.pwm(index=None, val=None, **kwargs)`
Set (enable or disable) or get the value of a pwm channel(s).

#### Parameter
- *index*: (0<= int < 5) The index of the pwm channel that we are interested in, to set or get its value.
- *val*: (binary 0 or 1) The value we want to assign to the specific pwm channel `index`.
- *kwargs*: ???

#### Usage
- `.pwm()`: Get the value of all the 5 pwm channels in a list of size 5. Where item `i` in the list is the value of `pwmi`.
- `.pwm(index)`: Get the value of the pwm channel `index`.
- `.pwm(index, val)`: Set the value of the pwm channel `index` to `val`, and return `.pwm(index)`.
- `.pwm(**kwargs)`: A helper function to send a `output` command. It is similar to the [`.play(cmd="output", **kwargs)`](#jointindexnone-valnone-kwargs), and return `.output()`. 

``` python
robot.pwm() # return the value of all the 5 pwms in a list of size 5
robot.pwm(0) # return the value of the pwm0
robot.pwm(0, 1) # set the value of the pwm0 to 1 and return its value
robot.pwm(pwm0=1, pwm2=0) # set the value of pwm0 to 1 and pwm2 to 0, and return .pwm() 
``` 

### `.freq(index=None, val=None, **kwargs)`
Set or get the frequency value of a pwm channel(s).

#### Parameters
- *index*: (0 <= int < 5) The index of the pwm channel that we are interested to set or get its frequency value.
- *val*: (0 <= float <= 120,000,000) The frequency value we want to assign to the pwm channel `index`.  
- *kwargs*: ??? Other key and value parameters associated to this method. Including `timeout`, `queue`, `id`, etc. 

#### Usage
- `.freq()`: Get the frequency value of all the 5 pwm channels in a list of size 5. Where item `i` in the list is the value of `freqi`.
- `.freq(index)`: Get the frequency value of the pwm channel `index`.
- `.freq(index, val)`: Set the frequency value of the pwm channel `index` to `val`, and return `.pwm(index)`.
- `.freq(**kwargs)`: ???A helper function to send a `output` command. It is similar to the [`.play(cmd="output", **kwargs)`](#jointindexnone-valnone-kwargs), and return `.output()`. 

``` python
robot.freq(0) # return the value of freq0
robot.freq(0, 10) # set the value of freq0 to 1 and return its value
robot.freq() # return the frequency value of all the 5 pwms in a list of size 5
robot.freq(freq0=10, freq2=20) # set the value of freq0 to 1 and freq2 to 0, and return the frequency value of all the 5 pwms in a list 
``` 

### `.duty(index=None, val=None, **kwargs)`
Set or get the duty cycle of a pwm channel(s). If the `index` parameter is presented then the duty cycle of the pwm channel `index` is returnred. Otherwise, the dudty cycle of all the 5 pwm channels are returned in a list of size 5, where item `i` in the list is the value of `dutyi`.

#### Parameters
- *index*: (0 <= int < 5) The index of the pwm channel that we are interested to set or get its duty cycle.
- *val*: (0<= float <=100) The value of the duty cycle we want to assign to the pwm channel `index`. 
- *kwargs*: ???Other key and value parameters associated to this method. Including `timeout`, `queue`, `id`, etc. 

#### Usage
- `.duty()`: Get the duty cycles of all the 5 pwm channels in a list of size 5. Where item `i` in the list is the value of `dutyi`.
- `.duty(index)`: Get the frequency value of the pwm channel `index`.
- `.duty(index, val)`: Set the frequency value of the pwm channel `index` to `val`, and return `.pwm(index)`.
- `.duty(**kwargs)`: ???A helper function to send a `output` command. It is similar to the [`.play(cmd="output", **kwargs)`](#jointindexnone-valnone-kwargs), and return `.output()`. 

``` python
robot.duty(0) # return the value of duty0
robot.duty(0, 10) # set the value of duty0 to 1 and return its value
robot.duty() # return the duty cycle of all the 5 pwms in a list of size 5
robot.duty(duty0=10, duty2=20) # set the duty0 to 1 and duty2 to 0, and return the duty cycles of all the 5 pwms in a list 
``` 
### `.input(index=None, **kwargs)`
Get the value of an input pin(s). If the `index` parameter is presented then the value of input pin `index` is returnred. Otherwise, the value of all the 16 input pins are returned in a list of size 16, where item `i` in the list is the value of `ini`.

#### Parameters
- *index*: (0 <= int < 16) The index of the input pin that we are interested to get its value.
- *kwargs*: ???Other key and value parameters associated to this method. Including `timeout`, `queue`, `id`, etc. 

#### Usage
- `.input()`: Get the value of all input pins in a list of size 8, where index `i` in the list is the value of `ini`.
- `.input(index)`: Get the value of input pin `index`.

``` python
robot.input(0) # return the value of in0
robot.input() # return the value of all the 16 input pins in a list of size 16
``` 

### `.adc(index=None, **kwargs)`
Get the value of an adc channel(s).
``` python
robot.adc(0) # return the value of adc0
robot.adc() # return the value of all the 5 adc channels in a list of size 5
``` 
#### Parameters
- *index*: (None or 0 <= int < 5) The index of the adc channel that we are interested to get its value.
- *kwargs*: Other key and value parameters associated to this method. Including `timeout`, `queue`, `id`, etc. 

#### Return
Returns the value of adc channel(s). If the `index` parameter is presented then the value of adc channel `index` is returnred. Otherwise, the value of all the 5 adc channels are returned in a list of size 5, where item `i` in the list is the value of `adci`.

## Wait
Wait for an input pins pattern, encoder indices, received message pattern or certain amount of time in your program.

### `.probe(index=None, val=None, **kwargs)`
Set the probe input pin and return the the value of the joints the moment that the input pin was matched to the probe pattern.
> Use this method to wait for a pattern in the input pin(s).

#### Parameter
- *index*: (0 <= int < 16) The index of the input pin that we are interested to track and wait for it to get low (0) or high (1).
- *val*: (0 or 1) The value that the input pin matches to.

## Usage
- `.probe(index, val)`: Return the joint values of the robot in a list of size 8 ([`.joint()`](#???)), the moment input pin `index` (0 <= int < 16), is equal to the `val` (0 or 1).
- `.probe(**kwargs)`: A helper function to send a `probe` command. It is similar to [`.play(cmd="probe", **kwargs)`](#???), and return [`.joint()`](#???). 

``` python
robot.probe(1, 0) # return the joint values, the moment in1 gets 0
``` 

### `.iprobe(index=None, val=None, **kwargs)`
This method is similar to the `probe` function but here we are waiting for an specific patter in the encoder indices, instead of the input pins. Similar to the `probe` function, it will return [`.joint()`](#???), the moment the matching occurs.    
> Notice that the encoder on the motors gets high (1), 8 times during one full rotation of the encoder, and we can locate these points by calling the `.iprobe` function.

#### Parameters
- *index*: (None or 0 <= int < 8) The index of the encoder that we are interested to set for iprobe.
- *val*: (None or binary) The value we want to assign to the encoder index `index` for the iprobe process. This basically means to return the joints value of the robot when encoder index `index` is equal to `val`.   
- *kwargs*: Other key and value parameters associated to this method. Including `timeout`, `queue`, `id`, etc. 

## Usage
- `.iprobe(index, val)`: Return the joint values of the robot in a list of size 8 ([`.joint()`](#???)), the moment that the encoder index `index` (0 <= int < 8), is equal to the `val` (0 or 1).
- `.iprobe(**kwargs)`: A helper function to send a `iprobe` command. It is similar to [`.play(cmd="iprobe", **kwargs)`](#???), and return [`.joint()`](#???). 
``` python
robot.iprobe(1, 1) # return the value of joints, the moment index1 (encoder 1 index) gets 1
robot.iprobe(in0=1, in3=1) # return the value of joints the moment index0 is 1 and index3 is 1 
``` 

### `.sleep(val=None, **kwargs)`
Send a `sleep` command to the controller and sleep for certain amount of time.
- `.sleep(val)`: Sleep for `val` (float >=0) seconds and return the status of the command. A successful sleep returns 2. A negative integer return means that there was an error during the excution of this command. 
- `.sleep(**kwargs)`: A helper function to send a `sleep` command. It is similar to the [`.play(cmd="sleep", **kwargs)`](#jointindexnone-valnone-kwargs), and return the final status of the executed command (2 or negative integer for an error).
``` python
robot.sleep(10) # the controller sleepsfor 10 seconds
``` 

### `.version(**kwargs)`
Get the firmware version of the controller.
``` python
robot.version() # get the firmware version
``` 

### `.uid(**kwargs)`
Get the controller ID.
``` python
robot.uid() # get the controller 
``` 

### `.motor(self, val=None, **kwargs)`
Set (disable or enable) or get the motor status.
- `.motor()`: Get the robot motor status (0 for disabled and 1 for enabled).
- `.motor(val)`: Set the value of the motors to `val` (0 or 1), and return `.motor()`.
- `.motor(**kwargs)`: A helper function to send a `motor` command. It is similar to the [`.play(cmd="motor", **kwargs)`](#jointindexnone-valnone-kwargs), and return `.motor()`.
``` python
robot.motor() # get the robot motor status
robot.motor(0) # disable the motors  
``` 

## Example
To learn more about the API, navigate to the main directory of the repository and check the `example` folder for more examples.  
https://github.com/dorna-robotics/dorna2-python/tree/master/example

[dorna]: https://dorna.ai
[json]: https://www.json.org
