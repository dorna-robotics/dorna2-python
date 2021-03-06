# Dorna2 
This is the Python API repository for [Dorna 2][dorna] robotic arm.   

## Installation
Notice that the program has been tested only on Python 3.7+.

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

### Getting started
Import `dorna2` module.
``` python
from dorna2 import dorna

ip = "127.0.0.1"
port = 443

robot = dorna()  # create the dorna object
robot.connect(ip, host)  # connect to the robot

# your code goes here

robot.close() # always close the socket when you are done
```  

### Document
For the full documentation please visit the [Dorna API documentation page](https://doc.dorna.ai/docs/api/python/manual).

### History
- 1.24
    - Adding multi threading feature and increased the WS speed significantly.
    - Bug fixed: The API and dorna lab can both connect to the robot at the same time.
    - Added [tracking command](https://doc.dorna.ai/docs/api/python/manual/#tracking-command) feature, for tracking the status of a command.
    - Added forward and inverse kinematics of the robot inside the `tool.py`.

[dorna]: https://dorna.ai