# Dorna2 
This is the Python API repository for [Dorna 2][dorna] robotic arm.   

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
python setup.py install --force
```

### Getting started
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

### Document
For the full documentation please visit the [Dorna API documentation page](https://doc.dorna.ai/docs/api/python/manual).

[dorna]: https://dorna.ai