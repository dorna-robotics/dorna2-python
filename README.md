# Dorna2 
This is Python API for [Dorna 2][dorna] robotic arm.

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
To start the software, first create a `Gui` object, and then call the `run()` method.
``` python
from dorna2 import dorna

robot = dorna()

# use the ws url to connect to the robot
robot.connect("ws://dorna:443")

# sample jmove command
robot.jmove(j0 = 0, j1 = 0, j2 = 0, j3 = 0, j4 = 0) 
```  
  
[dorna]: https://dorna.ai/
