from dorna2 import dorna
import config

if __name__ == '__main__':
    ip = "localhost"
    joint = "j5"

    # create robot object
    robot = dorna()
    robot.connect(ip)

    robot.home(joint, **config.config[joint])

    robot.close()