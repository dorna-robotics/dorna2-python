from dorna2 import dorna
import sys
sys.path.append('..')


def main(ip, port):
    robot = dorna()
    robot.connect(ip, port)

    for cmd in 100 * ["alarm", "toollength", "input", "output", "pwm", "adc"]:
        arg = {"cmd": cmd, "id": robot.rand_id()}
        print(arg)
        robot.play(**arg)
        robot.wait(arg["id"])

    robot.close()

if __name__ == '__main__':
    main("192.168.1.7", 443)
