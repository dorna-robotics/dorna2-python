import numpy as np
from node import Node

class Simulation:
    def __init__(self, name):
        self.root_node = Node("root")
        self.init_robot()


    def init_robot(self, root):
        