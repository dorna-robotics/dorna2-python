import numpy as np
import fcl

class Node:
    def __init__(self, name, parent=None, local_transform=None):
        self.name = name
        self.parent = None
        self.children = []
        self.lock = False
        self.local_transform = local_transform if local_transform is not None else np.eye(4)

        # Geometry
        self.visuals = []  # List of mesh or shape representations (could be your own format)
        self.collisions = []  # List of fcl.CollisionObject

        if parent:
            self.set_parent(parent)

    def get_global_transform(self):
        if self.parent:
            return self.parent.get_global_transform() @ self.local_transform
        return self.local_transform

    def set_local_transform(self, transform):
        self.local_transform = transform

    def set_parent(self, new_parent):
        if self.parent:
            self.parent.children.remove(self)
        self.parent = new_parent
        new_parent.children.append(self)

    def move_to(self, new_parent):
        """Reparents node while keeping global transform unchanged."""
        
        if self.lock: #locked nodes such as robot nodes, wont move around in tree
        	return

        global_tf = self.get_global_transform()
        inv_new_parent_global = np.linalg.inv(new_parent.get_global_transform())
        new_local = inv_new_parent_global @ global_tf
        self.set_parent(new_parent)
        self.local_transform = new_local
