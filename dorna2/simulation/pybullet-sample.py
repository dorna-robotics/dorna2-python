import numpy as np 

import time 


from simulation import check_path, create_cube, create_sphere

import pybullet as p
import pybullet_data 
# PyBullet setup 
p.connect(p.GUI) 
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 
p.setGravity(0,0,-9.81)

scene = [] 
for i in range(4):
    t = i * np.pi / 2 + np.pi/4
    l = 0.48
    tf = [l*np.cos(t),l*np.sin(t),0,0,0,0]

    scene.append(create_cube(tf, [0.05,0.05,0.8]))

tool = [create_sphere([0,0,0,0,0,0], [0.02,0.02,0.3])]


res = check_path(motion="lmove", start_joint=[-90,0,0,0,0,0,0], end_joint=[0,0,0,0,0,0,0], scene=scene, tool=tool, tcp=[0,0,0,0,0,0], steps=50, early_exit=True)
res["debug"]["sim"].preview_animation(res["debug"]["path"], res["debug"]["visuals"])
print(res)
