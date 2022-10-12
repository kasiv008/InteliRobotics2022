from controller import Robot, Motor, DistanceSensor, PositionSensor, Supervisor
import math
import numpy as np

    

evader = Robot()
timestep = int(evader.getBasicTimeStep())
pleft = evader.getDevice('left wheel motor')
pright = evader.getDevice('right wheel motor')
pleft.setPosition(float('inf'))
pright.setPosition(float('inf'))
#eleft = evader.getDevice('left wheel motor')
#eright = evader.getDevice('right wheel motor')
l_vel = 3
r_vel = 3
pleft.setVelocity(0)
pright.setVelocity(0)
#eleft.setVelocity(0.0)
#eright.setVelocity(0.0)
#ps = []
lidar = evader.getDevice('LDS-01')
lidar.enable(timestep)
lidar.enablePointCloud()
motor = evader.getDevice("LDS-01_main_motor")
smotor = evader.getDevice("LDS-01_secondary_motor")
motor.setPosition(float('inf'))
smotor.setPosition(float('inf'))
motor.setVelocity(30)
smotor.setVelocity(60)
#obs_pos = [(1.19,2.13),(0.81,1.143),(1.66,0.754),(1.96,1.92)]
#range = lidar.getMaxRange()
#rob = pursuer.getFromDef('pursuer_1')
#eob = pursuer.getFromDef('evader')
nrays = lidar.getHorizontalResolution()
while evader.step(timestep ) != -1:
    #pos = rob.getPosition()
    range_image = lidar.getRangeImage()
    if min(range_image[:45]) <= 0.2:
        pleft.setVelocity(0.3)
        pright.setVelocity(-0.3)
    elif min(range_image[-45:]) <= 0.2:
        pleft.setVelocity(-0.3)
        pright.setVelocity(0.3)
    else: 
        pleft.setVelocity(1.5)
        pright.setVelocity(1.5)
    pass