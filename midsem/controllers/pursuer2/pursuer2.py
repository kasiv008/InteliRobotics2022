from controller import Robot, Motor, DistanceSensor, PositionSensor, Supervisor
import math
import numpy as np


def line(x1,y1,x2,y2):
    slope = (y2-y1)/(x2-x1)
    c = y1 - slope*x1
    return slope,c
    
def checkcollision(a, b, c, x, y, radius):
     
    dist = ((abs(a * x + b * y + c)) /
            math.sqrt(a * a + b * b))
            
    if radius >= dist:
        return 1
    else:
        return 0
    
pursuer = Supervisor()
#evader = Robot()
timestep = int(pursuer.getBasicTimeStep())
pleft = pursuer.getDevice('left wheel motor')
pright = pursuer.getDevice('right wheel motor')
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
lidar = pursuer.getDevice('LDS-01')
lidar.enable(timestep)
lidar.enablePointCloud()
motor = pursuer.getDevice("LDS-01_main_motor")
smotor = pursuer.getDevice("LDS-01_secondary_motor")
motor.setPosition(float('inf'))
smotor.setPosition(float('inf'))
motor.setVelocity(30)
smotor.setVelocity(60)
obs_pos = [(1.68,1.79),(0.613,2.11),(1.31,0.74),(2.17,2.62),(2.59,1.11)]
#range = lidar.getMaxRange()
rob = pursuer.getFromDef('pursuer_2')
eob = pursuer.getFromDef('evader')
nrays = lidar.getHorizontalResolution()
while pursuer.step(timestep ) != -1:
    pos = rob.getPosition()
    pos_ev = eob.getPosition()
    #print(pos_ev)
    ori = rob.getOrientation()
    x_ori = math.acos(ori[0])
    #y_ori = math.acos(ori[3])
    range_image = lidar.getRangeImage()
    #print(y_ori)
    #print(x_ori)
    #print(range_image)
    #print(len(range_image))
    grid = np.zeros((7,7))
    for i in range(len(range_image)):
        x = range_image[i]*math.cos((i * math.pi/180)-x_ori)
        y = range_image[i]*math.sin((i * math.pi/180)-x_ori)
        #print(x,y)
        net_x = pos[0] +x
        net_y = pos[1] +y
        #print(net_x,net_y) 
        #print(pos[0])
        net_x = 3 if net_x>3 else net_x
        net_y = 3 if net_y >3 else net_y
        net_x = 0 if net_x<0 else net_x
        net_y = 0 if net_y <0 else net_y
        #print(net_x,net_y)
        grid[int(net_x*2)][int(net_y*2)] = grid[int(net_x*2)][int(net_y*2)] + 1
        #grid = np.fliplr(grid)
    if min(range_image[:45]) <= 0.2:
        pleft.setVelocity(-0.5)
        pright.setVelocity(0.5)
    elif min(range_image[-45:]) <= 0.2:
        pleft.setVelocity(0.5)
        pright.setVelocity(-0.5)
    else: 
        pleft.setVelocity(1)
        pright.setVelocity(1)
    slope , c = line(pos[0],pos[1],pos_ev[0],pos_ev[1])
    res = []
    for i in obs_pos:
        radius = 0.3
        lol = checkcollision(slope,-1,c,i[0],i[1],radius)
        res.append(lol)
    if max(res)==0:
        print('evader caught')
        pleft.setVelocity(0)
        pright.setVelocity(0)
        break
        #if i > 200:
            #break
    
    pass
#psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
#for i in range(8):
#    ps.append(robot.getDevice(psNames[i]))
 #   ps[i].enable(timestep)