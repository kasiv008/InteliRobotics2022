from controller import Robot, Motor, DistanceSensor, PositionSensor, Supervisor
import math
import numpy as np
import matplotlib.pyplot as plt


def line(pos,ori):
    slope = math.tan(math.acos(ori[0]))
    c = pos[1] - slope*pos[0]
    return (slope,c)
    
def line2(x1,y1,x2,y2):
    slope = (y2-y1)/(x2-x1)
    c = y2 - slope*x2
    return (slope,c)
    
def intersect(s1,c1,s2,c2):
    if s1-s2 == 0:
        return False
    else:
        x = (c2-c1)/(s1-s2)
        y = (s1*c2 - s2*c1)/(s1-s2)
        return (x,y)
        
def euc_dist(x1,y1,x2,y2):
    return math.sqrt((x2-x1)**2 + (y2-y1)**2)
    
def intersect_time(posx,posy,x,y,speed):
    dist = euc_dist(posx,posy,x,y)
    time = dist/speed
    #print(dist)
    return dist,time
    
robot = Supervisor()
#evader = Robot()
timestep = int(robot.getBasicTimeStep())
pleft = robot.getDevice('left wheel motor')
pright = robot.getDevice('right wheel motor')
pleft.setPosition(float('inf'))
pright.setPosition(float('inf'))
l_vel = 3
r_vel = 3
pleft.setVelocity(0)
pright.setVelocity(0)
lidar = robot.getDevice('LDS-01')
lidar.enable(timestep)
lidar.enablePointCloud()
motor = robot.getDevice("LDS-01_main_motor")
smotor = robot.getDevice("LDS-01_secondary_motor")
motor.setPosition(float('inf'))
smotor.setPosition(float('inf'))
motor.setVelocity(30)
smotor.setVelocity(60)


#Defining DEF names for the robots
rob = robot.getFromDef('robot')
eob1 = robot.getFromDef('rumba1')
eob2 = robot.getFromDef('rumba2')

nrays = lidar.getHorizontalResolution()
start = [-0.713,-1.12]
goal = (0.7,-1.23)
way_pnts = [(-0.22,1.39),(0.28,1.38)]
n = 0
pos_r1 = []
pos_r2 = []
pos_rob = []
while robot.step(timestep ) != -1:
    pos = rob.getPosition()
    pos_rumba1 = eob1.getPosition()
    pos_rumba2 = eob2.getPosition()
    #print(pos_ev)
    ori = rob.getOrientation()
    ori_rumba1 = eob1.getOrientation()
    ori_rumba2 = eob2.getOrientation()
    
    range_image = lidar.getRangeImage()
    
    rob_line = line(pos,ori)
    rumba1_line = line(pos_rumba1,ori_rumba1)
    rumba2_line = line(pos_rumba2,ori_rumba2)
    
    int1 = intersect(rumba1_line[0],rumba1_line[1],rob_line[0],rob_line[1])
    int2 = intersect(rumba2_line[0],rumba2_line[1],rob_line[0],rob_line[1])
    
    pos_r1.append((pos_rumba1[0],pos_rumba1[1]))
    pos_r2.append((pos_rumba2[0],pos_rumba2[1]))
    pos_rob.append((pos[0],pos[1]))
    #print(int1)
    if pos[0] <= 0 and n == 0:
        point = way_pnts[0]
        
    if math.sqrt((point[0]- pos[0])**2 + (point[1]- pos[1])**2) <= 0.1 and n == 2:
        pleft.setVelocity(0)
        pright.setVelocity(0)
        print('Goal Reached')
        break
        
    if pos[0]<= 0:
        robd,robt = intersect_time(pos[0],pos[1],int1[0],int1[1],6.67)
        rumbad,rumbat = intersect_time(pos_rumba1[0],pos_rumba1[1],int1[0],int1[1],16)
        #print(abs(rumbad + robd))
        if euc_dist(pos[0],pos[1],pos_rumba1[0],pos_rumba1[1])<0.65:
             if pos[1] < int1[1]:
                 if pos[1] > 1.3:
                     pleft.setVelocity(0)
                     pright.setVelocity(0)
                     continue
                 else:
                     pleft.setVelocity(-6.67)
                     pright.setVelocity(-6.67)
                     print('rumba in front')
                     continue
             else:
                 if pos[1] > 1.3:
                     pleft.setVelocity(0)
                     pright.setVelocity(0)
                     continue
                 else:
                     pleft.setVelocity(6.67)
                     pright.setVelocity(6.67)
                     print('rumba behind')
                     continue
    else:
        robd,robt = intersect_time(pos[0],pos[1],int2[0],int2[1],6.67)
        robd,rumbat = intersect_time(pos_rumba2[0],pos_rumba2[1],int2[0],int2[1],16)
        if euc_dist(pos[0],pos[1],pos_rumba2[0],pos_rumba2[1])<0.65:
             if pos[1] < int2[1]:
                 if pos[1] > 1.3:
                     pleft.setVelocity(0)
                     pright.setVelocity(0)
                     continue
                 else:
                     pleft.setVelocity(6.67)
                     pright.setVelocity(6.67)
                     print('rumba behind')
                     continue
             else:
                 if pos[1] > 1.3:
                     pleft.setVelocity(0)
                     pright.setVelocity(0)
                     continue
                 else:
                     pleft.setVelocity(-6.67)
                     pright.setVelocity(-6.67)
                     print('rumba in front')
                     continue
        
    des_heading = (point[0]- pos[0])/math.sqrt((point[0]- pos[0])**2 + (point[1]- pos[1])**2)
    cur_heading = ori[0]
    #print(des_heading, cur_heading)
    if round(des_heading,2) != round(cur_heading,2):
        if des_heading > cur_heading and n <2:
            pleft.setVelocity(0.8)
            pright.setVelocity(-0.8)
            print('turning right')
        elif des_heading < cur_heading and n <2: 
            pleft.setVelocity(-0.8)
            pright.setVelocity(0.8)
            print('turning left')
        elif des_heading > cur_heading and n == 2:
            pleft.setVelocity(-0.8)
            pright.setVelocity(0.8)
            print('turning left')
        elif des_heading < cur_heading and n == 2:
            pleft.setVelocity(0.8)
            pright.setVelocity(-0.8)
            print('turning right')
    else:
        print('moving forward')
        pleft.setVelocity(6.67)
        pright.setVelocity(6.67)
    if math.sqrt((point[0]- pos[0])**2 + (point[1]- pos[1])**2) <= 0.1 and n == 0:
        point = way_pnts[1]
        n = 1
    if math.sqrt((point[0]- pos[0])**2 + (point[1]- pos[1])**2) <= 0.1 and n == 1:
        point = goal
        n = 2
    pass
#print(pos_rob[0])
plt.plot(*zip(*pos_rob),color = 'blue')
plt.plot(*zip(*pos_r1),color = 'orange')
plt.plot(*zip(*pos_r2),color = 'orange')
plt.scatter(start[0],start[1],color = 'green',linewidths=3)
plt.scatter(goal[0],goal[1],color = 'red',linewidths=3)
plt.title("Patch tracing")
plt.show()
