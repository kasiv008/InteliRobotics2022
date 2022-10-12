from controller import Robot, Motor, DistanceSensor, PositionSensor, Supervisor
import math

TIME_STEP = 100

def line(x1,y1,x2,y2,x = None):
    slope = (y2-y1)/(x2-x1)
    c = y1 - slope*x1
    return slope,c
    
def euc_distance(x1,y1,x2,y2):
    distance = math.sqrt((x2-x1)**2+(y2-y1)**2)
    return distance
    
def theta(xg,yg,xp,yp):
    return math.atan((yg-yp)/(xg-xp))
    
#def attract_vec(pos,ori,goal):
    
    
# create the Robot instance.
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
goal = (0.7,0.5)
r_vel = 1
l_vel = 1
#parameters
alpha = 3
scale = 0.3
min_bound = 0.2
goal_tresh = 0.1
obs_tresh = 0.1
beta = 4.5

timestep = int(robot.getBasicTimeStep())
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)
obs_pos = [(0.667,0.661),(0.34,0.19),(0.681,0.299),(0.048,0.504)]
# ps = []
# psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
# for i in range(8):
    # ps.append(robot.getDevice(psNames[i]))
    # ps[i].enable(timestep)
# set the target position of the motors
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
rob = robot.getFromDef('e_puck')

while robot.step(timestep ) != -1:
    pos = rob.getPosition()
    ori = rob.getOrientation()
    dg = euc_distance(pos[1],pos[0],0.6,0.6)
    theta_g = theta(goal[0],goal[1],pos[1],pos[0])
    #print(dg,pos[0],pos[1])
    dx = 0
    dy = 0
    dox = 0
    doy = 0
    if dg <= goal_tresh:
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        break
    else:
        dxg = alpha*math.cos(theta_g)
        dyg = alpha*math.sin(theta_g)
    #elif dg < min_bound + goal_tresh:
     #   dxg = alpha*(dg-goal_tresh)*math.cos(theta_g)
    #    dyg = alpha*(dg-goal_tresh)*math.sin(theta_g)
    for i in obs_pos:
        do = euc_distance(pos[1],pos[0],i[0],i[1])
        theta_o = theta(i[0],i[1],pos[1],pos[0])
        #print(do , theta_o)
        if do < obs_tresh:
            dxi = -math.cos(theta_o)*5
            dyi = -math.sin(theta_o)*5
        elif do < scale + obs_tresh:
            dxi = -beta*(scale+obs_tresh-do)*math.cos(theta_o)
            dyi = -beta*(scale+obs_tresh-do)*math.sin(theta_o)
        elif do > scale+obs_tresh:
            dxi = 0
            dyi = 0
        dox = dox + dxi
        doy = doy + dyi
    dx = dxg + dox
    dy = dyg + doy
    #print(dx,dy)
    theta_t = math.atan(dy/dx)
    #net_x = pos[0] + dx
    #net_y = pos[1] + dy
    ang = math.acos(ori[0])
    # #net_x = 0.05
    # #net_y = 0.1
    # theta_t = theta(net_x,net_y,pos[0],pos[1])
    # print(net_x,net_y)
    # #print(theta_t-ang)
    if abs(theta_t) - abs(ang) > 0.1:
       leftMotor.setVelocity(-0.1)
       rightMotor.setVelocity(+0.1)
       continue
    # if dox == 0 and doy == 0 and abs(theta_g) - abs(ang) > 0.1:
        # leftMotor.setVelocity(-0.1)
        # rightMotor.setVelocity(+0.1)
        # continue
    # elif abs(theta_t - ang) > 3:
        # leftMotor.setVelocity(+0.2)
        # rightMotor.setVelocity(-0.2)
        # continue
    #print(dox,doy)
    leftMotor.setVelocity(0.3)
    rightMotor.setVelocity(0.3)
    pass
