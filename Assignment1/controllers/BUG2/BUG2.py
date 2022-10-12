from controller import Robot, Motor, DistanceSensor, PositionSensor, Supervisor
import math

TIME_STEP = 50

def line(x1,y1,x2,y2,x = None):
    slope = (y2-y1)/(x2-x1)
    c = y1 - slope*x1
    return slope,c
    
def euc_distance(x1,y1,x2,y2):
    distance = math.sqrt((x2-x1)**2+(y2-y1)**2)
    return distance
    
# create the Robot instance.
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
goal = (-0.45,0.45)
r_vel = 1
l_vel = 1
start = (0.419,-0.409)
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)
# set the target position of the motors
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
rob = robot.getFromDef('e_puck')

slope_g,c_g = line(start[0],start[1],goal[0],goal[1])
angle = math.atan(slope_g)
sin_c = round(math.sin(angle),1)
cos_c = -round(math.cos(angle),1)
while robot.step(timestep ) != -1:
    pos = rob.getPosition()
    ori = rob.getOrientation()
    dis = euc_distance(pos[0],pos[1],goal[0],goal[1])
    
    #slope,c = line(pos[0],pos[1],goal[0],goal[1])
    #print(ori)
    #print(angle)
    if dis <= 0.2:
        print('goal reached')
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0) 
        break
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
    #print(psValues,round(ori[0],1),cos_c,round(ori[4],1),sin_c)
    if max(psValues)<=90 and round(ori[0],1) != cos_c or round(ori[4],1) != sin_c:
        leftMotor.setVelocity(l_vel)
        rightMotor.setVelocity(-r_vel)
        continue  
        
    if psValues[0]>=115 or psValues[7]>=100 or psValues[1]>=100:
        mad = True
        leftMotor.setVelocity(-l_vel)
        rightMotor.setVelocity(r_vel)
        while robot.step(mad):
            pos = rob.getPosition()
            ori = rob.getOrientation()
            psValues = []
            for i in range(8):
                psValues.append(ps[i].getValue())
            slope,c = line(pos[0],pos[1],goal[0],goal[1])
            #print(psValues)
            if round(slope,2) != round(slope_g,2) or round(c,2) != round(c_g,2):
                #print('s')
                if psValues[0]>=115 or psValues[7]>=100 or psValues[1]>=100:
                    leftMotor.setVelocity(-l_vel)
                    rightMotor.setVelocity(r_vel)
                elif psValues[3]<=70 and psValues[2]<=70 and psValues[0] <= 70 and psValues[1]<=70:
                    leftMotor.setVelocity(l_vel)
                    rightMotor.setVelocity(-r_vel)
                else:
                    leftMotor.setVelocity(l_vel)
                    rightMotor.setVelocity(r_vel)
            else:
                (round(slope,2), round(slope_g,2),round(c,2),round(c_g,2))
                if max(psValues)<=65 and round(ori[0],1) != cos_c or round(ori[4],1) != sin_c:
                    leftMotor.setVelocity(-l_vel)
                    rightMotor.setVelocity(r_vel)
                else:
                    #print('lol')
                    mad = False
            # else :
            # lol = False
    leftMotor.setVelocity(l_vel)
    rightMotor.setVelocity(r_vel)  
    #leftMotor.setVelocity(l_vel)
    #rightMotor.setVelocity(r_vel)
    pass