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
    
# create the Robot instance.
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
goal = (0,0)
r_vel = 2
l_vel = 2
obs_tresh = 120
timestep = int(robot.getBasicTimeStep())
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

while robot.step(timestep ) != -1:
    pos = rob.getPosition()
    ori = rob.getOrientation()
    dis = euc_distance(pos[0],pos[1],goal[0],goal[1])
    slope,c = line(pos[0],pos[1],goal[0],goal[1])
    angle = math.atan(slope)
    #print(angle)
    if dis <= 0.01:
        print('goal reached')
        leftMotor.setVelocity(0.0)
        rightMotor.setVelocity(0.0)
        break
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
    if psValues[0]>=100 or psValues[7]>=100 or psValues[1]>=90 :
        leftMotor.setVelocity(-l_vel)
        rightMotor.setVelocity(r_vel)
        continue
    #leftMotor.setVelocity(l_vel)
    #rightMotor.setVelocity(r_vel)
    #print(psValues,round(abs(angle),1))
    if max(psValues)<=72.5 and round(abs(angle),1) != 1.6:
        #print('f')
        leftMotor.setVelocity(l_vel)
        rightMotor.setVelocity(-r_vel)
        continue    
    leftMotor.setVelocity(l_vel)
    rightMotor.setVelocity(r_vel)
    pass
