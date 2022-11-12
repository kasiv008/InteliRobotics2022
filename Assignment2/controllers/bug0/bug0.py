from controller import Robot, Supervisor
from utils import utils

def enable_proximity_sensors(robot, time_step):
    ps = []
    psNames = [
        'ps0', 'ps1', 'ps2', 'ps3',
        'ps4', 'ps5', 'ps6', 'ps7'
         ]
    for i in range(8):
        ps.append(robot.getDevice(psNames[i]))
        ps[i].enable(time_step)
        
    return ps

    
    
    
def run_robot(robot, utils, goal):
    time_step = 32
    max_speed = 6.24
    is_obstacle = False

    epuck = robot.getFromDef('e_puck')
    start = epuck.getPosition()[:2]
    m_point = start
    

    left_motor = robot.getMotor('left wheel motor')
    right_motor = robot.getMotor('right wheel motor')

    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    ps = enable_proximity_sensors(robot, time_step)
    

    
    while robot.step(time_step) != -1:
        
        pos = epuck.getPosition()[:2]
        
        heading_angle = utils.get_heading_angle(epuck.getOrientation())
        desire_angle = utils.angel_line_horizontal((pos[0],pos[1]),goal)

        front_obs = ps[0].getValue()
        right_obs = ps[2].getValue()
        right_corner = ps[1].getValue()
        
        if  front_obs > 80:
            is_obstacle = True 

        if not is_obstacle:
            if heading_angle - desire_angle < 0.01:
                left_speed = -max_speed *0.4
                right_speed = max_speed *0.4
                left_motor.setVelocity(left_speed)
                right_motor.setVelocity(right_speed)
                
            else:
                left_speed = max_speed *0.5
                right_speed = max_speed *0.5
                left_motor.setVelocity(left_speed)
                right_motor.setVelocity(right_speed)
                
        elif is_obstacle:
            # turn left
            if front_obs > 80:
                left_speed = -max_speed 
                right_speed = max_speed            
                # print('turn left')
            else:    
                # move forward
                if right_obs > 80 :
                    left_speed = max_speed *0.5
                    right_speed = max_speed *0.5
                else:
                    left_speed = max_speed 
                    right_speed = max_speed *0.125
                if right_corner > 80:
                    left_speed = max_speed *0.125
                    right_speed = max_speed                     
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)
            if abs(desire_angle - heading_angle) < 4 and front_obs < 80:
                is_obstacle = False
            



goal = (1.9,0.5)
robot = Supervisor()
utils = utils()
run_robot(robot, utils, goal)
