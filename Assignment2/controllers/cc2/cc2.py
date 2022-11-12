import math
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

def intersect_point_line(p,l1,l2):
    s1 = (l2[1]-l1[1])/(l2[0]-l1[0])
    s2 = -1/s1
    
    c2 = p[1]-s2*p[0]
    c1 = l2[1] - s1*l2[0]
    
    x = (c2-c1)/(s1-s2)
    y = (s1*c2 - s2*c1)/(s1-s2)
    return(x,y) 
    
    
def run_robot(robot, utils, goal, sci, delta, k, u_max, v, dt):
    time_step = 32
    max_speed = 6.24
    is_obstacle = False
    epuck = robot.getFromDef('e_puck_p1')
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

        theta = math.atan2(goal[1]-start[1], goal[0]-start[0])      
        d = intersect_point_line((pos[0],pos[1]), start, goal)
        
        xvt = d[0] + delta*math.cos(theta)
        yvt = d[1] + delta*math.sin(theta)
        
        theta_d = math.atan2(yvt-pos[1], xvt-pos[0])   
        
        
        e = (theta_d - sci)
        
            
        front_obs = ps[0].getValue()
        right_obs = ps[2].getValue()
        right_corner = ps[1].getValue()
        
        if  front_obs > 80:
            is_obstacle = True 
            
 
        if not is_obstacle:
            if heading_angle - desire_angle < 0.01:
            
                u = k* e
                if u > u_max:
                    u = u_max
                elif u < -u_max:
                    u = -u_max
        
                sci = sci + u *dt
                
                
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
            if front_obs > 80:
                left_speed = -max_speed 
                right_speed = max_speed            
            else:    
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
            if utils.perpendicular_dis(pos, utils.line_bw_points(start, pos)) < 0.01 and (start[1] < pos[1] < goal[1]):                  
                is_obstacle = False
                m_point = pos
        if utils.distance_bw_points(pos, goal) < 0.07:
            left_speed = max_speed *0
            right_speed = max_speed *0
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)

goal = (-1,0.5)
sci = 0.5
delta = 0.01
k = 0.1
u_max = 0.4
v = 0.1
dt = 0.01
robot = Supervisor()
utils = utils()
run_robot(robot, utils, goal, sci,delta,k,u_max,v,dt)
