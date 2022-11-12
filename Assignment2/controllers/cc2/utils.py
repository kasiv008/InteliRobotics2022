import numpy as np
from shapely.geometry import LineString

class utils():
    
    def get_heading_angle(self,R):
        theta_z = np.arctan2(R[3], R[0])
        angle = np.degrees(theta_z)
        if angle < 0:
            angle = 360 + angle
        return angle

    def angel_line_horizontal(self,p1, p2):
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        theta = np.arctan2(dy, dx)
        angle = np.degrees(theta)  # angle is in (-180, 180]
        if angle < 0:
            angle = 360 + angle
        return angle
        
    def get_heading_angle_180(self,R):
        theta_z = np.arctan2(R[3], R[0])
        angle = np.degrees(theta_z)
        return angle

    def angel_line_horizontal_180(self,p1, p2):
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        theta = np.arctan2(dy, dx)
        angle = np.degrees(theta)  
        return angle
    
    def line_bw_points(self,A, B):
        x_coff = B[1] - A[1] 
        y_coff = A[0] - B[0]
        c = x_coff*A[0] + y_coff*A[1]
        return (x_coff, y_coff, c)
        
    def intersect_point_line(p,l1,l2):
    
        s1 = (l2[1]-l1[1])/(l2[0]-l1[0])
        s2 = -1/s1
    
        c2 = p[1]-s2*p[0]
        c1 = l2[1] - s1*l2[0]
    
        x = (c2-c1)/(s1-s2)
        y = (s1*c2 - s2*c1)/(s1-s2)
        return(x,y)


    def perpendicular_dis(self, A, line):
        return abs((line[0] * A[0] + line[1] * A[1] + line[2])) / np.sqrt(np.square(line[0]) + np.square(line[1]))
        
    def distance_bw_points(self,A, B):
        A = np.array(A)
        B = np.array(B)
    
        return np.linalg.norm(A - B)
        
    def perpendicular_point(self, l1, l2, p):
        line = (l1, l2)
        point = (p[0], p[1])
        intersect = intersect_point_line(point, line[0], line[1])
        return (intersect[0], intersect[1])


    def getEquidistantPoints(self,p1, p2, parts):
        return zip(np.linspace(p1[0], p2[0], parts+1),
                   np.linspace(p1[1], p2[1], parts+1))

    def check_intersect(self,l1, l2):
        line = LineString(l1)
        other = LineString(l2)
        return line.intersects(other)
   
    
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
