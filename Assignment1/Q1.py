#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 19 12:21:11 2022

@author: dse
"""

import numpy as np
import math

def line(x1,y1,x2,y2,x = None):
    slope = (y2-y1)/(x2-x1)
    c = y1 - slope*x1
    return slope,c

def euc_distance(x1,y1,x2,y2):
    distance = math.sqrt((x2-x1)**2+(y2-y1)**2)
    return distance

def dist_line(x,y,slope,c):
    dist = abs(slope*x-y+c)/math.sqrt(slope**2 + 1)
    return dist

def dist_poly(vertices,x,y):
    distance = []
    for i in range(len(vertices)-1):
        slope , c = line(vertices[i][0],vertices[i][1],vertices[i+1][0],vertices[i+1][1])
        dist = dist_line(x,y,slope,c)
        distance.append(dist)
    return min(distance)


