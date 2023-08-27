import numpy as np
import matplotlib.pyplot as plt
from numpy import linalg as LA

map = np.load('risk_zones.npy')

def planLineLow(v0,v1):                         # Takes in tuples of vertices. Planning straight line path for slopes < 1
    # x incremented more frequently than y. 
    pixels = []
    heuristic_cost = 0
    dx = v1[0]-v0[0]                            # difference between max e and min e
    dy = v1[1]-v0[1]                            # difference between max n and min n
    yi = 1
    if dy<0:
        yi = -1
        dy = -dy
    y_increment_decider = (2 * dy) - dx             # The quantity that decides whether or not to increment y (n coordinate)           
    y = v0[1]
    for x in range(v0[0],v1[0]+1):
        pixels.append((x,y))                        # Adding the tuple to the list

        # Calculating cost on the raster path
        if map[x,y] == 0:
            heuristic_cost+=1                       # if the planned pixel lies in low risk zone
        elif map[x,y] == 1:
            heuristic_cost+=1.5                       # if the planned pixel lies in high risk zone
        else:
            heuristic_cost+=2                        # if the planned pixel lies inside the keep out zone

        if y_increment_decider>0 :
            y = y + yi
            y_increment_decider = y_increment_decider + (2*(dy-dx))
        else :
            y_increment_decider = y_increment_decider + 2*dy
    return heuristic_cost,pixels


def planLineHigh(v0,v1):           # Takes in tuples of vertices. Planning straight line path for slopes > 1
    # y incremented more frequently than x.
    pixels = []
    heuristic_cost = 0
    dx = v1[0]-v0[0]                            # difference between max e and min e
    dy = v1[1]-v0[1]                            # difference between max n and min n
    xi = 1
    if dx<0:
        xi = -1
        dx = -dx
    x_increment_decider = (2 * dx) - dy                           # The quantity that decides whether or not to increment x (e coordinate)
    x = v0[0]
    for y in range(v0[1],v1[1]+1):
        pixels.append((x,y))          # Adding the tuple to the list
        # Calculating cost on the raster path
        if map[x,y] == 0:
            heuristic_cost+=1         # if the planned pixel lies in low risk zone
        elif map[x,y] == 1:
            heuristic_cost+=1.5       # if the planned pixel lies in high risk zone
        else:
            heuristic_cost+=2         # if the planned pixel lies inside an obstacle
        if x_increment_decider>0 :
            x = x + xi
            x_increment_decider = x_increment_decider + (2*(dx-dy))
        else :
            x_increment_decider = x_increment_decider + 2*dx
    return heuristic_cost,pixels


def rasterize(v0,v1) :
    pixels = []
    if abs(v1[1]-v0[1])<abs(v1[0]-v0[0]):
        if v0[0]>v1[0] :
            return planLineLow(v1,v0)
        else :
            return planLineLow(v0,v1)
    else :
        if v0[1]>v1[1] :
            return planLineHigh(v1,v0)
        else :
            return planLineHigh(v0,v1)
        

def calculate_heuristic(v0,v1):
    return rasterize(v0,v1)[0]

def get_straight_line_path(v0,v1):
    path = rasterize(v0,v1)[1]
    if path[0]!=v0:
        path = path[::-1]
    return path
