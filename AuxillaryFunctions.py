import numpy as np
from numpy import linalg as LA

def eight_neighbors(v,occ_grid):
    neighbor_set = set()
    
    i = v[0]
    j = v[1]
    for p in range(-1,2): # Checking three rows around the current pixel for neighbors. 
        for q in range(-1,2): # Checking three columns around the current pixel for neighbors. 
            if p==0 and q==0:   
                continue
            elif i+p>=0 and i+p<=occ_grid.shape[0]-1 and j+q>=0 and j+q<=occ_grid.shape[1]-1 :  # Inside the square. 
                if occ_grid[i+p,j+q] == 0 or occ_grid[i+p,j+q] == 1:      # Adding a neighbor only if its pixel value is 0 or 1.
                    neighbor_set.add((i+p,j+q))  
            else : continue 
    return neighbor_set


occ_grid = np.load('risk_zones.npy')
# print(occ_grid.shape) 
vertices = set()  # Set of vertices
neighbors = dict()  # dictiionary to store set of neighbors

for i in range(occ_grid.shape[0]):
    for j in range(occ_grid.shape[1]):
        if occ_grid[i,j] == 0 or occ_grid[i,j] == 1:    # Adding a vertex to the graph only if its pixel value is 0 or 1.
            vertex = (i,j)
            vertices.add(vertex)
            neighbors[vertex] = eight_neighbors(vertex,occ_grid) 


def euclidean_distance(v1,v2):
    # print(v1)
    return LA.norm([v2[0]-v1[0],v2[1]-v1[1]])

def RecoverPath(s,g,pred): # Takes in the predecessor dictionary
    list = [g]
    v = g   # Setting v as the goal node. 
    while v!=s:
        list.insert(0,pred[v])
        v = pred[v]   # Setting v as the predecessor
    return list 

def pathlength(path):
    length = 0
    for i in range(0,len(path)-1):
        length = length + euclidean_distance(path[i],path[i+1])
    return length