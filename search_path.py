import numpy as np
import AuxillaryFunctions
import rasterize
map = np.load('risk_zones.npy')

# IMPORTING THE MAP AS A GRAPH WITH 8 CONNECTED NEIGHBORS
vertices = AuxillaryFunctions.vertices
neighbors = AuxillaryFunctions.neighbors


############ AUGMENTED A STAR ###############   
def augmented_astar_search(nest,site):
    
    # INITIALIZING DICTIONARIES FOR THE SEARCH ALGORITHM
    path = []                 # Initializing an empty path     
    EstTotalCost = dict()
    CostTo = dict()
    heuristic = dict()        # Dictionary to store heuristic
    EstTotalCost[nest] = rasterize.calculate_heuristic(nest,site)
    pred = dict()             # Dictionary to store the predecessors. 

    # POPULATING DICTIONARIES FOR ALL VERTICES IN THE GRAPH
    for vertex in vertices:
        CostTo[vertex] = float('inf')
        EstTotalCost[vertex] = float('inf')
        heuristic[vertex] = rasterize.calculate_heuristic(vertex,site)
    CostTo[nest] = 0          # Cost to te nest is zero
    EstTotalCost[nest] = heuristic[nest]    # Setting heuristic for start node as the estimated total cost for the node.
    queue = dict()                          # Initializing the queue dictionary
    queue[nest] = EstTotalCost[nest]


    while len(queue)>0 :
        vertex = list(queue.keys())[0]     # Getting the vertex with the smallest estimated total cost. The dictionary is sorted by value of est total cost. 
        del queue[vertex]                  # Removing that vertex from the dictionary
        if vertex==site:                   # Recovering the path if we reach the site
            print('Recovering path ... ')
            path = AuxillaryFunctions.RecoverPath(nest,site,pred)
            print("Path Recovered : A* successful")
            return path
        for neighbor in neighbors[vertex]:
            # cost to go and heuristic are calculated as the number of pixels. 
            # if the pixel is in the low risk zone, its cost is 1. 
            # if the pixel is in the high risk zone, its cost is higher. (Tunable)

            if map[neighbor[0],neighbor[1]] == 0 :
                cost_to_neighbor = CostTo[vertex] + 1       # neighbor is a low risk pixel
            if map[neighbor[0],neighbor[1]] == 1 :
                cost_to_neighbor = CostTo[vertex] + 1.5   # neighbor is a high risk pixel. Higher cost added.

            if cost_to_neighbor < CostTo[neighbor] :
                pred[neighbor] = vertex 
                CostTo[neighbor] = cost_to_neighbor
                EstTotalCost[neighbor] = cost_to_neighbor + heuristic[neighbor]
                queue[neighbor] = EstTotalCost[neighbor]    # Updating the estimated total cost of the vertex in the dictionary
                dict(sorted(queue.items(), key=lambda item: item[1]))   # Sorting the dictionary. 

    
    return path


############# A STAR SEARCH ####################
def astar_search(nest,site):

    # INITIALIZING DICTIONARIES FOR THE SEARCH ALGORITHM
    path = []                            # Initializing an empty path     
    EstTotalCost = dict()
    CostTo = dict()
    heuristic = dict()                   # Dictionary to store heuristic
    EstTotalCost[nest] = AuxillaryFunctions.euclidean_distance(nest,site)   # Cost for start node to reach goal 
    pred = dict()                        # Dictionary to store the predecessors. 

    # POPULATING DICTIONARIES FOR ALL VERTICES IN THE GRAPH
    for vertex in vertices:
        CostTo[vertex] = float('inf')
        EstTotalCost[vertex] = float('inf')
        heuristic[vertex] = AuxillaryFunctions.euclidean_distance(vertex,site)  
    CostTo[nest] = 0                                  # Cost to go the nest itself is zero since that is the starting point
    EstTotalCost[nest] = heuristic[nest]              # Setting heuristic for start node as the estimated total cost for the node.
    queue = dict()                                    # Initializing the queue dictionary
    queue[nest] = EstTotalCost[nest]


    while len(queue)>0 :
        vertex = list(queue.keys())[0]    # Getting the vertex with the smallest estimated total cost. The dictionary is sorted by value of est total cost. 
        del queue[vertex]                 # Removing that vertex from the dictionary
        if vertex==site:                  # Recovering the path if we reach the site
            print('Recovering path ... ')
            print('Recovering path ... ')
            path = AuxillaryFunctions.RecoverPath(nest,site,pred)
            print("Path Recovered : A* successful")
            return path
        for neighbor in neighbors[vertex]:
            cost_to_neighbor = CostTo[vertex] + AuxillaryFunctions.euclidean_distance(vertex,neighbor)  # Represents the total cost to get TO THE NODE i
            if cost_to_neighbor < CostTo[neighbor] :
                pred[neighbor] = vertex 
                CostTo[neighbor] = cost_to_neighbor
                EstTotalCost[neighbor] = cost_to_neighbor + heuristic[neighbor]
                queue[neighbor] = EstTotalCost[neighbor]                              # Updating the estimated total cost of the vertex in the dictionary
                dict(sorted(queue.items(), key=lambda item: item[1]))                 # Sorting the dictionary. 

    return path