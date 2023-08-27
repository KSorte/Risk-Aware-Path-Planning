"""
    This module is your primary workspace. Add whatever helper functions, classes, data structures, imports... etc here.

    We expect most results will utilize more than just dumping code into the plan_paths()
        function, that just serves as a meaningful entry point.

    In order for the rest of the scoring to work, you need to make sure you have correctly
        populated the DeliverySite.path for each result you produce.
"""
import typing
import numpy as np
from nest_info import Coordinate, DeliverySite, NestInfo
import search_path
import score_paths
import rasterize
map = np.load('risk_zones.npy')

class PathPlanner:
    def __init__(self, nest_info: NestInfo, delivery_sites: typing.List["DeliverySite"]):
        self.nest_info: NestInfo = nest_info
        self.delivery_sites: typing.List["DeliverySite"] = delivery_sites

    def plan_paths(self):
        """
        This is the function you should re-write. It is expected to mutate the list of
        delivery_sites by calling each DeliverySite's set_path() with the resulting
        path as an argument.

        The default construction shows this format, and should produce 10 invalid paths.
        """
        for site in self.delivery_sites:
            # YOUR CODE REPLACES THIS / WILL PLUG IN HERE
            nest_coord = self.nest_info.nest_coord    # a tuple
            site_coord = site.coord
            
            # Converting the coordinates to a standard tuple before passing to the search algorithm
            nest_tuple = (nest_coord.e,nest_coord.n)
            site_tuple = (site_coord.e,site_coord.n)

            max_range = self.nest_info.maximum_range    # Getting the max range of the 

            ############# PLANNER BEGINS ####################
            path_array = search_path.augmented_astar_search(nest_tuple, site_tuple)       #PATH SEARCHING using Risk aware A* search.

            if (score_paths.get_path_length(path_array)>max_range):
                alternative_path = rasterize.get_straight_line_path(nest_tuple,site_tuple)           # Straight line alternate path if range exceeded
                for pixel in alternative_path:
                    if map[pixel[0],pixel[1]] == 2:
                        alternative_path = search_path.astar_search(nest_tuple,site_tuple) # A* path is straight line path enters keep out.
                path_array = alternative_path
            ################### PLANNER ENDS ##########################
            path_coords = [Coordinate(arr[0], arr[1]) for arr in path_array]
            # Once you have a solution for the site - populate it like this:
            site.set_path(path_coords)
