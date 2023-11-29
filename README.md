
In this repository I have solved the problem outlined below : 
# The Problem
A new drone nest is opened internally called `BON-0`! After a large cross team effort,
all of the information for the new nest has been compiled into one file `nest_info.yaml`.
This includes the nest coordinates, the coordinates for numerous delivery sites, and the maximum range of the zip.
The only thing missing is the paths to get to those excited customers!
After diligent work by the geographical information team, you have also been provided with a "risk map".
This map classifies the land as "low risk", "high risk", or "keep out".
Your job is to make a path planning algorithm to make paths from the nest to each site that stays out of
keep out zones, _minimizes_ entry into high risk zones, and gets to the site within the alloted range of the zip. _You don't need to worry about inbound paths, the zip can just follow the same path home_

# Getting Started Instructions

1. Install requirements from requirements.txt (usually this means `pip install -U -r requirements.txt`). There's nothing fancy going on in the starter code, but if you have an incompatible version of numpy or matplotlib, you'll have a tricky time getting the helper code to run.
2. From this directory - Run `python3 test_planner.py`. You should see a plot showing the Risk Map, Nest Location, Site Locations, and some (_invalid and risky_) paths to each site. This will write a copy of the plot to `results_fig.png`. This function will also evaluate the path to each site for validity and evaluate the quality of the path.
3. Your task is to replace the function `PathPlanner.plan_path` in `path_planner.py` to plan the "best" paths to all sites. Paths must follow these rules to be considered valid:
    - __Must__ not enter keep out zones, as denoted by a value in the risk map of 2
    - All path locations __must__ be integers
    - All path steps __must__ take place on an 8 connected grid of step size one
    - Paths __must__ stay within the coordinates of the risk map
    - Paths __must__ start at the nest and end at the site
    - Length of path __must__ be less than specified maximum in nest_info.maximum_range
    - Paths _should_ minimize the accumulated risk of paraland. Accumulated risk is calculated as the total number of risk pixels that path steps end in
4. Once you have written your code, test it with `python3 test_planner.py` again
5. In addition to your source code, please submit an informal description of your algorithm, a copy of `results_fig.png` with your paths, a copy of `results.yaml`, as well as any other graphics that you think will help us understand whats going on with your algorithm!


#### Notes:

- Be careful of the coordinate order! The map stores coordinates in (east, north) order. The bottom left of the plot corresponds to 0,0 (similar to a mathematical x,y axis).
-  Only the path step start and end points are considered (so cutting corners is a valid tactic!)
