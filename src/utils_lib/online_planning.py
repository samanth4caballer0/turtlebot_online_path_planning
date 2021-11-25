import numpy as np
import math
from ompl import base as ob
from ompl import geometric as og

def wrap_angle(angle):
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )

class StateValidityChecker:
    """ Checks if a position or a path is valid given an occupancy map."""

    def __init__(self, distance=0.1):
        self.map = None                             # map: 2D array of integers [-127,128] which categorize world occupancy
        self.resolution = None                      # map sampling resolution
        self.origin = None                          # world position of (0,0) element of the map
        self.there_is_map = False                   # set method has been called
        self.distance = distance                    # radius arround the robot used to check occupancy of a trial pose
        
    def set(self, data, resolution, origin):
        self.map = data
        self.resolution = resolution
        self.origin = np.array(origin)
        self.there_is_map = True
        
    def is_valid(self, pose): 

        # TODO: convert world robot position to map coordinates using method __position_to_map__
        p = ... 

        # TODO: check occupancy of the vicinity of robot position following self.distance atribude. Be aware to only generate elements inside the map.  
        if len(p) == 2: # if p is outside the map return true (no explored positions are considered true)
            u_min = ...
            v_min = ...
            u_max = ...
            v_max = ...
            if np.any(self.map[u_min:u_max, v_min:v_max] > 0):
                return False                                        # Obstacle
        return True

    def check_path(self, path, min_dist=0.1):
        for i in range(len(path) - 1):

            # TODO: get dist and angle between current element and next element in path
            angle = ...
            dist = ...

            for d in np.arange(0, dist, min_dist):

                # TODO: check occupancy of each element d. If one element is occupied return False. 
                p = ...
                #if ...
                
        return True

    def __position_to_map__(self, p):

        # TODO: convert world position to map coordinates
        uv = ...

        # keep position inside map
        if uv[0] < 0 or uv[0] >= self.map.shape[0] or uv[1] < 0 or uv[1] >= self.map.shape[1]:
            return []
        return uv
    

# Planner
def compute_path(start_p, goal_p, state_validity_checker, dominion, max_time=1.0):

    # TODO: Plan a path from start_p to goal_p inside dominion using the OMPL and the state_validity_checker object. Follow notebook example.
    # some code

    ret = []
    # TODO: if solved fill ret with the points [x, y] in the solution path

    return ret


# Controller
def move_to_point(current, goal, Kv=0.5, Kw=0.5):
    
    # TODO: Implement a proportional controller which sets a velocity command to move from current position to goal (u = Ke)
    # Make it sequential to avoid strange curves. First correct orientation and then distance. 
    # Hint: use wrap_angle function to maintain yaw in [-pi, pi]
    v = ...     # linear velocity
    w = ...     # angular velocity
    
    return v, w
