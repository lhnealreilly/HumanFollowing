import random
import numpy as np
import math
#
#   Defines the random generation of human paths with some constraint options.
#
class HumanPathGenerator: 
    def __init__(self, max_turn_raidus=(math.pi/3), max_velocity=1.31):
        self.max_turn_raidus = max_turn_raidus
        self.max_velocity = max_velocity
    
    #Generate a random path with a path_length number of points and a delta of time_step in milliseconds between each
    def generatePath(self, path_length=100, time_step=10):
        path = [[0, 0, 0]]
        velocity = [0, 0]
        for i in range(path_length - 1):
            # generate a random turn angle
            turn = random.gauss(0, (self.max_turn_raidus * time_step / 1000)/2)
            # update the orientation of the human
            theta = path[-1][2] + turn
            # generate a random step size
            step = random.gauss(self.max_velocity / 1000 * time_step, (self.max_velocity / 1000 * time_step)/3)
            # compute the new position of the human
            x = path[-1][0] + step*math.sin(theta)
            y = path[-1][1] + step*math.cos(theta)
            # add the new position and orientation to the path
            path.append([x, y, theta])
        return path


    def generateMultiplePaths(self, num_paths=1):
        return [self.generatePath() for x in range(num_paths)]
    

