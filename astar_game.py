import numpy as np

class Obstacle:
    '''
    Class for a moving circular obstacle
    '''
    def __init__(self, r, fx, fy):
        self.r = r
        self.fx = fx
        self.fy = fy
    def get_position(self, t):
        return (self.fx(t), self.fy(t))

class State:
    def __init__(self, x, y, t):
        self.x = x
        self.y = y
        self.t = t
    def __repr__(self):
        return ("<XY %5.2f,%5.2f @ %5.1f s>" %
                (self.x, self.y, self.t))
    def Draw(self, fig, **kwargs):
        return
    def InFreeSpace(self, obstacles):
        for obstacle in obstacles:
            xpos, ypos = obstacle.get_position(t)
            if np.sqrt((self.y - ypos)**2 + (self.x - xpos)**2) <= obstacle.r:
                return False
        return True
    def 
