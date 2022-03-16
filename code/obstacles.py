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

def obstacles_1():
    obstacles = []
    obstacles.append(Obstacle(0.5, lambda t: 1, lambda t: 5 + 4.5 * np.sin(t)))
    obstacles.append(Obstacle(0.5, lambda t: 2, lambda t: 5 - 4.5 * np.sin(t)))
    obstacles.append(Obstacle(0.5, lambda t: 3, lambda t: 5 + 4.5 * np.sin(t)))
    obstacles.append(Obstacle(0.5, lambda t: 4, lambda t: 5 - 4.5 * np.sin(t)))
    obstacles.append(Obstacle(0.5, lambda t: 5, lambda t: 5 + 4.5 * np.sin(t)))
    obstacles.append(Obstacle(0.5, lambda t: 6, lambda t: 5 - 4.5 * np.sin(t)))
    obstacles.append(Obstacle(0.5, lambda t: 7, lambda t: 5 + 4.5 * np.sin(t)))
    obstacles.append(Obstacle(0.5, lambda t: 8, lambda t: 5 - 4.5 * np.sin(t)))
    obstacles.append(Obstacle(0.5, lambda t: 9, lambda t: 5 + 4.5 * np.sin(t)))
    obstacles.append(Obstacle(0.5, lambda t: 10, lambda t: 5 - 4.5 * np.sin(t)))
    obstacles.append(Obstacle(0.5, lambda t: 11, lambda t: 5 + 4.5 * np.sin(t)))
    obstacles.append(Obstacle(0.5, lambda t: 12, lambda t: 5 - 4.5 * np.sin(t)))
    obstacles.append(Obstacle(0.5, lambda t: 13, lambda t: 5 + 4.5 * np.sin(t)))
    return obstacles

def obstacles_2():
    obstacles = []

    # Bottom row
    obstacles.append(Obstacle(1/2, lambda t: 2 + 1/2 * np.cos(t), lambda t: 2 + 1/2 * np.sin(t)))
    obstacles.append(Obstacle(1/2, lambda t: 2 + 1/2 * np.cos(t + np.pi), lambda t: 2 + 1/2 * np.sin(t + np.pi)))

    obstacles.append(Obstacle(1/2, lambda t: 2 + 1/2 * np.cos(t), lambda t: 4 + 1/2 * np.sin(t)))
    obstacles.append(Obstacle(1/2, lambda t: 2 + 1/2 * np.cos(t + np.pi), lambda t: 4 + 1/2 * np.sin(t + np.pi)))

    obstacles.append(Obstacle(1/2, lambda t: 4 + 1/2 * np.cos(t + np.pi/2), lambda t: 2 + 1/2 * np.sin(t + np.pi/2)))
    obstacles.append(Obstacle(1/2, lambda t: 4 + 1/2 * np.cos(t + 3 * np.pi / 2), lambda t: 2 + 1/2 * np.sin(t + 3 * np.pi / 2)))

    obstacles.append(Obstacle(1/2, lambda t: 4 + 1/2 * np.cos(t + np.pi/2), lambda t: 4 + 1/2 * np.sin(t + np.pi/2)))
    obstacles.append(Obstacle(1/2, lambda t: 4 + 1/2 * np.cos(t + 3 * np.pi / 2), lambda t: 4 + 1/2 * np.sin(t + 3 * np.pi / 2)))

    obstacles.append(Obstacle(1/2, lambda t: 6 + 1/2 * np.cos(t), lambda t: 2 + 1/2 * np.sin(t)))
    obstacles.append(Obstacle(1/2, lambda t: 6 + 1/2 * np.cos(t + np.pi), lambda t: 2 + 1/2 * np.sin(t + np.pi)))

    obstacles.append(Obstacle(1/2, lambda t: 6 + 1/2 * np.cos(t), lambda t: 4 + 1/2 * np.sin(t)))
    obstacles.append(Obstacle(1/2, lambda t: 6 + 1/2 * np.cos(t + np.pi), lambda t: 4 + 1/2 * np.sin(t + np.pi)))

    obstacles.append(Obstacle(1/2, lambda t: 8 + 1/2 * np.cos(t + np.pi/2), lambda t: 2 + 1/2 * np.sin(t + np.pi/2)))
    obstacles.append(Obstacle(1/2, lambda t: 8 + 1/2 * np.cos(t + 3 * np.pi / 2), lambda t: 2 + 1/2 * np.sin(t + 3 * np.pi / 2)))

    obstacles.append(Obstacle(1/2, lambda t: 8 + 1/2 * np.cos(t + np.pi/2), lambda t: 4 + 1/2 * np.sin(t + np.pi/2)))
    obstacles.append(Obstacle(1/2, lambda t: 8 + 1/2 * np.cos(t + 3 * np.pi / 2), lambda t: 4 + 1/2 * np.sin(t + 3 * np.pi / 2)))

    obstacles.append(Obstacle(1/2, lambda t: 10 + 1/2 * np.cos(t), lambda t: 2 + 1/2 * np.sin(t)))
    obstacles.append(Obstacle(1/2, lambda t: 10 + 1/2 * np.cos(t + np.pi), lambda t: 2 + 1/2 * np.sin(t + np.pi)))

    obstacles.append(Obstacle(1/2, lambda t: 10 + 1/2 * np.cos(t), lambda t: 4 + 1/2 * np.sin(t)))
    obstacles.append(Obstacle(1/2, lambda t: 10 + 1/2 * np.cos(t + np.pi), lambda t: 4 + 1/2 * np.sin(t + np.pi)))

    obstacles.append(Obstacle(1/2, lambda t: 12 + 1/2 * np.cos(t + np.pi/2), lambda t: 2 + 1/2 * np.sin(t + np.pi/2)))
    obstacles.append(Obstacle(1/2, lambda t: 12 + 1/2 * np.cos(t + 3 * np.pi / 2), lambda t: 2 + 1/2 * np.sin(t + 3 * np.pi / 2)))

    obstacles.append(Obstacle(1/2, lambda t: 12 + 1/2 * np.cos(t + np.pi/2), lambda t: 4 + 1/2 * np.sin(t + np.pi/2)))
    obstacles.append(Obstacle(1/2, lambda t: 12 + 1/2 * np.cos(t + 3 * np.pi / 2), lambda t: 4 + 1/2 * np.sin(t + 3 * np.pi / 2)))

    # Top row
    obstacles.append(Obstacle(1/2, lambda t: 2 + 1/2 * np.cos(t), lambda t: 6 + 1/2 * np.sin(t)))
    obstacles.append(Obstacle(1/2, lambda t: 2 + 1/2 * np.cos(t + np.pi), lambda t: 6 + 1/2 * np.sin(t + np.pi)))

    obstacles.append(Obstacle(1/2, lambda t: 2 + 1/2 * np.cos(t), lambda t: 8 + 1/2 * np.sin(t)))
    obstacles.append(Obstacle(1/2, lambda t: 2 + 1/2 * np.cos(t + np.pi), lambda t: 8 + 1/2 * np.sin(t + np.pi)))

    obstacles.append(Obstacle(1/2, lambda t: 4 + 1/2 * np.cos(t + np.pi/2), lambda t: 6 + 1/2 * np.sin(t + np.pi/2)))
    obstacles.append(Obstacle(1/2, lambda t: 4 + 1/2 * np.cos(t + 3 * np.pi / 2), lambda t: 6 + 1/2 * np.sin(t + 3 * np.pi / 2)))

    obstacles.append(Obstacle(1/2, lambda t: 4 + 1/2 * np.cos(t + np.pi/2), lambda t: 8 + 1/2 * np.sin(t + np.pi/2)))
    obstacles.append(Obstacle(1/2, lambda t: 4 + 1/2 * np.cos(t + 3 * np.pi / 2), lambda t: 8 + 1/2 * np.sin(t + 3 * np.pi / 2)))

    obstacles.append(Obstacle(1/2, lambda t: 6 + 1/2 * np.cos(t), lambda t: 6 + 1/2 * np.sin(t)))
    obstacles.append(Obstacle(1/2, lambda t: 6 + 1/2 * np.cos(t + np.pi), lambda t: 6 + 1/2 * np.sin(t + np.pi)))

    obstacles.append(Obstacle(1/2, lambda t: 6 + 1/2 * np.cos(t), lambda t: 8 + 1/2 * np.sin(t)))
    obstacles.append(Obstacle(1/2, lambda t: 6 + 1/2 * np.cos(t + np.pi), lambda t: 8 + 1/2 * np.sin(t + np.pi)))

    obstacles.append(Obstacle(1/2, lambda t: 8 + 1/2 * np.cos(t + np.pi/2), lambda t: 6 + 1/2 * np.sin(t + np.pi/2)))
    obstacles.append(Obstacle(1/2, lambda t: 8 + 1/2 * np.cos(t + 3 * np.pi / 2), lambda t: 6 + 1/2 * np.sin(t + 3 * np.pi / 2)))

    obstacles.append(Obstacle(1/2, lambda t: 8 + 1/2 * np.cos(t + np.pi/2), lambda t: 8 + 1/2 * np.sin(t + np.pi/2)))
    obstacles.append(Obstacle(1/2, lambda t: 8 + 1/2 * np.cos(t + 3 * np.pi / 2), lambda t: 8 + 1/2 * np.sin(t + 3 * np.pi / 2)))

    obstacles.append(Obstacle(1/2, lambda t: 10 + 1/2 * np.cos(t), lambda t: 6 + 1/2 * np.sin(t)))
    obstacles.append(Obstacle(1/2, lambda t: 10 + 1/2 * np.cos(t + np.pi), lambda t: 6 + 1/2 * np.sin(t + np.pi)))

    obstacles.append(Obstacle(1/2, lambda t: 10 + 1/2 * np.cos(t), lambda t: 8 + 1/2 * np.sin(t)))
    obstacles.append(Obstacle(1/2, lambda t: 10 + 1/2 * np.cos(t + np.pi), lambda t: 8 + 1/2 * np.sin(t + np.pi)))

    obstacles.append(Obstacle(1/2, lambda t: 12 + 1/2 * np.cos(t + np.pi/2), lambda t: 6 + 1/2 * np.sin(t + np.pi/2)))
    obstacles.append(Obstacle(1/2, lambda t: 12 + 1/2 * np.cos(t + 3 * np.pi / 2), lambda t: 6 + 1/2 * np.sin(t + 3 * np.pi / 2)))

    obstacles.append(Obstacle(1/2, lambda t: 12 + 1/2 * np.cos(t + np.pi/2), lambda t: 8 + 1/2 * np.sin(t + np.pi/2)))
    obstacles.append(Obstacle(1/2, lambda t: 12 + 1/2 * np.cos(t + 3 * np.pi / 2), lambda t: 8 + 1/2 * np.sin(t + 3 * np.pi / 2)))


    return obstacles
