import numpy as np
import math
import bisect
import random
from sklearn.neighbors import KDTree
import matplotlib.pyplot as plt
import time

vmax = 10
(xmin, xmax) = (0, 14)
(ymin, ymax) = (0, 10)
tmax = 30
(startx, starty) = (0, 4)
(goalx,  goaly)  = (14, 5)
obstacles = []

dstep = 1
Nmax  = 1000

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
    def InFreespace(self):
        for obstacle in obstacles:
            xpos, ypos = obstacle.get_position(self.t)
            if np.sqrt((self.y - ypos)**2 + (self.x - xpos)**2) <= obstacle.r:
                return False
        return True

    def Coordinates(self):
        return (self.x, self.y)

    def Distance(self, other):
        return np.sqrt((other.x - self.x)**2 + (other.y - self.y)**2)

    def DistSquared(self, other):
        return ((self.x - other.x)**2 + (self.y - other.y)**2)

    def Intermediate(self, other, alpha):
        return State(self.x + alpha * (other.x - self.x),
                     self.y + alpha * (other.y - self.y),
                     self.t + alpha * (other.t - self.t))

    def ConnectsTo(self, other):
        if other.t < self.t:
            return False
        if np.sqrt((self.x - other.x)**2 +(self.y - other.y)**2)/ \
                    (other.t - self.t) > vmax:
            return False
        for i in range(1,101):
            if not self.Intermediate(other, i/100).InFreespace():
                return False
        return True

#
#   Node class upon which to build the graph (roadmap) and which
#   supports the A* search tree.
#
class Node:
    def __init__(self, state, parentnode):
        # Save the state matching this node.
        self.state = state

        # Link to parent for the tree structure.
        self.parent = parentnode

        # Automatically draw.
        self.Draw('r-', linewidth=1)

    # Draw a line to the parent.
    def Draw(self, *args, **kwargs):
        if self.parent is not None:
            plt.quiver(
                self.parent.state.x, self.parent.state.y,
                self.state.x - self.parent.state.x,
                self.state.y - self.parent.state.y,
                scale_units='xy', angles='xy', scale=1, color=(self.state.t/tmax, 0, 0))
            plt.pause(0.001)


######################################################################
#
#   Visualization
#
class Visualization:
    def __init__(self):
        # Clear and show.
        self.ClearFigure()
        self.ShowFigure()

    def ClearFigure(self):
        # Clear the current, or create a new figure.
        plt.clf()

        # Create a new axes, enable the grid, and set axis limits.
        plt.axes()
        plt.grid(True)
        plt.gca().axis('on')
        plt.gca().set_xlim(xmin, xmax)
        plt.gca().set_ylim(ymin, ymax)
        plt.gca().set_aspect('equal')


    def ShowFigure(self):
        # Show the plot.
        plt.pause(0.001)


    def DrawState(self, state, *args, **kwargs):
        plt.plot(state.x, state.y, *args, **kwargs)

    def DrawLocalPath(self, head, tail, *args, **kwargs):
        plt.plot((head.x, tail.x),
                 (head.y, tail.y), *args, **kwargs)

    def DrawObstacle(self, obstacle, t, *args, **kwargs):
        x, y = obstacle.get_position(t)
        circ = plt.Circle((x, y), obstacle.r)
        plt.gca().add_patch(circ)


######################################################################
#
#   RRT Functions
#
#   Again I am distiguishing state (containing x/y information) and
#   node (containing tree structure/parent information).
#
def RRT(tree, goalstate, Nmax):
    # Loop.
    while True:
        # Determine the target state.

        # 5% of time choose target routine
        target_coin = random.uniform(0, 1)
        if target_coin <= 0.05:
            x = goalstate.x
            y = goalstate.y
            t = goalstate.t
        else:
            x = random.uniform(xmin, xmax)
            y = random.uniform(ymin, ymax)
            t = random.uniform(0, tmax)

        targetstate = State(x,y,t)        # How to pick x/y?

        # Find the nearest node (node with state nearest the target state).
        # This is inefficient (slow for large trees), but simple.
        list = [(node.state.DistSquared(targetstate), node) for node in tree]
        (d2, nearnode)  = min(list)
        d = np.sqrt(d2)
        nearstate = nearnode.state

        # Determine the next state, a step size (dstep) away
        alpha = min(dstep / np.sqrt(nearstate.DistSquared(targetstate)), 1)

        nextstate = nearstate.Intermediate(targetstate, alpha)

        # Check whether to attach (creating a new node).
        if nearstate.ConnectsTo(nextstate):
            nextnode = Node(nextstate, nearnode)
            tree.append(nextnode)

            # Also try to connect the goal.
            if np.sqrt(nextstate.DistSquared(goalstate)) < dstep and nextstate.ConnectsTo(goalstate):
                goalnode = Node(goalstate, nextnode)
                tree.append(goalnode)
                return(goalnode)

        # Check whether we should abort (tree has gotten too large).
        if (len(tree) >= Nmax):
            return None

#
#  Main Code
#
def main():
    # Report the parameters.
    print('Running with step size ', dstep, ' and up to ', Nmax, ' nodes.')

    # Create the figure.
    Visual = Visualization()

    # Create the start/goal nodes.
    startstate = State(startx, starty, 0)
    goalstate  = State(goalx,  goaly, tmax)

    # Show the start/goal states.
    Visual.DrawState(startstate, 'ro')
    Visual.DrawState(goalstate,  'ro')
    Visual.ShowFigure()
    input("Showing basic world (hit return to continue)")

    # Start the tree with the start state and no parent.
    tree = [Node(startstate, None)]

    # Execute the search (return the goal leaf node).
    node = RRT(tree, goalstate, Nmax)

    # Check the outcome
    if node is None:
        print("UNABLE TO FIND A PATH in %d steps", Nmax)
        input("(hit return to exit)")
        return

    path = []
    while node.parent is not None:
        path.insert(0, node)
        node = node.parent

    # Show the path.
    for i in range(len(path)):
        Visual.ClearFigure()
        Visual.DrawState(startstate, 'bo')
        Visual.DrawState(goalstate,  'bo')

        Visual.DrawState(path[i].state, 'ro', linewidth=1)

        for obstacle in obstacles:
            Visual.DrawObstacle(obstacle, path[i].state.t)

        Visual.ShowFigure()
        input("Showing the PATH(hit return to continue)")

    full_viz_states = []
    for i in range(len(path) - 1):
        dt = .05
        diff = path[i+1].state.t - path[i].state.t
        for j in range(int(diff/dt)):
            full_viz_states.append(path[i].state.Intermediate(path[i + 1].state, j * dt/diff))
    full_viz_states.append(path[-1].state)

    for s in full_viz_states:
        Visual.ClearFigure()
        Visual.DrawState(startstate, 'bo')
        Visual.DrawState(goalstate,  'bo')
        Visual.DrawState(s, 'ro', linewidth=1)
        for obstacle in obstacles:
            Visual.DrawObstacle(obstacle, s.t)
        Visual.ShowFigure()

    input("Showing the raw path (hit return to continue)")

if __name__== "__main__":
    for obs in obstacles:
        print(obs.get_position(1))
    main()
