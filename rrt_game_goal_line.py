import numpy as np
import math
import bisect
import random
from sklearn.neighbors import KDTree
import matplotlib.pyplot as plt
import time

from code.obstacles import *
from code.visuals import *
from code.rrt_utils import *

# Restrict dot movement + x, y, t domains
vmax = 10
tmax = 30
(xmin, xmax) = (0, 14)
(ymin, ymax) = (0, 10)

# Define start and goal x, y
(startx, starty) = (0, 4)
(goalx,  goaly)  = (14, 5)

# Grab obstacles for RRT run
obstacles = obstacles_1()

# Define parameters for RRT algo
dstep = 1
Nmax  = 10000

visual_type = 2

######################################################################
#
#   RRT Functions
#
def RRT(tree, goalstate, Nmax):
    # Loop.
    while True:
        # Determine the target state. 5% of time go for goal line
        target_coin = random.uniform(0, 1)
        if target_coin <= 0.05:
            # Go for the goal line. Start by finding the
            # closest node to the goal line in x, y space.
            # We will determine arrival time later.
            targetstate = goalstate
            list = [(node.state.XY_DistSquared(targetstate), node) for node in tree]
        else:
            # Go for a random node.
            x = random.uniform(xmin, xmax)
            y = random.uniform(ymin, ymax)
            t = random.uniform(0, tmax)
            targetstate = State(x, y, t)
            # Find the closest node to the new node
            # in x, y, t space
            list = [(node.state.XYT_DistSquared(targetstate), node) for node in tree]

        # Get nearest node to target, and distance between the two
        (d2, nearnode)  = min(list)
        d = np.sqrt(d2)
        nearstate = nearnode.state

        # If going for goal line, we need to determine the soonest possible time
        # we can reach the goal line from our nearest state. This is related to
        # vmax.
        if targetstate == goalstate:
            targetstate = nearstate.ClosestStateOnGoalLine(goalstate, tmax, vmax)

        # Determine the next state, a step size (dstep) away
        alpha = min(dstep / np.sqrt(nearstate.XY_DistSquared(targetstate)), 1)
        nextstate = nearstate.Intermediate(targetstate, alpha)

        # Check whether to attach (creating a new node).
        if nearstate.ConnectsTo(nextstate, vmax, obstacles):
            nextnode = Node(nextstate, nearnode, nearnode.viz, nearnode.viz_type)
            tree.append(nextnode)

            # Also try to connect the goal.
            if np.sqrt(nextstate.XY_DistSquared(goalstate)) < dstep:
                result = nextstate.ConnectsToGoal(goalstate, tmax, vmax, obstacles)
                if result[0]:
                    goalnode = Node(result[1], nextnode, nearnode.viz, nearnode.viz_type)
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

    # Create the start/goal nodes.
    startstate = State(startx, starty, 0)
    goalstate  = State(goalx,  goaly, tmax)

    if visual_type == 2:
        Visual = Visualization_2D(xmin, xmax, ymin, ymax)
        Visual.DrawState(startstate, 'ro')
        Visual.DrawState(goalstate,  'ro')
        Visual.ShowFigure()
    elif visual_type == 3:
        Visual = Visualization_3D((startx, starty), (goalx, goaly), tmax)
        Visual.DrawConfigSpace(obstacles)
    input("Showing basic world (hit return to continue)")

    # Start the tree with the start state and no parent.
    tree = [Node(startstate, None, Visual, visual_type)]

    # Execute the search (return the goal leaf node).
    node = RRT(tree, goalstate, Nmax)
    print("Reached goal in {} seconds".format(node.state.t))

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
    Visual.DrawDiscretePath(startstate, goalstate, path, obstacles)

    # Collect and show a continuous time path
    Visual.DrawContinuousPath(startstate, goalstate, path, obstacles)

    input("Showing the raw path (hit return to continue)")

if __name__== "__main__":
    for obs in obstacles:
        print(obs.get_position(1))
    main()
