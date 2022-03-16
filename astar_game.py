import numpy as np
import math
import bisect
import random
from sklearn.neighbors import KDTree
import matplotlib.pyplot as plt
import time

from code.obstacles import *
from code.visuals import *

vmax = 10
(xmin, xmax) = (0, 14)
(ymin, ymax) = (0, 10)
tmax = 30
(startx, starty) = (0, 4)
(goalx,  goaly)  = (14, 5)
obstacles = obstacles_1()

N = 200
K = 40

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
    def __init__(self, state):
        # Save the state matching this node.
        self.state = state

        # Edges used for the graph structure (roadmap).
        self.children = []
        self.parents  = []

        # Status, edge, and costs for the A* search tree.
        self.seen        = False
        self.done        = False
        self.treeparent  = []
        self.costToReach = 0
        self.costToGoEst = math.inf
        self.cost        = self.costToReach + self.costToGoEst

    # Define the "less-than" to enable sorting by cost in A*.
    def __lt__(self, other):
        return self.cost < other.cost

    # Distance to another node, for A*, using the state distance.
    def Distance(self, other):
        return self.state.Distance(other.state)


#
#   A* Planning Algorithm
#
def AStar(nodeList, start, goal):
    # Prepare the still empty *sorted* on-deck queue.
    onDeck = []

    # Clear the search tree (for repeated searches).
    for node in nodeList:
        node.seen = False
        node.done = False

    # Begin with the start state on-deck.
    start.done        = False
    start.seen        = True
    start.treeparent  = None
    start.costToReach = 0
    start.costToGoEst = start.Distance(goal)
    start.cost        = start.costToReach + start.costToGoEst
    bisect.insort(onDeck, start)

    # Continually expand/build the search tree.
    while True:
        # Grab the next node (first on deck).
        node = onDeck.pop(0)

        # Add the children to the on-deck queue (or update)
        for child in node.children:
            # Skip if already done.
            if child.done:
                continue

            # Compute the cost to reach the child via this new path.
            costToReach = node.costToReach + node.Distance(child)

            # Just add to on-deck if not yet seen (in correct order).
            if not child.seen:
                child.seen        = True
                child.treeparent  = node
                child.costToReach = costToReach
                child.costToGoEst = child.Distance(goal)
                child.cost        = child.costToReach + child.costToGoEst
                bisect.insort(onDeck, child)
                continue

            # Skip if the previous cost was better!
            if child.costToReach <= costToReach:
                continue

            # Update the child's connection and resort the on-deck queue.
            child.treeparent  = node
            child.costToReach = costToReach
            child.cost        = child.costToReach + child.costToGoEst
            onDeck.remove(child)
            bisect.insort(onDeck, child)

        # Declare this node done.
        node.done = True

        # Check whether we have processed the goal (now done).
        if (goal.done):
            break

        # Also make sure we still have something to look at!
        if not (len(onDeck) > 0):
            return []

    # Build the path.
    path = [goal]
    while path[0].treeparent is not None:
        path.insert(0, path[0].treeparent)

    # Return the path.
    return path


######################################################################
#
#   PRM Functions
#
#
# Sample the space
#
def AddNodesToList(nodeList, N):
    while (N > 0):
        state = State(random.uniform(xmin, xmax),
                      random.uniform(ymin, ymax),
                      random.uniform(0, tmax))
        if state.InFreespace():
            nodeList.append(Node(state))
            N = N-1


#
#   Connect the nearest neighbors
#
def ConnectNearestNeighbors(nodeList, K):
    # Clear any existing neighbors.
    for node in nodeList:
        node.children = []
        node.parents  = []

    # Determine the indices for the nearest neighbors.  This also
    # reports the node itself as the closest neighbor, so add one
    # extra here and ignore the first element below.
    X   = np.array([node.state.Coordinates() for node in nodeList])
    kdt = KDTree(X)
    idx = kdt.query(X, k=(K+1), return_distance=False)

    # Add the edges (from parent to child).  Ignore the first neighbor
    # being itself.
    for i, nbrs in enumerate(idx):
        for n in nbrs[1:]:
            if nodeList[i].state.ConnectsTo(nodeList[n].state):
                nodeList[i].children.append(nodeList[n])
                nodeList[n].parents.append(nodeList[i])

#
#  Post Process the Path
#
# def PostProcess(path):
#     i = 0
#     j = 2
#     while j < len(path):
#         s1 = path[i].state
#         s2 = path[j].state
#         if s1.ConnectsTo(s2):
#             path.pop(j-1)
#         else:
#             i += 1
#             j += 1
######################################################################
#
#  Main Code
#
def main():


    # Report the parameters.
    print('Running with ', N, ' nodes and ', K, ' neighbors.')

    # Create the figure.
    Visual = Visualization_astar(xmin, xmax, ymin, ymax, tmax)

    # Create the start/goal nodes.
    startnode = Node(State(startx, starty, 0))
    goalnode  = Node(State(goalx,  goaly, tmax))

    # Show the start/goal states.
    Visual.DrawState(startnode.state, 'ro')
    Visual.DrawState(goalnode.state,  'ro')
    Visual.ShowFigure()
    input("Showing basic world (hit return to continue)")


    # Create the list of sample points.
    start = time.time()
    nodeList = []
    AddNodesToList(nodeList, N)
    print('Sampling took ', time.time() - start)

    # Show the sample states.
    for node in nodeList:
        Visual.DrawState(node.state, 'kx')
    Visual.ShowFigure()
    input("Showing the nodes (hit return to continue)")

    # Add the start/goal nodes.
    nodeList.append(startnode)
    nodeList.append(goalnode)


    # Connect to the nearest neighbors.
    start = time.time()
    ConnectNearestNeighbors(nodeList, K)
    print('Connecting took ', time.time() - start)

    # Show the neighbor connections.
    for node in nodeList:
        for child in node.children:
            Visual.DrawLocalPath(node.state, child.state, 'g-', linewidth=0.5)
    Visual.ShowFigure()
    input("Showing the full graph (hit return to continue)")


    # Run the A* planner.
    start = time.time()
    path = AStar(nodeList, startnode, goalnode)
    print('A* took ', time.time() - start)
    if not path:
        print("UNABLE TO FIND A PATH")
        return

    # Show the path.
    for i in range(len(path)):
        Visual.ClearFigure()
        Visual.DrawState(startnode.state, 'bo')
        Visual.DrawState(goalnode.state,  'bo')

        Visual.DrawState(path[i].state, 'ro', linewidth=1)

        for obstacle in obstacles:
            Visual.DrawObstacle(obstacle, path[i].state.t)

        Visual.ShowFigure()
        input("Showing the PATH(hit return to continue)")

    full_viz_states = []
    for i in range(len(path) - 1):
        dt = .1
        diff = path[i+1].state.t - path[i].state.t
        for j in range(int(diff/dt)):
            full_viz_states.append(path[i].state.Intermediate(path[i + 1].state, j * dt/diff))
    full_viz_states.append(path[-1].state)

    for s in full_viz_states:
        Visual.ClearFigure()
        Visual.DrawState(startnode.state, 'bo')
        Visual.DrawState(goalnode.state,  'bo')
        Visual.DrawState(s, 'ro', linewidth=1)
        for obstacle in obstacles:
            Visual.DrawObstacle(obstacle, s.t)
        Visual.ShowFigure()

    input("Showing the raw path (hit return to continue)")


    # Post Process the path.
    # PostProcess(path)

    # Show the post-processed path.
    # for i in range(len(path)-1):
    #     Visual.DrawLocalPath(path[i].state, path[i+1].state, 'b-', linewidth=2)
    # Visual.ShowFigure()
    # input("Showing the post-processed path (hit return to continue)")


if __name__== "__main__":
    for obs in obstacles:
        print(obs.get_position(1))
    main()
