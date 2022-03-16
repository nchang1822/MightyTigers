import numpy as np
import matplotlib.pyplot as plt

class State:
    def __init__(self, x, y, t):
        self.x = x
        self.y = y
        self.t = t

    def __repr__(self):
        return ("<XY %5.2f,%5.2f @ %5.1f s>" %
                (self.x, self.y, self.t))

    def InFreespace(self, obstacles):
        for obstacle in obstacles:
            xpos, ypos = obstacle.get_position(self.t)
            if np.sqrt((self.y - ypos)**2 + (self.x - xpos)**2) <= obstacle.r:
                return False
        return True

    def XY_DistSquared(self, other):
        return ((self.x - other.x)**2 + (self.y - other.y)**2)

    def XYT_DistSquared(self, other, vmax):
        return ((self.x - other.x)**2 + (self.y - other.y)**2 + (vmax**2) * (self.t - other.t)**2)

    def Intermediate(self, other, alpha):
        return State(self.x + alpha * (other.x - self.x),
                     self.y + alpha * (other.y - self.y),
                     self.t + alpha * (other.t - self.t))

    def ConnectsTo(self, other, vmax, obstacles):
        if other.t < self.t:
            return False

        if np.sqrt((self.x - other.x)**2 +(self.y - other.y)**2)/ \
                    (other.t - self.t) > vmax:
            return False

        for i in range(1,101):
            if not self.Intermediate(other, i/100).InFreespace(obstacles):
                return False
        return True

    def ClosestStateOnGoalLine(self, goalstate, tmax, vmax):
        v = vmax
        while v > 0:
            arrival_t = (np.sqrt(self.XY_DistSquared(goalstate)) / v) + self.t
            if arrival_t < tmax:
                targetstate = State(goalstate.x, goalstate.y, arrival_t)
                return targetstate
            v = v - .1
        return None

    def ConnectsToGoal(self, goalstate, tmax, vmax, obstacles):
        v = vmax
        while v > 0:
            arrival_t = (np.sqrt(self.XY_DistSquared(goalstate)) / v) + self.t
            if arrival_t < tmax:
                targetstate = State(goalstate.x, goalstate.y, arrival_t)
                valid = True
                for i in range(1,101):
                    if not self.Intermediate(targetstate, i/100).InFreespace(obstacles):
                        valid = False
                        break
                if valid:
                    return (True, targetstate)
            v = v - .1
        return (False, None)

#
#   Node class upon which to build the graph (roadmap) and which
#   supports the A* search tree.
#
class Node:
    def __init__(self, state, parentnode, viz, viz_type):
        # Save the state matching this node.
        self.state = state

        # Link to parent for the tree structure.
        self.parent = parentnode

        self.viz = viz

        self.viz_type = viz_type

        # Automatically draw.
        if self.viz_type != 0:
            self.Draw('r-', linewidth=1)

    # Draw a line to the parent.
    def Draw(self, *args, **kwargs):
        if self.parent is not None:
            # 2D Visualization
            if self.viz_type == 2:
                plt.quiver(
                    self.parent.state.x, self.parent.state.y,
                    self.state.x - self.parent.state.x,
                    self.state.y - self.parent.state.y,
                    scale_units='xy', angles='xy', scale=1, color=(self.state.t/30, 0, 0))
            elif self.viz_type == 3:
                # 3D Visualization
                self.viz.ax.quiver(
                    self.parent.state.x, self.parent.state.y, self.parent.state.t,
                    self.state.x - self.parent.state.x,
                    self.state.y - self.parent.state.y,
                    self.state.t - self.parent.state.t,
                    color=(self.state.t/30, 0, 0))
            plt.pause(0.001)
