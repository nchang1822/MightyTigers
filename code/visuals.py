import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

######################################################################
#
#   Visualization
#
class Visualization_2D:
    def __init__(self, xmin, xmax, ymin, ymax):
        self.xmin = xmin
        self.xmax = xmax
        self.ymin = ymin
        self.ymax = ymax

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
        plt.gca().set_xlim(self.xmin, self.xmax)
        plt.gca().set_ylim(self.ymin, self.ymax)
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

    def DrawDiscretePath(self, start, goal, path, obstacles):
        for i in range(len(path)):
            self.ClearFigure()
            self.DrawState(start, 'bo')
            self.DrawState(goal,  'bo')

            self.DrawState(path[i].state, 'ro', linewidth=1)

            for obstacle in obstacles:
                self.DrawObstacle(obstacle, path[i].state.t)

            self.ShowFigure()
            input("Showing the PATH(hit return to continue)")

    def DrawContinuousPath(self, start, goal, path, obstacles):
        # Collect and show a continuous time path
        full_viz_states = []
        for i in range(len(path) - 1):
            dt = .05
            diff = path[i+1].state.t - path[i].state.t
            for j in range(int(diff/dt)):
                full_viz_states.append(path[i].state.Intermediate(path[i + 1].state, j * dt/diff))
        full_viz_states.append(path[-1].state)

        for s in full_viz_states:
            self.ClearFigure()
            self.DrawState(start, 'bo')
            self.DrawState(goal,  'bo')
            self.DrawState(s, 'ro', linewidth=1)
            for obstacle in obstacles:
                self.DrawObstacle(obstacle, s.t)
            plt.title("Time: {}".format(round(s.t, 1)))
            self.ShowFigure()

class Visualization_3D:
    def __init__(self, start, goal, tmax):
        self.fig = plt.figure()
        self.ax = plt.axes(projection="3d")

        self.start = start
        self.goal = goal

        self.tmax = tmax
        self.sample_num = 1000

    def ClearFigure(self):
        # Clear the current, or create a new figure.
        plt.clf()

        # Create a new axes, enable the grid, and set axis limits.
        self.ax = plt.axes(projection="3d")

    def ShowFigure(self):
        # Show the plot.
        plt.pause(0.001)

    def DrawState(self, state, *args, **kwargs):
        return self.ax.scatter3D(state.x, state.y, state.t, color="r")

    def DrawObstacle(self, obstacle):
        tline = np.linspace(0, self.tmax, self.sample_num)

        obst_xf = np.vectorize(obstacle.fx)
        obst_yf = np.vectorize(obstacle.fy)

        xline = obst_xf(tline)
        yline = obst_yf(tline)
        self.ax.scatter3D(xline, yline, tline, s=obstacle.r)

    def DrawGoalLine(self):
        self.ax.scatter3D(
            self.goal[0] * np.ones(self.sample_num),
            self.goal[1] * np.ones(self.sample_num),
            np.linspace(0, self.tmax, self.sample_num))

    def DrawConfigSpace(self, obstacles):
        self.ax.scatter3D(self.start[0], self.start[1], 0)
        self.DrawGoalLine()

        for obstacle in obstacles:
            self.DrawObstacle(obstacle)

        plt.pause(0.001)

    def DrawDiscretePath(self, start, goal, path, obstacles):
        self.DrawState(start, 'bo')
        self.DrawGoalLine()

        for obstacle in obstacles:
            self.DrawObstacle(obstacle)

        for i in range(len(path)):
            self.DrawState(path[i].state, 'ro', linewidth=1)
            self.ShowFigure()
            input("Showing the PATH(hit return to continue)")

        self.ClearFigure()

    def DrawContinuousPath(self, start, goal, path, obstacles):
        # Collect and show a continuous time path
        full_viz_states = []
        for i in range(len(path) - 1):
            dt = .05
            diff = path[i+1].state.t - path[i].state.t
            for j in range(int(diff/dt)):
                full_viz_states.append(path[i].state.Intermediate(path[i + 1].state, j * dt/diff))
        full_viz_states.append(path[-1].state)

        self.DrawState(start, 'bo')
        self.DrawGoalLine()

        for obstacle in obstacles:
            self.DrawObstacle(obstacle)

        p = None
        for s in full_viz_states:
            if p != None:
                p.remove()

            p = self.DrawState(s, 'ro', linewidth=1)
            self.ShowFigure()

######################################################################
#
#   Visualization
#
class Visualization_astar:
    def __init__(self, xmin, xmax, ymin, ymax, tmax):
        self.xmin = xmin
        self.xmax = xmax
        self.ymin = ymin
        self.ymax = ymax
        self.tmax = tmax
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
        plt.gca().set_xlim(self.xmin, self.xmax)
        plt.gca().set_ylim(self.ymin, self.ymax)
        plt.gca().set_aspect('equal')


    def ShowFigure(self):
        # Show the plot.
        plt.pause(0.001)


    def DrawState(self, state, *args, **kwargs):
        plt.plot(state.x, state.y, color = (state.t/self.tmax, 0.1, 0.1), *args, **kwargs)

    def DrawLocalPath(self, head, tail, *args, **kwargs):
        plt.plot((head.x, tail.x),
                 (head.y, tail.y), *args, **kwargs)
        # plt.quiver(
        #     head.x, head.y,
        #     tail.x - head.x,
        #     tail.y - head.y,
        #     scale_units='xy', angles='xy', scale=1,width = 0.001, headwidth = 100, color='g')

    def DrawObstacle(self, obstacle, t, *args, **kwargs):
        x, y = obstacle.get_position(t)
        circ = plt.Circle((x, y), obstacle.r)
        plt.gca().add_patch(circ)
