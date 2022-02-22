import numpy as np
import matplotlib.pyplot as plt

class Obstacle:
    '''
    Class for a 1x1 moving obstacle
    '''
    def __init__(self, row, col, drow, dcol, walls):
        self.row = row
        self.col = col
        self.drow = drow
        self.dcol = dcol
        self.walls = walls
    def update(self):
        # Move the obstacle in its current direction
        row = self.row + self.drow
        col = self.col + self.dcol
        if not self.walls[row, col]:
            self.row = row
            self.col = col
        else:
            # Bouce off wall and change direction
            self.drow = self.drow * -1
            self.dcol = self.dcol * -1

#
#   Probailiity Grid Visualization
#
class Visualization():
    def __init__(self, walls, obstacles):
        # Save the walls and determine the rows/cols:
        self.walls = walls
        # Array of obstacle objects
        self.obstacles = obstacles
        self.spots = np.sum(np.logical_not(walls))
        self.rows  = np.size(walls, axis=0)
        self.cols  = np.size(walls, axis=1)


        # Clear the current, or create a new figure.
        plt.clf()

        # Create a new axes, enable the grid, and set axis limits.
        plt.axes()
        plt.grid(False)
        plt.gca().axis('off')
        plt.gca().set_aspect('equal')
        plt.gca().set_xlim(0, self.cols)
        plt.gca().set_ylim(self.rows, 0)

        # Add the row/col numbers.
        for row in range(0, self.rows, 2):
            plt.gca().text(         -0.3, 0.5+row, '%d'%row,
                           verticalalignment='center',
                           horizontalalignment='right')
        for row in range(1, self.rows, 2):
            plt.gca().text(self.cols+0.3, 0.5+row, '%d'%row,
                           verticalalignment='center',
                           horizontalalignment='left')
        for col in range(0, self.cols, 2):
            plt.gca().text(0.5+col,          -0.3, '%d'%col,
                           verticalalignment='bottom',
                           horizontalalignment='center')
        for col in range(1, self.cols, 2):
            plt.gca().text(0.5+col, self.rows+0.3, '%d'%col,
                           verticalalignment='top',
                           horizontalalignment='center')

        # Draw the grid, zorder 1 means draw after zorder 0 elements.
        for row in range(self.rows+1):
            plt.gca().axhline(row, lw=1, color='k', zorder=1)
        for col in range(self.cols+1):
            plt.gca().axvline(col, lw=1, color='k', zorder=1)

        # Clear the content and mark.  Then show with zeros.
        self.content = None
        self.mark    = None
        self.Show(np.zeros((self.rows, self.cols)))

    def Flush(self):
        # Show the plot.
        plt.pause(0.001)

    def Mark(self, row, col):
        # Check the row/col arguments.
        assert (row >= 0) and (row < self.rows), "Illegal row"
        assert (col >= 0) and (col < self.cols), "Illegal col"

        # Potentially remove the previous mark.
        if self.mark is not None:
            self.mark.remove()
            self.mark = None

        # Draw the mark.
        self.mark  = plt.gca().text(0.5+col, 0.5+row, 'x', color = 'green',
                                    verticalalignment='center',
                                    horizontalalignment='center',
                                    zorder=1)

    def Grid(self):
        # Potentially remove the previous grid/content.
        if self.content is not None:
            self.content.remove()
            self.content = None

        # Create the color range.  There are clearly more elegant ways...
        color = np.ones((self.rows, self.cols, 3))
        for row in range(self.rows):
            for col in range(self.cols):
                if self.walls[row,col]:
                    color[row,col,0:3] = np.array([0.0, 0.0, 0.0])   # Black

        for obs in self.obstacles:
            color[obs.row,obs.col,0:3] = np.array([0.0, 0.0, 1.0]) # Blue

        # Draw the boxes.
        self.content = plt.gca().imshow(color,
                                        aspect='equal',
                                        interpolation='none',
                                        extent=[0, self.cols, self.rows, 0],
                                        zorder=0)

    def Show(self, pos = None):
        # Update the content.
        self.Grid()

        # Potentially add the mark.
        if pos is not None:
            self.Mark(pos[0], pos[1])

        # Flush the figure.
        self.Flush()
