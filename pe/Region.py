import math
from tkinter import Tk, Canvas, W, E, NW
import time
import numpy as np


class Region():
    """Creates the canvas drawing region
    """

    def __init__(self):

        self.root = Tk()
        self.root.title('TEST')
        self.width = self.root.winfo_screenwidth()
        self.height = self.root.winfo_screenheight()
        self.canvas = Canvas(self.root, width=self.width,
                             height=self.height, background='white')
        self.canvas.grid(row=0, column=0)
        self.root.state('zoomed')
        self.canvas.bind('<Button-1>', self.click)

    def click(self, event):
        """Custom event when click ocurs

        Args:
            event (event): Tkinter canvas event of click. event.x and event.y are useful
        """
        self.draw_circle([event.x, event.y], 20)

    def update(self):
        """Update the region
        """
        self.canvas.update()

    def delete_all(self):
        """Cleans the current drawing region
        """
        self.canvas.delete('all')

    def run(self):
        """Starts the region
        """
        self.root.mainloop()

    def create_circle(self, X, r, color='black', **kargs):
        """Draws a circle in the specified

        Args:
            X (list): Center of the circle [x,y]
            r (float): radius of the circle
            color (str, optional): Color of the circle. Defaults to 'black'.
            **kargs (args): Another args for the create_oval canvas method
        """
        x = X[0]
        y = X[1]
        self.canvas.create_oval(x-r, y-r, x+r, y+r, fill=color, **kargs)
        self.update()

    def create_line(self, X0, XF, color='black', **kargs):
        """Draws a line in the specified coords

        Args:
            X0 (list): start coords of the line
            XF (list): end coords of the line
            color (str, optional): Color of the line segment. Defaults to 'black'.
            **kargs (args): Another args for the create_line canvas method

        """
        self.canvas.create_line(X0[0], X0[1], XF[1],
                                XF[0], fill=color, **kargs)
        self.update()


if __name__ == '__main__':
    def main():
        region = Region()
        region.create_circle([1067, 429], 20)
        region.run()
    main()
