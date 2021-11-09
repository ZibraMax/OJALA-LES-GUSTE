import math
from tkinter import Event, Tk, Canvas, W, E, NW
import time
import numpy as np


class Region():
    """Creates the canvas drawing region
    """

    def __init__(self, x_range: list[float, float], y_range: list[float, float]) -> None:

        self.root = Tk()
        self.root.title('TEST')
        self.width = self.root.winfo_screenwidth()*0.85
        self.height = self.root.winfo_screenheight()*0.85
        self.xrange = x_range
        self.yrange = y_range
        pwidth = x_range[-1]-x_range[0]
        pheight = y_range[-1]-y_range[0]
        multipl = min(pwidth, pheight)
        self.mult = min(self.width, self.height)/multipl
        self.canvas = Canvas(self.root, width=min(self.width, self.height),
                             height=min(self.width, self.height), background='white')
        self.canvas.pack()
        self.root.bind('<Escape>', lambda e: self.root.destroy())
        self.canvas.bind('<Button-1>', self.click)

    def coords_transform(self, X):
        X = np.array(X)
        X *= self.mult
        X[-1] = self.height-X[-1]
        return X

    def _coords_transform(self, X):
        X = np.array(X)
        X[-1] = self.height-X[-1]
        X = X/self.mult
        return X

    def click(self, event: Event) -> None:
        """Custom event when click ocurs

        Args:
            event (event): Tkinter canvas event of click. event.x and event.y are useful
        """
        self.draw_circle([event.x, event.y], 20)

    def update(self) -> None:
        """Update the region
        """
        self.canvas.update()
        self.root.update()

    def delete_all(self) -> None:
        """Cleans the current drawing region
        """
        self.canvas.delete('all')

    def run(self) -> None:
        """Starts the region
        """
        self.root.mainloop()

    def create_circle(self, X: list[float, float], r: float, color: str = 'black', **kargs) -> None:
        """Draws a circle in the specified

        Args:
            X (list): Center of the circle [x,y]
            r (float): radius of the circle
            color (str, optional): Color of the circle. Defaults to 'black'.
            **kargs (args): Another args for the create_oval canvas method
        """
        rA = r*self.mult
        XA = self.coords_transform(X)
        x = XA[0]
        y = XA[1]
        self.canvas.create_oval(x-rA, y-rA, x+rA, y+rA, fill=color, **kargs)

    def create_line(self, x0: list[float, float], xf: list[float, float], color: str = 'black', **kargs) -> None:
        """Draws a line in the specified coords

        Args:
            x0 (list): start coords of the line
            xf (list): end coords of the line
            color (str, optional): Color of the line segment. Defaults to 'black'.
            **kargs (args): Another args for the create_line canvas method

        """
        X0 = self.coords_transform(x0)
        XF = self.coords_transform(xf)
        self.canvas.create_line(X0[0], X0[1], XF[1],
                                XF[0], fill=color, **kargs)


if __name__ == '__main__':
    def main():
        region = Region([0, 1], [0, 1])
        region.create_circle([0.5, 0.25], 0.25)
        region.run()
    main()
