from tkinter import Event, Tk, Canvas, W, E, NW
from typing import Union
import numpy as np

from PIL import ImageGrab


class Region():
    """Creates the canvas drawing region

        Args:
            x_range (Union[list, np.ndarray]): X range of canvas coordinates. Must be positive
            y_range (Union[list, np.ndarray]): Y range of canvas coordinates. Must be positive
        """

    def __init__(self, x_range: Union[list, np.ndarray], y_range: Union[list, np.ndarray]) -> None:
        """Creates the canvas drawing region

        Args:
            x_range (Union[list, np.ndarray]): X range of canvas coordinates. Must be positive
            y_range (Union[list, np.ndarray]): Y range of canvas coordinates. Must be positive
        """

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
        self.root.state('zoomed')
        self.canvas.bind('<Button-1>', self.click)

    def image(self, filepath):
        x = self.root.winfo_rootx()+self.canvas.winfo_x()
        y = self.root.winfo_rooty()+self.canvas.winfo_y()
        x1 = x+self.canvas.winfo_width()
        y1 = y+self.canvas.winfo_height()
        ImageGrab.grab().crop((x, y, x1, y1)).save(filepath)

    def coords_transform(self, X: Union[list, np.ndarray]) -> np.ndarray:
        """Transform from canvas coordinates to pixel coordinates

        Args:
            X (Union[list, np.ndarray]): Canvas coordinates

        Returns:
            np.ndarray: Pixel coordinates related to canvas coordinates
        """
        X = np.array(X)
        X *= self.mult
        X[-1] = self.height-X[-1]
        return X

    def _coords_transform(self, X: Union[list, np.ndarray]) -> np.ndarray:
        """Transforms from pixel coordinates to canvas coordinates

        Args:
            X (Union[list, np.ndarray]): Pixel coordinates

        Returns:
            np.ndarray: Canvas coordinates related to pixel coordinates
        """
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

    def create_text(self, X: Union[list, np.ndarray], text: str) -> None:
        """Creates a text in the canvas

        Args:
            X (Union[list, np.ndarray]): Text position in canvas coordintaes (not pixels)
            text (str): Text to be placed
        """
        x = self.coords_transform(X)
        self.canvas.create_text(x[0], x[1], fill="black",
                                font='20', text=text, anchor=W)

    def create_circle(self, X: Union[list, np.ndarray], r: float, color: str = 'black', **kargs) -> None:
        """Draws a circle in the specified

        Args:
            X (Union[list, np.ndarray]): Center of the circle [x,y]
            r (float): radius of the circle
            color (str, optional): Color of the circle. Defaults to 'black'.
            **kargs (args): Another args for the create_oval canvas method
        """
        rA = r*self.mult
        XA = self.coords_transform(X)
        x = XA[0]
        y = XA[1]
        self.canvas.create_oval(x-rA, y-rA, x+rA, y+rA, fill=color, **kargs)

    def create_line(self, x0: Union[list, np.ndarray], xf: Union[list, np.ndarray], color: str = 'black', **kargs) -> None:
        """Draws a line in the specified coords

        Args:
            x0 (: Union[list, np.ndarray]): start coords of the line
            xf (: Union[list, np.ndarray]): end coords of the line
            color (str, optional): Color of the line segment. Defaults to 'black'.
            **kargs (args): Another args for the create_line canvas method

        """
        X0 = self.coords_transform(x0)
        XF = self.coords_transform(xf)
        self.canvas.create_line(X0[0], X0[1], XF[0],
                                XF[1], fill=color, **kargs)


if __name__ == '__main__':
    def main():
        region = Region([0, 1], [0, 1])
        region.create_circle([0.5, 0.25], 0.25)
        region.run()
    main()
