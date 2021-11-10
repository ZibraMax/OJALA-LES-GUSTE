from typing import List, Tuple, Union
import numpy as np

from .Region import Region
from .Utils import point_to_line_distance as p2l


class LinealCollider():
    """Creates a lineal collider

        Args:
            X0 (UnionUnion[list, np.ndarray]): Initial point of collider
            XF (Union[list, np.ndarray]): End point of collider
        """

    def __init__(self, X0: Union[list, np.ndarray], XF: Union[list, np.ndarray]) -> None:
        """Creates a lineal collider

        Args:
            X0 (UnionUnion[list, np.ndarray]): Initial point of collider
            XF (Union[list, np.ndarray]): End point of collider
        """
        X0 = np.array(X0)
        XF = np.array(XF)
        self.X0 = X0
        self.XF = XF
        self.delta = XF-X0
        self.l = np.linalg.norm(self.delta)
        self.n = np.array([-self.delta[1]/self.l, self.delta[0]/self.l])
        self.color = 'black'

    def callback(self, object: 'Node') -> None:
        """Callback of the collider

        Args:
            object (Node): Node wich collide
        """
        pass

    def draw(self, region: Region) -> None:
        """Draws the lineal collider

        Args:
            region (Region): Drawing Region
        """
        region.create_line(self.X0, self.XF, color=self.color, width=10)


class Wall(LinealCollider):
    """Creates a Wall

        Args:
            x0 (List): Initial point of wall
            xf (List): End point of wall
        """

    def __init__(self, x0: List, xf: List) -> None:
        """Creates a Wall

        Args:
            x0 (List): Initial point of wall
            xf (List): End point of wall
        """
        LinealCollider.__init__(self, x0, xf)
        self.color = 'blue'

    def callback(self, object: 'Node') -> None:
        """Wall calback on collision detection

        Args:
            object (Node): Node wich collide
        """
        d, dx = p2l(object.U, self.X0, self.XF)
        if d <= object.r:
            object.V -= 2*np.dot(object.V, dx)*dx/d/d*.95
            object.U += dx/d*(d-object.r)
