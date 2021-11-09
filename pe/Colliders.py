import numpy as np
from .Utils import point_to_line_distance as p2l


class LinealCollider():
    """docstring for LinealCollider
    """

    def __init__(self, X0, XF):
        X0 = np.array(X0)
        XF = np.array(XF)
        self.X0 = X0
        self.XF = XF
        self.delta = XF-X0
        self.l = np.linalg.norm(self.delta)
        self.n = np.array([-self.delta[1]/self.l, self.delta[0]/self.l])
        self.color = 'black'

    def callback(self, object):
        pass

    def draw(self, region):
        region.create_line(self.X0, self.XF, color=self.color)


class Wall(LinealCollider):
    """docstring for Wall
    """

    def __init__(self, x0, xf):
        LinealCollider.__init__(self, x0, xf)
        self.color = 'blue'

    def callback(self, object):
        d, dx = p2l(object.U, self.X0, self.XF)
        if d <= object.r:
            object.V -= 2*np.dot(object.V, dx)*dx/d/d*0.95
