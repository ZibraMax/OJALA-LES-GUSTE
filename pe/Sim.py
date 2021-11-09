from numpy.lib.function_base import append
from Region import Region
from Utils import point_to_line_distance as p2l
import numpy as np
import time


class Sim():
    """docstring for SIM
    """

    def __init__(self, x_range, y_range):
        self.region = Region(x_range, y_range)
        self.nodes = []
        self.walls = []
        self.t = 0.0
        self.region.canvas.bind('<Button-1>', self.click)

    def click(self, event):
        X = np.array([event.x, event.y])
        X = self.region._coords_transform(X)
        node = Node(0.5, X, [0.0, 0.0])
        node.add_force(lambda t, obj, **kargs: np.array([0.0, -9.81*node.m]))
        self.add_node(node)

    def add_node(self, node):
        self.nodes.append(node)

    def add_wall(self, wall):
        self.walls.append(wall)

    def update(self, dt):
        self.region.delete_all()
        for node in self.nodes:
            node.move(self.t, dt)
            node.collide(self.walls)
        self.draw_walls()
        self.draw_nodes()
        self.region.update()
        self.t += dt
        # time.sleep(1)

    def draw_nodes(self):
        for node in self.nodes:
            node.draw(self.region)

    def draw_walls(self):
        for wall in self.walls:
            wall.draw(self.region)

    def run(self, dt=1/500):

        def f():
            while True:
                self.update(dt)
        self.region.root.after(1, f)
        self.region.run()


class Solver():
    """docstring for solver
    """

    def __init__(self):
        pass

    def solve_euler(self, f, x0, xf, y0, n):
        h = (xf-x0)/n
        xi = x0
        yi = y0
        for _ in range(n):
            yi += f(xi, yi)*h
            xi += h
        return yi

    def solve(self, *args):
        return self.solve_euler(*args)


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


class Node():
    """docstring for Node
    """

    def __init__(self, m, U, V, solver=None, r=None):
        self.U = np.array(U)
        self.m = m
        self.V = np.array(V)
        self.forces = []
        self.solver = solver
        if not solver:
            self.solver = Solver()
        self.r = r
        if not r:
            self.r = self.m/20
        print(self.r)

    def collide(self, colliders):
        for collider in colliders:
            collider.callback(self)

    def add_force(self, f):
        self.forces.append(f)

    def move(self, t, dt, **kargs):
        def f(t, y):
            return self.V
        self.U = self.solver.solve(f, t, t+dt, self.U, 1)

        def f(t, y):
            sumatoria = 0.0
            for g in self.forces:
                sumatoria += g(t, self, **kargs)
            return sumatoria/self.m
        self.V = self.solver.solve(f, t, t+dt, self.V, 1)

    def draw(self, region):
        region.create_circle(self.U, self.r)


if __name__ == '__main__':
    def main():
        solver = Solver()

        h = 1.0

        SIM = Sim([0, h], [0, h])

        wall = Wall([0.0, 0.0], [h, 0.0])
        SIM.add_wall(wall)
        wall = Wall([h, 0.0], [h, h])
        SIM.add_wall(wall)
        wall = Wall([h, h], [0.0, h])
        SIM.add_wall(wall)
        wall = Wall([0.0, h], [0.0, 0.0])
        SIM.add_wall(wall)

        node = Node(0.5, [0.5, 0.5], [1.0, 1.0], solver)
        node.add_force(lambda t, obj, **kargs: np.array([0.0, -9.81*node.m]))
        SIM.add_node(node)

        SIM.run()

    main()
