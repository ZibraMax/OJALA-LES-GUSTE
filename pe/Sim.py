from numpy.lib.function_base import append
from .Region import Region
from .Solvers import *
from .Colliders import *
import numpy as np
import time


class Force(object):
    """docstring for Force
    """

    def __init__(self, expresion, domain=None):
        self.domain = domain
        self.function = expresion
        if not domain:
            self.domain = [-np.inf, np.inf]

    def __call__(self, t, obj):
        if (t > self.domain[0] and t < self.domain[1]):
            return self.function(t, obj)
        return 0.0


class Sim():
    """docstring for SIM
    """

    def __init__(self, x_range, y_range):
        self.region = Region(x_range, y_range)
        self.nodes = []
        self.colliders = []
        self.t = 0.0
        self.region.canvas.bind('<Button-1>', self.click)
        self.gen_forces = []

    def click(self, event):
        X = np.array([event.x, event.y])
        X = self.region._coords_transform(X)
        masa = 0.5
        if len(self.nodes) > 0:
            masa = self.nodes[-1].m
        node = Node(masa, X, [0.0, 0.0])
        self.add_node(node)

    def add_node(self, node):
        for force in self.gen_forces:
            node.add_force(force)
        node.parent = self
        node.id = len(self.nodes)
        self.nodes.append(node)

    def add_collider(self, collider):
        self.colliders.append(collider)

    def update(self, dt):
        self.region.delete_all()
        for node in self.nodes:
            node.move(self.t, dt)
            node.collide(self.colliders)
        self.draw_colliders()
        self.draw_nodes()
        self.region.update()
        self.t += dt
        # time.sleep(1)

    def draw_nodes(self):
        for node in self.nodes:
            node.draw(self.region)

    def draw_colliders(self):
        for collider in self.colliders:
            collider.draw(self.region)

    def add_gen_force(self, force):
        self.gen_forces.append(force)

    def run(self, dt=1/500):

        def f():
            while True:
                self.update(dt)
        self.region.root.after(1, f)
        self.region.run()


class Node():
    """docstring for Node
    """

    def __init__(self, m, U, V, solver=None, r=None):
        self.U = np.array(U)
        self.m = m
        self.V = np.array(V)
        self.forces = []
        self.solver = solver
        self.parent = None
        self.id = None
        if not solver:
            self.solver = Solver()
        self.r = r
        if not r:
            self.r = self.m/20

    def collide(self, colliders):
        for collider in colliders:
            collider.callback(self)

    def add_force(self, f):
        self.forces.append(f)

    def move(self, t, dt):
        def f(t, y):
            return self.V
        self.U = self.solver.solve(f, t, t+dt, self.U, 1)

        def f(t, y):
            sumatoria = 0.0
            for g in self.forces:
                sumatoria += g(t, self)
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
        SIM.add_collider(wall)
        wall = Wall([h, 0.0], [h, h])
        SIM.add_collider(wall)
        wall = Wall([h, h], [0.0, h])
        SIM.add_collider(wall)
        wall = Wall([0.0, h], [0.0, 0.0])
        SIM.add_collider(wall)

        node = Node(0.5, [0.5, 0.5], [1.0, 1.0], solver)
        node.add_force(lambda t, obj: np.array([0.0, -9.81*node.m]))
        SIM.add_node(node)

        SIM.run()

    main()
