from numpy.lib.function_base import append
from Region import Region
import numpy as np
import time


class Sim():
    """docstring for SIM
    """

    def __init__(self, x_range, y_range):
        self.region = Region(x_range, y_range)
        self.nodes = []
        self.t = 0.0

    def add_node(self, node):
        self.nodes.append(node)

    def update(self, dt):
        for node in self.nodes:
            node.move(self.t, dt)

        self.region.delete_all()
        self.draw_nodes()
        self.region.update()
        self.t += dt
        # time.sleep(1)

    def draw_nodes(self):
        for node in self.nodes:
            node.draw(self.region)

    def run(self, dt=0.001):

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


class Node():
    """docstring for Node
    """

    def __init__(self, m, U, V, solver):
        self.U = np.array(U)
        self.m = m
        self.V = np.array(V)
        self.forces = []
        self.solver = solver

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
        region.create_circle(self.U.tolist(), self.m*20/region.mult)


if __name__ == '__main__':
    def main():
        solver = Solver()

        SIM = Sim([0, 10], [0, 10])

        node = Node(0.5, [5.0, 5.0], [0.1, 0.1], solver)
        node.add_force(lambda t, obj, **kargs: np.array([0.0, -9.81*node.m]))
        SIM.add_node(node)
        SIM.run()

    main()
