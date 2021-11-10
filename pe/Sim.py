from numpy.lib.function_base import append, select
from .Region import Region
from .Solvers import *
from .Colliders import *
from .Utils import point_point_distance as p2p
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

    def __init__(self, x_range, y_range, dt=1/500):
        self.region = Region(x_range, y_range)
        self.nodes = []
        self.colliders = []
        self.t = 0.0
        self.region.canvas.bind('<Button-1>', self.click)
        self.region.canvas.bind('<Button-2>', self.middle_click)
        self.region.canvas.bind('<Button-3>', self.right_click)
        self.region.root.bind('<KeyRelease>', self.keyup)
        self.region.root.bind('<MouseWheel>', self.wheel)
        # self.region.root.bind('<Motion>', self.moving)
        self.gen_forces = []
        self.gen_forces_mult = 0.0
        self.springs = []
        self.adding_spring = False
        self.pause = False
        self.nearI = None
        self.nearF = None
        self.dt = dt
        self.adding_line = False

    def moving(self, event):
        if self.adding_spring:
            self.update_graphics()
            X = np.array([event.x, event.y])
            X = self.region._coords_transform(X)
            self.region.create_line(self.nodes[self.nearI].U, X, color='gray')

    def info_text(self):
        deltax = self.region.xrange[-1] - self.region.xrange[0]
        deltay = self.region.yrange[-1] - self.region.yrange[0]
        self.region.create_text(
            [self.region.xrange[0]+deltax*0.1, self.region.yrange[0]+deltay*0.1], f'dt={self.dt:.5f}')

    def wheel(self, event):
        delta = event.delta
        self.dt += 0.00005*np.sign(delta)
        self.dt = max(self.dt, 0.00001)

    def keyup(self, event):
        if event.char.lower() == 'g':
            self.gen_forces_trigger()
        elif event.char.lower() == 'p':
            self.pause_trigger()
        elif event.char.lower() == 'l':
            self.adding_line = not self.adding_line

    def gen_forces_trigger(self):
        self.gen_forces_mult = 1.0 - self.gen_forces_mult

    def pause_trigger(self):
        if self.pause:
            self.pause = not self.pause
            self.run()
        else:
            self.pause = not self.pause

    def add_spring(self, spring):
        self.springs.append(spring)

    def nearest_node(self, X):
        mini = np.inf
        selected = None
        for i, node in enumerate(self.nodes):
            d = p2p(X, node.U)
            if d < mini:
                mini = d
                selected = i
        return selected

    def middle_click(self, event):
        X = np.array([event.x, event.y])
        X = self.region._coords_transform(X)
        masa = 0.5
        if len(self.nodes) > 0:
            masa = self.nodes[-1].m
        node = Node(masa, X, [0.0, 0.0])
        node.fix()
        self.add_node(node)
        self.update_graphics()

    def right_click(self, event):
        X = self.region._coords_transform([event.x, event.y])
        if not self.adding_spring:
            if not self.adding_line:
                nodeii = self.nearest_node(X)
                self.nearI = nodeii
            else:
                self.nearI = X
            self.adding_spring = True
            self.update_graphics()
        else:
            if not self.adding_line:

                nodeif = self.nearest_node(X)
                k = 1000
                d = 2
                spring = Spring(
                    k, d, self.nodes[self.nearI], self.nodes[nodeif])
                self.add_spring(spring)
            else:
                wall = Wall(self.nearI, X)
                self.add_collider(wall)
            self.adding_spring = False
            self.update_graphics()
            self.run()

    def update_graphics(self):
        self.region.delete_all()
        self.draw_colliders()
        self.draw_springs()
        self.draw_nodes()
        self.info_text()
        self.region.update()

    def click(self, event):
        X = np.array([event.x, event.y])
        X = self.region._coords_transform(X)
        masa = 0.5
        if len(self.nodes) > 0:
            masa = self.nodes[-1].m
        node = Node(masa, X, [0.0, 0.0])
        self.add_node(node)
        self.update_graphics()

    def add_node(self, node):
        for force in self.gen_forces:
            node.add_force(force)
        node.parent = self
        node.id = len(self.nodes)
        self.nodes.append(node)

    def add_collider(self, collider):
        self.colliders.append(collider)

    def update(self):
        if not self.adding_spring:
            for spring in self.springs:
                spring.move()
            for node in self.nodes:
                node.move(self.t, self.dt)
                node.collide(self.colliders)

            self.update_graphics()
            self.t += self.dt
        # time.sleep(1)

    def draw_springs(self):
        for spring in self.springs:
            spring.draw(self.region)

    def draw_nodes(self):
        for node in self.nodes:
            node.draw(self.region)

    def draw_colliders(self):
        for collider in self.colliders:
            collider.draw(self.region)

    def add_gen_force(self, force):
        self.gen_forces.append(force)

    def run(self):

        def f():
            while True and not self.adding_spring and not self.pause:
                self.update()
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
        self.fixed = False
        if not solver:
            self.solver = Solver()
        self.r = r
        if not r:
            self.r = self.m/20

    def object_collide(self):
        for i, o in enumerate(self.parent.nodes):
            if not i == self.id:
                dx = o.U[0] - self.U[0]
                dy = o.U[1] - self.U[1]
                dist = (dx**2+dy**2)**0.5
                minDist = o.r + self.r
                if dist < minDist:
                    m1 = self.m
                    m2 = o.m
                    v1 = self.V
                    v2 = o.V
                    x1 = self.U
                    x2 = o.U
                    dot1 = 2 * m2 / \
                        (m1 + m2) * ((v1[0]-v2[0])*(x1[0]-x2[0]) +
                                     (v1[1]-v2[1])*(x1[1]-x2[1])) / dist / dist
                    dot2 = 2 * m1 / \
                        (m1 + m2) * ((v2[0]-v1[0])*(x2[0]-x1[0]) +
                                     (v2[1]-v1[1])*(x2[1]-x1[1])) / dist / dist
                    delta1 = [dot1 * (x1[0] - x2[0]), dot1 * (x1[1] - x2[1])]
                    delta2 = [dot2 * (x2[0] - x1[0]), dot2 * (x2[1] - x1[1])]
                    self.V[0] -= delta1[0]
                    self.V[1] -= delta1[1]
                    o.V[0] -= delta2[0]
                    o.V[1] -= delta2[1]

                    self.U += (1-self.fixed)*(dist-minDist) * \
                        np.array([dx, dy])/dist

    def collide(self, colliders):
        self.object_collide()
        for collider in colliders:
            collider.callback(self)

    def add_force(self, f):
        self.forces.append(f)

    def move(self, t, dt):
        if not self.fixed:
            def f(t, y):
                return self.V
            self.U = self.solver.solve(f, t, t+dt, self.U, 1)

            def f(t, y):
                sumatoria = 0.0
                for g in self.forces:
                    sumatoria += g(t, self)*self.parent.gen_forces_mult
                return sumatoria/self.m
            self.V = self.solver.solve(f, t, t+dt, self.V, 1)

    def fix(self):
        self.fixed = not self.fixed

    def draw(self, region):
        if self.fixed:
            region.create_circle(self.U, self.r, color='red')
        else:
            region.create_circle(self.U, self.r, color='blue')


class Spring():
    """docstring for Spring
    """

    def __init__(self, k, d, nodeI, nodeF):
        self.nodeI = nodeI
        self.nodeF = nodeF
        self.dx = nodeF.U-nodeI.U
        self.dx0 = nodeF.U-nodeI.U
        self.l = np.linalg.norm(self.dx)
        self.s = 0.0
        self.d = d
        self.k = k
        self.color = 'green'

        def fi(t, obj):
            return -self.s*self.dir*self.k-self.d*obj.V

        def ff(t, obj):
            return self.s*self.dir*self.k-self.d*obj.V

        self.nodeI.add_force(fi)
        self.nodeF.add_force(ff)

    def move(self):
        self.dx = self.nodeF.U-self.nodeI.U
        ld = np.linalg.norm(self.dx)
        self.s = self.l-ld
        self.dir = np.array([self.dx[0]/ld, self.dx[1]/ld])

    def draw(self, region):
        region.create_line(self.nodeI.U, self.nodeF.U,
                           color=self.color, width=3)


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
