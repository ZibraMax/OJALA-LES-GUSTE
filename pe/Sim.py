import time
from .Region import Region, Event
from .Solvers import *
from .Quadtree import *
from .Colliders import *
from .Utils import point_point_distance as p2p
import numpy as np
import os


class Force(object):
    """Creates a force

    Args:
        expresion (Callable): Force function. Functions can evaluate time and Node. Node has a parent attribute
        domain (Union[list, np.ndarray], optional): Domain of the force functon. Defaults to None.
    """

    def __init__(self, expresion: Callable, domain: Union[list, np.ndarray] = None) -> None:
        """Creates a force

        Args:
            expresion (Callable): Force function. Functions can evaluate time and Node. Node has a parent attribute
            domain (Union[list, np.ndarray], optional): Domain of the force functon. Defaults to None.
        """
        self.domain = domain
        self.function = expresion
        if not domain:
            self.domain = [-np.inf, np.inf]

    def __call__(self, t: float, obj: 'Node') -> np.ndarray:
        """When the functions is called

        Args:
            t (float): Time
            obj (Node): Node which the load is applied

        Returns:
            np.ndarray: Force value in x and y
        """
        if (t > self.domain[0] and t < self.domain[1]):
            return self.function(t, obj)
        return 0.0


class Sim():
    """Creates a simulation object

    Args:
        x_range (Union[list, np.ndarray]): X range of canvas coordinates. Must be positive
        y_range (Union[list, np.ndarray]): Y range of canvas coordinates. Must be positive
        dt (float, optional): Time step. Defaults to 1/500.
    """

    def __init__(self, x_range: Union[list, np.ndarray], y_range: Union[list, np.ndarray], dt: float = 1/500, frames_folder=None) -> None:
        """Creates a simulation object

        Args:
            x_range (Union[list, np.ndarray]): X range of canvas coordinates. Must be positive
            y_range (Union[list, np.ndarray]): Y range of canvas coordinates. Must be positive
            dt (float, optional): Time step. Defaults to 1/500.
        """

        self.region = Region(x_range, y_range)
        W = x_range[-1]-x_range[0]
        H = y_range[-1]-y_range[0]
        self.boundary = Quadrant(((x_range[-1]+x_range[0])/2,
                                  (y_range[-1]+y_range[0])/2), (W/2, H/2))
        self.QuadTree = QuadTree(self.boundary)
        self.nodes = []
        self.colliders = []
        self.t = 0.0
        self.max_radius = -np.inf
        self.region.canvas.bind('<Button-1>', self.click)
        self.region.canvas.bind('<Button-2>', self.middle_click)
        self.region.canvas.bind('<Button-3>', self.right_click)
        self.region.root.bind('<KeyRelease>', self.keyup)
        self.region.root.bind('<MouseWheel>', self.wheel)
        self.region.canvas.bind('<Motion>', self.move)
        self.gen_forces = []
        self.gen_forces_mult = 1.0
        self.springs = []
        self.adding_spring = False
        self.pause = True
        self.nearI = None
        self.nearF = None
        self.dt = dt
        self.fps = 60
        self.adding_line = True
        self.moving_mass = False
        self.move_flag = False
        self.frame = 0
        self.frames_folder = frames_folder
        self.draw_quad_tree_flag = False
        if frames_folder:
            try:
                os.mkdir(frames_folder)
            except Exception as e:
                pass

    def info_text(self) -> None:
        """Place information text in canvas
        """
        deltax = self.region.xrange[-1] - self.region.xrange[0]
        deltay = self.region.yrange[-1] - self.region.yrange[0]
        self.region.create_text(
            [self.region.xrange[0]+deltax*0.1, self.region.yrange[0]+deltay*0.1], f'dt={self.dt:.5f}')

        if self.adding_line:
            self.region.create_text(
                [self.region.xrange[0]+deltax*0.1, self.region.yrange[0]+deltay*0.92], f'Poniendo barrera, oprima la tecla L para poner un resorte.')
            if self.adding_spring:
                self.region.create_text(
                    [self.region.xrange[0]+deltax*0.1, self.region.yrange[0]+deltay*0.9], f'Haga click derecho en un segundo sitio para agregar una barrera.')
            else:
                self.region.create_text(
                    [self.region.xrange[0]+deltax*0.1, self.region.yrange[0]+deltay*0.9], f'Haga click derecho en cualquier sitio para poner una barrera.')
        else:
            self.region.create_text(
                [self.region.xrange[0]+deltax*0.1, self.region.yrange[0]+deltay*0.92], f'Poniendo resorte, oprima la tecla L para poner una barrera.')

            if self.adding_spring:
                self.region.create_text(
                    [self.region.xrange[0]+deltax*0.1, self.region.yrange[0]+deltay*0.9], f'Haga click derecho en el segundo nodo para poner el resorte.')
            else:
                self.region.create_text(
                    [self.region.xrange[0]+deltax*0.1, self.region.yrange[0]+deltay*0.9], f'Haga click derecho en un nodo para poner un resorte.')
        if self.moving_mass:
            self.region.create_text(
                [self.region.xrange[0]+deltax*0.1, self.region.yrange[0]+deltay*0.88], f'Haga click izquierdo para mover una pelota.')
        else:
            self.region.create_text(
                [self.region.xrange[0]+deltax*0.1, self.region.yrange[0]+deltay*0.88], f'Haga click izquierdo para agregar una pelota.')
        self.region.create_text(
            [self.region.xrange[0]+deltax*0.1, self.region.yrange[0]+deltay*0.86], f'Haga click de rueda de ratón para agregar una pelota fija.')
        self.region.create_text(
            [self.region.xrange[0]+deltax*0.1, self.region.yrange[0]+deltay*0.84], f'Presione G para activar y desactivar las físicas.')

        if self.pause:
            self.region.create_text(
                [self.region.xrange[0]+deltax*0.1, self.region.yrange[0]+deltay*0.82], f'La simulación se encuentra pausada, presione p para despausar.')
        else:
            self.region.create_text(
                [self.region.xrange[0]+deltax*0.1, self.region.yrange[0]+deltay*0.82], f'La simulación se encuentra coriendo, t={self.t:.5f}. fps={self.fps:.2f}')
        self.region.create_text(
            [self.region.xrange[0]+deltax*0.1, self.region.yrange[0]+deltay*0.8], f'Con la rueda del mouse se puede aumentar o disminuir el dt')

    def wheel(self, event: Event) -> None:
        """Event when the mouse wheel turns

        Args:
            event (Event): Mouse wheel event
        """

        delta = event.delta
        self.dt += 0.00005*np.sign(delta)
        self.dt = max(self.dt, 0.00001)

    def keyup(self, event: Event) -> None:
        """Event called when a key is presseed

        Args:
            event (Event): KeyPress event
        """
        if event.char.lower() == 'g':
            self.gen_forces_trigger()
        elif event.char.lower() == 'p':
            self.pause_trigger()
        elif event.char.lower() == 'l':
            self.adding_line = not self.adding_line
        elif event.char.lower() == 'm':
            self.moving_mass = not self.moving_mass
        elif event.char.lower() == 't':
            self.draw_quad_tree_flag = not self.draw_quad_tree_flag
        self.update_graphics()

    def gen_forces_trigger(self) -> None:
        """Trigger the forces multiplier (1 to 0 or 0 to 1)
        """
        self.gen_forces_mult = 1.0 - self.gen_forces_mult

    def pause_trigger(self) -> None:
        """Pauses or resume the simulation
        """
        if self.pause:
            self.pause = not self.pause
            self.run()
        else:
            self.pause = not self.pause

    def add_spring(self, spring: 'Spring') -> None:
        """Adds a spring to the simulation

        Args:
            spring (Spring): Spring to be added
        """
        self.springs.append(spring)

    def nearest_node(self, X: Union[list, np.ndarray]) -> int:
        """Find the nearest node to the coordinates X

        Args:
            X (Union[list, np.ndarray]): X coordinates to find the nearest node

        Returns:
            int: Nearest node index
        """
        mini = np.inf
        selected = None
        for i, node in enumerate(self.nodes):
            d = p2p(X, node.U)
            if d < mini:
                mini = d
                selected = i
        return selected

    def middle_click(self, event: Event) -> None:
        """Middle click event

        Args:
            event (Event): Middle click event
        """

        X = np.array([event.x, event.y])
        X = self.region._coords_transform(X)
        masa = 0.5
        if len(self.nodes) > 0:
            masa = self.nodes[-1].m
        node = Node(masa, X, [0.0, 0.0])
        node.fix()
        self.add_node(node)
        self.update_graphics()

    def right_click(self, event: Event) -> None:
        """Right click event

        Args:
            event (Event): Right click event
        """
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

    def update_graphics(self) -> None:
        """Draws all simulation elements in the current region
        """
        self.region.delete_all()
        self.draw_colliders()
        self.draw_springs()
        self.draw_nodes()
        if self.draw_quad_tree_flag:
            self.draw_quad_tree()
        self.info_text()
        self.region.update()

    def draw_quad_tree(self):
        self.QuadTree.draw(self.region)

    def move(self, event: Event) -> None:
        """Mouse move event

        Args:
            event (Event): Mouse move event
        """

        if self.moving_mass and self.move_flag:
            X = np.array([event.x, event.y])
            X = self.region._coords_transform(X)
            self.nodes[self.nearI].U = X
            self.nodes[self.nearI].V = np.array([0.0, 0.0])
            # self.update_graphics()

    def click(self, event: Event) -> None:
        """Click event

        Args:
            event (Event): Click event
        """
        X = np.array([event.x, event.y])
        X = self.region._coords_transform(X)
        if self.moving_mass:
            if not self.move_flag:
                nodeii = self.nearest_node(X)
                self.nearI = nodeii
                self.move_flag = True
            else:
                self.move_flag = False

        else:
            masa = 0.5
            if len(self.nodes) > 0:
                masa = self.nodes[-1].m
            node = Node(masa, X, [0.0, 0.0])
            self.add_node(node)
        self.update_graphics()

    def add_node(self, node: 'Node') -> None:
        """Adds a node to the simulation

        Args:
            node (Node): Node to add
        """
        if node.r > self.max_radius:
            self.max_radius = node.r
        for force in self.gen_forces:
            node.add_force(force)
        node.parent = self
        node.id = len(self.nodes)
        self.nodes.append(node)

    def add_collider(self, collider: LinealCollider) -> None:
        """Adds a Lineal Collider to the simulation

        Args:
            collider (LinealCollider): Lineal collider object to add
        """
        self.colliders.append(collider)

    def update(self) -> None:
        """Runs a simulation frame
        """
        clock_start = time.time()
        self.QuadTree = QuadTree(self.boundary)
        for node in self.nodes:
            self.QuadTree.add_point(node)

        if not self.adding_spring:
            for spring in self.springs:
                spring.move()
            for i, node in enumerate(self.nodes):
                if self.moving_mass and self.move_flag and self.nearI == i:
                    pass
                else:
                    node.move(self.t, self.dt)
                node.collide(self.colliders)

            self.update_graphics()
            self.t += self.dt
            if self.t >= self.frame*1/60 and self.frames_folder:
                self.region.image(f'{self.frames_folder}/f_{self.frame}.png')
                self.frame += 1
        clock_end = time.time()
        self.fps = 1/(clock_end-clock_start)
        # time.sleep(1)

    def draw_springs(self) -> None:
        """Draws all springs in simulation
        """
        for spring in self.springs:
            spring.draw(self.region)

    def draw_nodes(self) -> None:
        """Draws all nodes in simulation
        """
        for node in self.nodes:
            node.draw(self.region)

    def draw_colliders(self) -> None:
        """Draws all colliders in simulation
        """
        for collider in self.colliders:
            collider.draw(self.region)

    def add_gen_force(self, force: Union[Force, Callable]) -> None:
        """Add a general force to the simulation. This force will be applied to all nodes

        Args:
            force (Union[Force, Callable]): Force to be applied
        """
        self.gen_forces.append(force)

    def run(self) -> None:
        """Runs the simulation
        """
        self.update_graphics()

        def f():
            while True and not self.adding_spring and not self.pause:
                self.update()
        self.region.root.after(1, f)
        self.region.run()


class Node():
    """Creates a node

    Args:
        m (float): Node mass
        U (Union[list, np.ndarray]): Node initial position
        V (Union[list, np.ndarray]): Node initial velocity
        solver (Solver, optional): Solver to be used. Defaults to None.
        r (float, optional): Node radius. Defaults to None.
    """

    def __init__(self, m: float, U: Union[list, np.ndarray], V: Union[list, np.ndarray], solver: Solver = None, r: float = None, trace=0) -> None:
        """Creates a node

        Args:
            m (float): Node mass
            U (Union[list, np.ndarray]): Node initial position
            V (Union[list, np.ndarray]): Node initial velocity
            solver (Solver, optional): Solver to be used. Defaults to None.
            r (float, optional): Node radius. Defaults to None.
        """
        self.U = np.array(U)
        self.m = m
        self.V = np.array(V)
        self.forces = []
        self.solver = solver
        self.parent = None
        self.id = None
        self.fixed = False
        self.trace = trace
        self.Uh = []
        if not solver:
            self.solver = Solver()
        self.r = r
        if not r:
            self.r = self.m/20

    def object_collide(self) -> None:
        """Object object collision
        """
        if not self.fixed:
            ran = Quadrant(self.U, (2*self.parent.max_radius,
                           2*self.parent.max_radius))
            collision_nodes = self.parent.QuadTree.query_range(ran)
            for i, o in enumerate(collision_nodes):
                if not o.id == self.id:
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
                        delta1 = [dot1 * (x1[0] - x2[0]),
                                  dot1 * (x1[1] - x2[1])]
                        delta2 = [dot2 * (x2[0] - x1[0]),
                                  dot2 * (x2[1] - x1[1])]
                        if o.fixed:
                            self.V -= 2 * \
                                np.dot(self.V, np.array(
                                    [dx, dy]))*np.array([dx, dy])/dist/dist
                        else:
                            self.V[0] -= (1-self.fixed)*delta1[0]
                            self.V[1] -= (1-self.fixed)*delta1[1]
                            o.V[0] -= delta2[0]
                            o.V[1] -= delta2[1]

                        self.U += (1-self.fixed)*(dist-minDist) * \
                            np.array([dx, dy])/dist

    def collide(self, colliders: List[LinealCollider]) -> None:
        """Makes the node collide with the colliders given by parameters

        Args:
            colliders (List[LinealCollider]): Colliders
        """
        self.object_collide()
        for collider in colliders:
            collider.callback(self)

    def add_force(self, f: Union[Force, Callable]) -> None:
        """Adds a force to the node

        Args:
            f (Union[Force,Callable]): Function to be added
        """
        self.forces.append(f)

    def move(self, t: float, dt: float) -> None:
        """Runs a simulation frem over the node

        Args:
            t (float): Time
            dt (float): Delta time
        """
        if not self.fixed:

            sumatoria = 0.0
            for g in self.forces:
                sumatoria += g(t, self)*self.parent.gen_forces_mult
            sumatoria = sumatoria/self.m
            self.V = self.V+sumatoria*dt
            self.U += self.V*dt
            if self.trace:
                self.Uh += [self.U.tolist()]

    def fix(self) -> None:
        """Locks the node movement
        """
        self.fixed = not self.fixed

    def draw(self, region: Region) -> None:
        """Draws the current node

        Args:
            region (Region): Canvas drawing region
        """
        if self.fixed:
            region.create_circle(self.U, self.r, color='red')
        else:
            region.create_circle(self.U, self.r, color='blue')
        if self.trace:
            if self.trace == -1:
                n = len(self.Uh)
            else:
                n = min(self.trace, len(self.Uh))
            for i in range(1, n):
                region.create_line(np.array(self.Uh[-i]), np.array(self.Uh[-(i+1)]),
                                   color='gray', width=3)


class Spring():
    """Creates a spring between 2 nodes

    Args:
        k (float): Spring constant
        d (float): Node damping
        nodeI (Node): Initial node
        nodeF (Node): End node

    """

    def __init__(self, k: float, d: float, nodeI: Node, nodeF: Node) -> None:
        """Creates a spring between 2 nodes

        Args:
            k (float): Spring constant
            d (float): Node damping
            nodeI (Node): Initial node
            nodeF (Node): End node

        """
        self.nodeI = nodeI
        self.nodeF = nodeF
        self.dx = nodeF.U-nodeI.U
        self.l = np.linalg.norm(self.dx)
        self.s = 0.0
        self.dir = self.dx/self.l
        self.d = d
        self.k = k
        self.color = 'green'

        def fi(t, obj):
            return self.dir*(self.s*self.k+self.d*np.dot(self.nodeF.V-self.nodeI.V, self.dir))

        def ff(t, obj):
            return -self.dir*(self.s*self.k+self.d*np.dot(self.nodeF.V-self.nodeI.V, self.dir))

        self.nodeI.add_force(fi)
        self.nodeF.add_force(ff)

    def move(self) -> None:
        """Updates spring position and forces
        """
        self.dx = self.nodeF.U-self.nodeI.U
        ld = np.linalg.norm(self.dx)
        self.s = ld-self.l
        self.dir = self.dx/ld

    def draw(self, region: Region) -> None:
        """Draws the spring

        Args:
            region (Region): Canvas drawing region
        """
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
