import pe
import numpy as np

h = 10.0

SIM = pe.Sim([0, h], [0, h])

wall = pe.Colliders.Wall([0.0, 0.0], [h, 0.0])
SIM.add_collider(wall)
wall = pe.Colliders.Wall([h, 0.0], [h, h])
SIM.add_collider(wall)
wall = pe.Colliders.Wall([h, h], [0.0, h])
SIM.add_collider(wall)
wall = pe.Colliders.Wall([0.0, h], [0.0, 0.0])
SIM.add_collider(wall)

G = 6.67408*10**-1


def gravity_newton(t, obj):
    f = np.array([0.0, 0.0])
    parent = obj.parent
    for i, obji in enumerate(parent.nodes):
        if not obj.id == i:
            dx = obji.U-obj.U
            d = np.linalg.norm(dx)
            dh = dx/d
            f += dh*G*obj.m*obji.m/d
    return f


force = pe.Force(gravity_newton, [-1, 5])

gravity = pe.Force(lambda t, obj: np.array([0, -9.81*obj.m]), [5, np.inf])


SIM.add_gen_force(force)
SIM.add_gen_force(gravity)
node = pe.Node(5, [5.0, 5.0], [0.0, 0.0])
SIM.add_node(node)

SIM.run()
