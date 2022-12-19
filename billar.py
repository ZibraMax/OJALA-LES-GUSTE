import pe
import numpy as np

h = 1.0

G = 6.67408*10**-2


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

#force = pe.Force(gravity_newton)


SIM = pe.Sim([0, h], [0, h], dt=1/60)

# SIM.add_gen_force(force)
wall = pe.Colliders.Wall([0.0, 0.0], [h, 0.0])
SIM.add_collider(wall)
wall = pe.Colliders.Wall([h, 0.0], [h, h])
SIM.add_collider(wall)
wall = pe.Colliders.Wall([h, h], [0.0, h])
SIM.add_collider(wall)
wall = pe.Colliders.Wall([0.0, h], [0.0, 0.0])
SIM.add_collider(wall)

masa = 0.3
r = 0.05
node = pe.Node(masa, (0.2, 0.6), [0.1, 0.0], r=r)
SIM.add_node(node)

node = pe.Node(masa, (0.5, 0.6), [0.0, 0.0], r=r)
SIM.add_node(node)

node = pe.Node(masa, (0.5+2*r, 0.6), [0.0, 0.0], r=r)
SIM.add_node(node)

SIM.run()
