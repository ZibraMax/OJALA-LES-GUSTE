import pe
import numpy as np

from pe.Sim import Spring


h = 2.0

SIM = pe.Sim([0, h], [0, h])

wall = pe.Colliders.Wall([0.0, 0.0], [h, 0.0])
SIM.add_collider(wall)
wall = pe.Colliders.Wall([h, 0.0], [h, h])
SIM.add_collider(wall)
wall = pe.Colliders.Wall([h, h], [0.0, h])
SIM.add_collider(wall)
wall = pe.Colliders.Wall([0.0, h], [0.0, 0.0])
SIM.add_collider(wall)

# wall = pe.Colliders.Wall([0.5, 0], [h, 1])
# SIM.add_collider(wall)

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


force = pe.Force(gravity_newton)

gravity = pe.Force(lambda t, obj: np.array([0, -9.81*obj.m]))


# SIM.add_gen_force(force)
SIM.add_gen_force(gravity)
n = 7
m = 7
r = 0.04
d = 2.5*r
x0 = 1
y0 = 1
for i in range(n):
    for j in range(m):
        node1 = pe.Node(1, [x0+i*d, y0+j*d], [0.0, 0.0], r=r)
        SIM.add_node(node1)
nodes = SIM.nodes
k = 10000
d = 5
for i in range(n-1):
    for j in range(m):
        f = int(j*n+i)
        spring = Spring(k, d, nodes[f], nodes[f+1])
        SIM.add_spring(spring)

for i in range(n):
    for j in range(m-1):
        f = int(j*n+i)
        f2 = int((j+1)*n+i)
        spring = Spring(k, d, nodes[f], nodes[f2])
        SIM.add_spring(spring)


for i in range(n-1):
    for j in range(m-1):
        f1 = int(j*n+i)
        f2 = int(j*n+i)+1

        f3 = int((j+1)*n+i+1)
        f4 = int((j+1)*n+i)

        spring1 = Spring(k, d, nodes[f1], nodes[f3])
        spring2 = Spring(k, d, nodes[f2], nodes[f4])
        SIM.add_spring(spring1)
        SIM.add_spring(spring2)


# node2 = pe.Node(0.5, [0.6, 0.58], [0.0, 0.0])
# SIM.add_node(node2)
# k = 100
# d = 0.25
# spring = pe.Spring(k, d, node1, node2)
# SIM.add_spring(spring)

SIM.run()
