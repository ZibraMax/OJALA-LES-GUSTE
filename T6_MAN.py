import pe
import numpy as np

from pe.Sim import Spring


h = 8.0

SIM = pe.Sim([0, h], [0, h], frames_folder='frames_t6')

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


gravity = pe.Force(lambda t, obj: np.array([0, -9.81*obj.m]))


SIM.add_gen_force(gravity)
node1 = pe.Node(2/9.81, [h/2, h/2], [0.0, 0.0], r=0.3)
node1.fix()
node2 = pe.Node(2/9.81, [h/2+0.19966683, h/2-1.99000833],
                [0.19900083, -0.01996668], r=0.3, trace=-1)
SIM.add_node(node1)
SIM.add_node(node2)
k = 5
d = 0
spring = Spring(k, d, node1, node2)
SIM.add_spring(spring)


# node2 = pe.Node(0.5, [0.6, 0.58], [0.0, 0.0])
# SIM.add_node(node2)
# k = 100
# d = 0.25
# spring = pe.Spring(k, d, node1, node2)
# SIM.add_spring(spring)

SIM.run()
