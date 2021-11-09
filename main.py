import pe
import numpy as np

h = 1.0

SIM = pe.Sim([0, h], [0, h])

wall = pe.Colliders.Wall([0.0, 0.0], [h, 0.0])
SIM.add_collider(wall)
wall = pe.Colliders.Wall([h, 0.0], [h, h])
SIM.add_collider(wall)
wall = pe.Colliders.Wall([h, h], [0.0, h])
SIM.add_collider(wall)
wall = pe.Colliders.Wall([0.0, h], [0.0, 0.0])
SIM.add_collider(wall)
gravity = pe.Force(lambda t, obj: np.array([0, -9.81*obj.m]))
SIM.add_gen_force(gravity)
node = pe.Node(0.5, [0.5, 0.5], [1.0, 1.0])
SIM.add_node(node)

SIM.run()
