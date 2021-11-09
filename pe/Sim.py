from .Region import Region


class Sim():
    """docstring for SIM
    """

    def __init__(self):
        self.region = Region()

    def run(self):
        self.region.run()


class Node():
    """docstring for Node
    """

    def __init__(self, U, V):
        self.U = U
        self.V = V
