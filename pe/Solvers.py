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
