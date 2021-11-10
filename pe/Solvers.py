from typing import Callable


class Solver():
    """ODE 1st order equation solver
    """

    def __init__(self):
        """ODE 1st order equation solver
        """
        pass

    def solve_euler(self, f: Callable, x0: float, xf: float, y0: float, n: int) -> float:
        """Solves tehe differential equation y'=f(x,y)

        Args:
            f (Callable): Derivative function
            x0 (float): Initial point for solution
            xf (float): End point for solution
            y0 (float): Initial point of the dependent variable (Border condition)
            n (int): Number of iterations

        Returns:
            float: Dependent variable solution (y) for the final point xf
        """
        h = (xf-x0)/n
        xi = x0
        yi = y0
        for _ in range(n):
            yi += f(xi, yi)*h
            xi += h
        return yi

    def solve(self, *args) -> float:
        return self.solve_euler(*args)
