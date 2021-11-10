from typing import Tuple, Union
import numpy as np


def point_point_distance(p1: Union[list, np.ndarray], p2: Union[list, np.ndarray]) -> float:
    """Calculates the distance between 2 points

    Args:
        p1 (list): First point
        p2 (list): Second point

    Returns:
        float: Distance between point 1 and point 2
    """
    return np.linalg.norm(np.array(p2)-np.array(p1))


def point_to_line_distance(p: Union[list, np.ndarray], x0: Union[list, np.ndarray], xf: Union[list, np.ndarray]) -> Tuple[float, list]:
    x = p[0]
    y = p[1]

    x1 = x0[0]
    y1 = x0[1]

    x2 = xf[0]
    y2 = xf[1]

    A = x - x1
    B = y - y1
    C = x2 - x1
    D = y2 - y1

    dot = A*C + B*D
    len_sq = C*C + D*D
    param = -1.0
    if not len_sq == 0:
        param = dot / len_sq

    if param < 0:
        xx = x1
        yy = y1
    elif param > 1:
        xx = x2
        yy = y2
    else:
        xx = x1 + param*C
        yy = y1 + param*D

    dx = x - xx
    dy = y - yy
    return (dx*dx + dy*dy)**0.5, np.array([-dx, -dy])
