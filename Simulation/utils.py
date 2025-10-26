import math

def clamp(v:float, lo:float, hi:float) -> float:
    return max(lo, min(hi, v))

def normalize(x:float, y:float) -> tuple[float, float]:
    n = math.hypot(x, y)
    if n < 1e-9:
        return 1.0, 0.0
    return x / n, y / n

def reflect_point(x, y, *, left=None, right=None, top=None, bottom=None):
    rx, ry = x, y
    if left is not None and x < left:
        rx = 2 * left - x
    if right is not None and x > right:
        rx = 2 * right - x
    if top is not None and y < top:
        ry = 2 * top - y
    if bottom is not None and y > bottom:
        ry = 2 * bottom - y
    return rx, ry

def rotate90(x, y, direction=1):
    return (-y, x) if direction >= 0 else (y, -x)