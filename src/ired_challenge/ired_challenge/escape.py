"""Getting the robot out when it is stuck inside an obstacle's inflation.

If AMCL error or a tight gap leaves the robot centre within the robot radius of
a wall or box, every Nav2 plan fails at once ("start occupied") and Nav2's own
recoveries (spin, back up) refuse to move because they see a collision. Seen in
the PC sim: BS in C_Z22 next to the START fence, 0 zones in 10 min. The way out
is a short, slow slide directly away from the nearby obstacle points (an omni
base can move in any direction without turning)."""
import math


def escape_direction(points, radius=0.45):
    """Unit (x, y) in the robot frame pointing away from the obstacle points
    (robot frame, metres) closer than `radius`, nearer points weighing more.
    None if nothing is that close."""
    sx = sy = 0.0
    for x, y in points:
        d = math.hypot(x, y)
        if 1e-3 < d < radius:
            w = (radius - d) / d
            sx -= x * w
            sy -= y * w
    n = math.hypot(sx, sy)
    if n < 1e-6:
        return None
    return sx / n, sy / n


def nearest(points):
    return min((math.hypot(x, y) for x, y in points), default=math.inf)
