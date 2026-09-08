from math import pi

def wrap(theta):
    """Wrap an angle to the range [-pi, pi]."""
    if theta > pi:
        theta = -pi + theta % (pi)
    if theta < -pi:
        theta = pi - theta % (pi)
    return theta