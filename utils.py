from math import fmod, pi


def normalize_angle(angle):
    result = fmod(angle + pi, 2.0 * pi)
    if result <= 0.0:
        return result + pi
    return result - pi
