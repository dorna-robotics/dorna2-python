import numpy as np


def solve_cs_equation(aa, bb, cc):
    """
    Solves the equation aa + bb * cos(θ) + cc * sin(θ) = 0
    for θ, returns (cos(θ), sin(θ)) if a solution exists, otherwise None.

    i = 0 or 1 to choose between the two possible solutions
    """
    delta = cc * cc * (-aa * aa + bb * bb + cc * cc)

    if delta < 0.0:
        return None

    if np.isclose(bb,0.0) and np.isclose(cc,0.0):
        return None

    if np.isclose(bb,0.0):
        s1 = -aa / cc
        if abs(s1) > 1.0:
            return None
        c1 = np.sqrt(max(1.0 - s1 * s1,0))
        return [[c1,s1],[-c1,s1]]

    if np.isclose(cc,0.0):
        c1 = -aa / bb
        if abs(c1) > 1.0:
            return None
        s1 = np.sqrt(max(0.,1.0 - c1 * c1))
        return [[c1, s1],[c1,-s1]]

    denom = bb * bb + cc * cc
    sqrt_delta = np.sqrt(delta)

    c1 = (-aa * bb + sqrt_delta) / denom
    c12 = (-aa * bb - sqrt_delta) / denom

    s1 = -(aa + bb * c1) / cc
    s12 = -(aa + bb * c12) / cc

    return [[c1, s1],[c12,s12]]
