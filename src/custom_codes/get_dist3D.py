from numpy import array, cos, sin, dot, sqrt, asarray, sqrt
import numpy as np
"""
    Calculate the distance between obstacle and robot's floating control point (repulsive)
    """

def get_dist3D(A, B, P):
        A = asarray(A)
        B = asarray(B)
        P = asarray(P)

        n = B - A
        pa = A - P

        c = dot(n, pa)

        # Closest point is a
        if c > 0:
            # print("Valor mais proximo de A")
            return A, sqrt(dot(pa, pa))

        bp = P - B

        # Closest point is b
        if dot(n, bp) > 0:
            # print("Valor mais proximo de B")
            return B, sqrt(dot(bp, bp))

        # Closest point is between a and b
        e = pa - n * (c / dot(n, n))
        aq = - pa + e
        Q = aq + A

        return Q, sqrt(dot(e, e))
