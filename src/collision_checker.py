#!/usr/bin/env python3
"""
Collision checker (аналитическая проверка расстояния между TCP)
"""

import numpy as np


def segment_distance(p1, p2, q1, q2):
    """
    Минимальное расстояние между двумя отрезками [p1,p2] и [q1,q2]
    """
    u = np.array(p2) - np.array(p1)
    v = np.array(q2) - np.array(q1)
    w0 = np.array(p1) - np.array(q1)

    a = np.dot(u, u)
    b = np.dot(u, v)
    c = np.dot(v, v)
    d = np.dot(u, w0)
    e = np.dot(v, w0)

    D = a * c - b * b
    sc, sN, sD = D, D, D
    tc, tN, tD = D, D, D

    if D < 1e-9:
        sN = 0.0
        sD = 1.0
        tN = e
        tD = c
    else:
        sN = (b * e - c * d)
        tN = (a * e - b * d)
        if sN < 0:
            sN = 0
            tN = e
            tD = c
        elif sN > sD:
            sN = sD
            tN = e + b
            tD = c

    if tN < 0:
        tN = 0
        if -d < 0:
            sN = 0
        elif -d > a:
            sN = sD
        else:
            sN = -d
            sD = a
    elif tN > tD:
        tN = tD
        if (-d + b) < 0:
            sN = 0
        elif (-d + b) > a:
            sN = sD
        else:
            sN = (-d + b)
            sD = a

    sc = 0.0 if abs(sN) < 1e-9 else sN / sD
    tc = 0.0 if abs(tN) < 1e-9 else tN / tD

    dP = w0 + (sc * u) - (tc * v)
    return np.linalg.norm(dP)


def check_collision(path1, path2, safe_dist=0.1):
    """
    path1, path2: списки точек [(x,y,z), ...]
    safe_dist: минимальное допустимое расстояние
    """
    for i in range(len(path1) - 1):
        for j in range(len(path2) - 1):
            d = segment_distance(path1[i], path1[i+1], path2[j], path2[j+1])
            if d < safe_dist:
                return True
    return False


if __name__ == "__main__":
    path1 = [(0, 0, 0), (1, 0, 0)]
    path2 = [(0.5, 0.1, 0), (0.5, -0.1, 0)]
    print("Collision?", check_collision(path1, path2))
