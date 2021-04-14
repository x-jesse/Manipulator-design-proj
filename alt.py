import math as m
import numpy as np
from numba import njit

# Given three colinear points p, q, r, the function checks if
# point q lies on line segment 'pr'
@njit(nogil=True)
def onSegment(p_x, p_y, q_x, q_y, r_x, r_y):
    if ((q_x <= max(p_x, r_x)) and (q_x >= min(p_x, r_x)) and
            (q_y <= max(p_y, r_y)) and (q_y >= min(p_y, r_y))):
        return True
    return False

@njit(nogil=True)
def orientation(p_x, p_y, q_x, q_y, r_x, r_y):
    val = (float(q_y - p_y) * (r_x - q_x)) - (float(q_x - p_x) * (r_y - q_y))
    if val > 0:
        return 1
    elif val < 0:
        return 2
    else:
        return 0

# The main function that returns true if
# the line segment 'p_x, p_yq_x, q_y' and 'p2_x, p2_yq2_x, q2_y' intersect.
@njit(nogil=True)
def doIntersect(p_x, p_y, q_x, q_y, p2_x, p2_y, q2_x, q2_y):
    # Find the 4 orientations required for
    # the general and special cases
    o1 = orientation(p_x, p_y, q_x, q_y, p2_x, p2_y)
    o2 = orientation(p_x, p_y, q_x, q_y, q2_x, q2_y)
    o3 = orientation(p2_x, p2_y, q2_x, q2_y, p_x, p_y)
    o4 = orientation(p2_x, p2_y, q2_x, q2_y, q_x, q_y)

    # General case
    if (o1 != o2) and (o3 != o4):
        return True

    # Special Cases
    # p_x, p_y , q_x, q_y and p2_x, p2_y are colinear and p2_x, p2_y lies on segment p_x, p_yq_x, q_y
    if (o1 == 0) and onSegment(p_x, p_y, p2_x, p2_y, q_x, q_y):
        return True
    # p_x, p_y , q_x, q_y and q2_x, q2_y are colinear and q2_x, q2_y lies on segment p_x, p_yq_x, q_y
    if (o2 == 0) and onSegment(p_x, p_y, q2_x, q2_y, q_x, q_y):
        return True
    # p2_x, p2_y , q2_x, q2_y and p_x, p_y are colinear and p_x, p_y lies on segment p2_x, p2_yq2_x, q2_y
    if (o3 == 0) and onSegment(p2_x, p2_y, p_x, p_y, q2_x, q2_y):
        return True
    # p2_x, p2_y , q2_x, q2_y and q_x, q_y are colinear and q_x, q_y lies on segment p2_x, p2_yq2_x, q2_y
    if (o4 == 0) and onSegment(p2_x, p2_y, q_x, q_y, q2_x, q2_y):
        return True
    # If none of the cases
    return False

@njit(nogil=True)
def valid_end_pos(end_point, target):
    if abs(end_point[0] - target[0]) <= 0.01 and abs(end_point[1] - target[1]) <= 0.01:
        return True
    else:
        return False

@njit(nogil=True)
def weight(length, thickness):
    return length * thickness * 9.81

@njit(nogil=True)
def calc_torque(l1, l2, l3, q1, q2, q3):
    # x-components of length vectors used to calculate torque
    l3_x = l3 * m.cos(q3)
    l2_x = l2 * m.cos(q2)
    l1_x = l1 * m.cos(q1)

    # calculate weight of each component based on length and thickness (thickness is constant)
    w1 = weight(l1, 4)
    w2 = weight(l2, 2)
    w3 = weight(l3, 1)

    # calculate torque for each joint
    t1 = (l1_x / 2) * w1
    t2 = (l1_x + l2_x / 2) * w2
    t3 = (l1_x + l2_x + l3_x / 2) * w3

    # calculates torque from 5kg gripper load
    tl = (l1_x + l2_x + l3_x) * 5 * 9.81

    # returns net torque
    return t1 + t2 + t3 + tl

@njit(nogil=True)
def main():
    origin = (0, 0)

    start_length1, length_range1, start_length2, length_range2, start_length3, length_range3 = 60, 70, 1, 100, 1, 100
    angle_range = 360
    final_l1, final_l2, final_l3 = 0, 0, 0
    final_torque = 9999

    for i in range(start_length1, length_range1):
        for ii in range(start_length2, length_range2):
            for iii in range(start_length3, length_range3):
                if i + ii + iii < 100:
                    continue

                l1, l2, l3 = [n / 100 for n in [i, ii, iii]]
                t1, t2, t3 = 999, 999, 999

                for q1 in range(angle_range):
                    for q2 in range(angle_range):
                        case1, case2, case3 = False, False, False

                        x1, y1 = l1 * m.cos(m.radians(q1)), l1 * m.sin(m.radians(q1))
                        x2, y2 = l2 * m.cos(m.radians(q2)), l2 * m.sin(m.radians(q2))
                        joint1, joint2 = (x1, y1), (x1+x2, y1+y2)

                        # CASE X = 0.75M, Y = 0.1M, THETA = -60 WRT X
                        if not case1:
                            q3 = m.radians(-60)
                            x3, y3 = l3 * m.cos(q3), l3 * m.sin(q3)
                            endpoint = (x1+x2+x3, y1+y2+y3)
                            # print(endpoint)
                            target = (0.75, 0.1)

                            if valid_end_pos(endpoint, target) and not doIntersect(origin[0], origin[1], joint1[0], joint1[1], joint2[0], joint2[1], endpoint[0], endpoint[1]):
                                # print("t1", endpoint)
                                t1 = calc_torque(l1, l2, l3, q1, q2, q3)
                                case1 = True
                                continue

                        # CASE X = 0.5M, Y = 0.5M, THETA = 0 WRT X
                        if not case2:
                            q3 = 0
                            x3, y3 = l3 * m.cos(q3), l3 * m.sin(q3)
                            endpoint = (x1+x2+x3, y1+y2+y3)
                            target = (0.5, 0.5)

                            # print(valid_end_pos(endpoint, target))
                            if valid_end_pos(endpoint, target) and not doIntersect(origin[0], origin[1], joint1[0], joint1[1], joint2[0], joint2[1], endpoint[0], endpoint[1]):
                                # print("t2", endpoint)
                                t2 = calc_torque(l1, l2, l3, q1, q2, q3)
                                case2 = True
                                continue

                        # CASE X = 0.2M, Y = 0.6M, THETA = 45 WRT X
                        if not case3:
                            q3 = m.radians(45)
                            x3, y3 = l3 * m.cos(q3), l3 * m.sin(q3)
                            endpoint = (x1+x2+x3, y1+y2+y3)
                            target = (0.2, 0.6)

                            # print(valid_end_pos(endpoint, target))
                            if valid_end_pos(endpoint, target) and not doIntersect(origin[0], origin[1], joint1[0], joint1[1], joint2[0], joint2[1], endpoint[0], endpoint[1]):
                                # print("t3", endpoint)
                                t3 = calc_torque(l1, l2, l3, q1, q2, q3)
                                case3 = True

                if case1 and case2 and case3:
                    torque = m.sqrt(t1**2+t2**2+t3**2)
                    print(torque)
                    if torque < final_torque:
                        final_l1, final_l2, final_l3 = l1, l2, l3
                        final_torque = torque
                        print("T=", final_torque, "l1=", final_l1, "l2=", final_l2, "l3=", final_l3)


main()

