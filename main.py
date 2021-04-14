import math

######################################
# MANIPULATOR DESIGN PROJECT MTE 119 #
######################################


def torque(l1, l2, l3, x, y, q3):
    delta_x, delta_y, = x - l3*math.cos(q3), y - l3*math.sin(q3)

    try:
        q2 = math.acos((delta_x**2+delta_y**2-l1**2-l2**2)/(2*l1*l2))
        q1 = math.atan(delta_y/delta_x) - math.atan(l2*math.sin(q2)/(l1-l2*math.cos(q2)))
    except ValueError:
        # print("breaks", l1, l2, l3)
        return 999, 0, 0
    else:
        t = 0.5*9.81*(l1*4*l1*math.sin(q1) + l2*2*l2*math.sin(q2) + l3*l3*5*math.sin(q3))
        return t, q1, q2

# TORQUE CALCULATIONS

# LET Q1, Q2, Q3 BE THE JOINT ANGLES, l1, l2, l3 are the lengths

# CASE X = 0.75M, Y = 0.1M, THETA = -60 WRT X
x1, y1 = 0.75, 0.1

# CASE X = 0.5M, Y = 0.5M, THETA = 0 WRT X
x2, y2 = 0.5, 0.5

# CASE X = 0.2M, Y = 0.6M, THETA = 45 WRT X
x3, y3 = 0.2, 0.6

cases = [(x1, y1, math.radians(-60)), (x2, y2, 0), (x3, y3, math.radians(45))]

min_torque = 100
a, b, c = 0, 0, 0
q1, q2, q3 = 0, 0, 0

for l in range(50):
    l3 = l/100
    for ll in range(50):
        l2 = ll/100
        for lll in range(50):
            l1 = lll/100
            if l + ll + lll < 100:
                continue

            # print(l1, l2, l3)
            torque_val = 999
            for case in cases:
                x, y, q3 = case
                temp_torque, a1, a2 = torque(l1, l2, l3, x, y, q3)
                torque_val += temp_torque**2

            if abs(torque_val)**0.5 < min_torque:
                min_torque = torque_val ** 0.5
                a, b, c = l1, l2, l3
                q1, q2 = a1, a2

    print(min_torque, "l1 =", a, "q1 =", q1, "l2 =", b, "q2 =", q2, "l3 =", c)
