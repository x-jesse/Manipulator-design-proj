import math

######################################
# MANIPULATOR DESIGN PROJECT MTE 119 #
######################################


# TORQUE CALCULATIONS


# def torque(l1, l2, l3, x, y, q3, delta_x, delta_y):
#     try:
#         q2 = math.acos((delta_x**2+delta_y**2-l1**2-l2**2)/float(2*l1*l2))
#         q1 = math.atan(delta_y/delta_x) - math.atan(l2*math.sin(q2)/(l1-l2*math.cos(q2))) + math.pi/2
#         q2 = math.pi/2-(q1+q2)
#         q3 = math.pi/2-(q1+q2+q3)
#     except ValueError:
#         # print("breaks", l1, l2, l3)
#         return 999, 0, 0
#     else:
#         t = 0.5*9.81*(l1*4*l1*math.sin(q1) + l2*2*l2*math.sin(q2) + l3*l3*5*math.sin(q3))
#         return t, q1, q2

def weight(length, thickness):
    return length * thickness * 9.81


def new_torque(l1, l2, l3, delta_x, delta_y, q3):
    # redefine delta_x, delta_y as new x, y
    x, y = delta_x, delta_y  # this gives the position of the second joint, which is necessary for angle calculations

    # this sometimes doesn't work, prob bc the lengths are too long/short and its not physically possible to get a solution
    try:
        # joint angles given by inverse kinematic equations
        q2 = math.acos((x ** 2 + y ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2))
        q1 = math.atan(y / x) - math.atan(l2 * math.sin(q2) / (l1 - l2 * math.cos(q2)))

    except ValueError:  # in the event that it breaks, return 999 and move on
        return 999

    # x-components of length vectors used to calculate torque
    l3_x = l3 * math.cos(q3)
    l2_x = l2 * math.cos(q2)
    l1_x = l1 * math.cos(q1)

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


# LET Q1, Q2, Q3 BE THE JOINT ANGLES, l1, l2, l3 are the lengths

# CASE X = 0.75M, Y = 0.1M, THETA = -60 WRT X
x1, y1 = 0.75, 0.1

# CASE X = 0.5M, Y = 0.5M, THETA = 0 WRT X
x2, y2 = 0.5, 0.5

# CASE X = 0.2M, Y = 0.6M, THETA = 45 WRT X
x3, y3 = 0.2, 0.6

# combines cases into one list
cases = [(x1, y1, math.radians(-60)), (x2, y2, math.radians(0)), (x3, y3, math.radians(45))]

# initializes values
min_torque = 100
final_l1, final_l2, final_l3 = 0, 0, 0
q1, q2, q3 = 0, 0, 0
lengths_range = 50

# loops for lengths in lengths_range, testing all possible combinations of l1, l2, l3
for l in range(1, lengths_range):
    l3 = l / 100
    for ll in range(1, lengths_range):
        l2 = ll / 100
        for lll in range(1, lengths_range):
            l1 = lll / 100
            if l + ll + lll < 100:  # lengths cannot sum to less than 1m
                continue

            torque_val = 0  # initializes torque value

            for case in range(3): # tests each case
                x, y, q3 = cases[case]

                # each case has different x, y position depending on the direction of the third joint
                # this might need fixing, does not account for all possible cases
                # (ie. multiple different arrangements of joints could result in same end position but different torque)
                # also does not consider overlapping arms
                if case == 1:
                    delta_x, delta_y = x - l3 * math.cos(q3), y - l3 * math.sin(q3)
                elif case == 2:
                    delta_x, delta_y = x - l3 * math.cos(q3), 0
                else:
                    delta_x, delta_y = x - l3 * math.cos(q3), y - l3 * math.sin(q3)

                # really janky solution in the case that the third joint passes through the origin (obv not allowed)
                if delta_y == 0 or delta_x == 0:
                    break

                # gets the torque, stores its square
                temp_torque = new_torque(l1, l2, l3, delta_x, delta_y, q3)
                torque_val += temp_torque ** 2

            if abs(torque_val) ** 0.5 < min_torque:
                min_torque = torque_val ** 0.5
                final_l1, final_l2, final_l3 = l1, l2, l3
                # q1, q2 = a1, a2

    # print("T =", min_torque, "l1 =", final_l1, "q1 =", q1, "l2 =", final_l2, "q2 =", q2, "l3 =", final_l3, "q3 =", q3)
    print("T =", min_torque, "l1 =", final_l1, "l2 =", final_l2, "l3 =", final_l3)

# T = 0.0005142084671561576 l1 = 0.65 q1 = 1.797457664136724 l2 = 0.57 q2 = -2.089439581786089 l3 = 0.5 q3 = 0.7853981633974483

